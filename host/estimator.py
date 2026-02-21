#!/usr/bin/env python3
"""
estimator.py -- Per-Rep Dead-Reckoning VBT Estimator.

Architecture informed by validated VBT devices (VmaxPro, GymAware) and
2024-2026 research (Zhang et al. 2024, Haj Lotfalian 2026, Wannouch et al.
2025 systematic review):

  * **Orientation-only EKF** -- 7-state [q(4), bg(3)] gyro propagation
    with accelerometer gravity-reference correction.  Used solely to
    rotate accelerometer data into the world frame for proper gravity
    removal.  *No velocity in the EKF* -- eliminates continuous
    integration drift entirely.

  * **Displacement-reversal rep detection** -- reps are detected by
    finding peaks and valleys in the continuous displacement signal.
    This works for ALL exercise types including touch-and-go reps,
    continuous tempo sets, and exercises without true zero-velocity
    phases (bench press, pendlay rows, etc.).

  * **Phase segmentation** -- each rep is split into concentric (up) and
    eccentric (down) phases via displacement reversal points.  Mean
    Concentric Velocity (MCV) and Peak Concentric Velocity (PCV) are the
    gold-standard VBT metrics.

  * **Per-rep linear drift correction** -- applied between detected
    reversal points, linearly distributing residual drift.

  * **Multi-criteria rep validation** -- minimum ROM, minimum duration,
    and minimum peak velocity thresholds reject noise, vibration, and
    repositioning moves that are NOT actual reps.

  * **Butterworth low-pass on accel** -- 20 Hz 2nd-order IIR removes
    sensor noise without lagging the velocity waveform excessively.

State vector  (7)
    q  = [q0, q1, q2, q3]   orientation quaternion  (scalar-first)
    bg = [bgx, bgy, bgz]    gyro bias  (deg/s)

Velocity & position are NOT part of the EKF -- they are computed via
continuous integration with adaptive drift correction.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .calibration import CalibrationResult
from .imu_driver import IMUSample

# -- Physical constants -------------------------------------------------------
G_MPS2 = 9.80665          # m/s^2
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# Velocity magnitude clamp -- no human movement exceeds this (m/s)
_VZ_CLAMP = 5.0

# Low-acceleration threshold for soft velocity decay (m/s^2)
# When |az| is below this, velocity decays even if ZUPT says "moving"
_LOW_ACCEL_THRESH = 0.3

# -- State indices -------------------------------------------------------------
_SQ  = slice(0, 4)     # quaternion
_SBG = slice(4, 7)     # gyro bias
_NX  = 7               # total state dimension


# -- Quaternion helpers (scalar-first: q = [w, x, y, z]) ----------------------

def _qnorm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product  a * b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector *v* by quaternion *q*:  v' = q * [0,v] * q*."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return _qmul(_qmul(q, qv), qc)[1:]


def _qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _q2dcm(q: np.ndarray) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix  (body->world)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def _omega_matrix(w: np.ndarray) -> np.ndarray:
    """Omega(w) for quaternion derivative:  q_dot = 0.5 * Omega(w) * q."""
    wx, wy, wz = w
    return np.array([
        [ 0,  -wx, -wy, -wz],
        [ wx,  0,   wz, -wy],
        [ wy, -wz,  0,   wx],
        [ wz,  wy, -wx,  0 ],
    ])


def _skew(v: np.ndarray) -> np.ndarray:
    """3-vector -> skew-symmetric matrix [v]x."""
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ])


# -- Butterworth 2nd-order IIR low-pass ----------------------------------------

class Butter2LP:
    """
    2nd-order Butterworth low-pass filter (Direct Form II Transposed).

    Designed for 3-axis signals.  Coefficients computed at construction
    from cutoff frequency and sample rate.
    """
    def __init__(self, fc: float, fs: float, n_ch: int = 3):
        wc = math.tan(math.pi * fc / fs)
        wc2 = wc * wc
        k1 = math.sqrt(2.0) * wc
        k2 = wc2
        a0 = 1.0 + k1 + k2

        self.b0 = k2 / a0
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (k2 - 1.0) / a0
        self.a2 = (1.0 - k1 + k2) / a0

        self.z1 = np.zeros(n_ch)
        self.z2 = np.zeros(n_ch)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y


# -- SHOE-based ZUPT detector -------------------------------------------------

class ZUPTDetector:
    """
    Zero-Velocity Update detector using SHOE test statistic.

    SHOE (Stance Hypothesis Optimal dEtection) from Skog et al. (2010)
    uses a variance-based test over a sliding window.  A single outlier
    sample no longer vetoes the entire window.
    """
    def __init__(self, window: int = 20,
                 accel_thresh: float = 0.06,
                 gyro_thresh: float = 2.5):
        self.window = window
        self.accel_thresh = accel_thresh
        self.gyro_thresh  = gyro_thresh
        self._accel_buf: list[float] = []
        self._gyro_buf:  list[float] = []

    def update(self, accel_mag: float, gyro_mag: float) -> bool:
        """Return True if sensor is currently stationary."""
        self._accel_buf.append(accel_mag)
        self._gyro_buf.append(gyro_mag)
        if len(self._accel_buf) > self.window:
            self._accel_buf.pop(0)
            self._gyro_buf.pop(0)
        if len(self._accel_buf) < self.window:
            return True

        a_arr = np.array(self._accel_buf)
        a_var = float(np.mean((a_arr - 1.0) ** 2))

        g_arr = np.array(self._gyro_buf)
        g_var = float(np.mean(g_arr ** 2))

        a_ok = a_var < self.accel_thresh ** 2
        g_ok = g_var < self.gyro_thresh ** 2
        return a_ok and g_ok


# -- Rep metrics ---------------------------------------------------------------

@dataclass
class RepMetrics:
    """Per-rep metrics computed after a complete rep."""
    rep_number: int
    mean_concentric_velocity: float   # MCV (m/s)
    peak_concentric_velocity: float   # PCV (m/s)
    mean_eccentric_velocity: float    # MEV (m/s)
    peak_eccentric_velocity: float    # PEV (m/s)
    concentric_rom: float             # range of motion (m)
    eccentric_rom: float              # range of motion (m)
    total_rom: float                  # total vertical displacement (m)
    concentric_duration: float        # seconds
    eccentric_duration: float         # seconds
    rep_duration: float               # seconds
    mean_velocity: float              # overall mean |v_z| during rep
    peak_velocity: float              # overall peak |v_z| during rep


# -- Estimator output ----------------------------------------------------------

@dataclass
class EstimatorState:
    """Readable snapshot of the estimator output per sample."""
    t: float
    q: np.ndarray
    euler: np.ndarray                  # [roll, pitch, yaw]  (deg)
    velocity: np.ndarray               # [vx, vy, vz]  world frame  (m/s)
    position: np.ndarray               # [px, py, pz]  world frame  (m)
    accel_world: np.ndarray            # linear accel world frame (m/s^2)
    speed: float                       # |v|  (m/s)
    vertical_velocity: float           # v_z  (m/s) -- primary VBT signal
    vertical_displacement: float       # z displacement from rep start (m)
    is_stationary: bool
    is_moving: bool                    # in a movement window
    gyro_bias: np.ndarray
    accel_bias: np.ndarray
    rep_metrics: Optional[RepMetrics] = None


# -- Per-Rep Dead-Reckoning VBT Estimator --------------------------------------

class IMUEstimator:
    """
    VBT estimator: orientation-only EKF + per-rep dead-reckoning.

    Orientation: 7-state EKF [q(4), bg(3)].
    Velocity: per-rep trapezoidal integration with drift correction.
    """

    def __init__(self,
                 cal: CalibrationResult,
                 dt_nominal: float = 1.0 / 446.0,
                 sigma_gyro: float = 0.3,
                 sigma_accel: float = 0.03,
                 sigma_gyro_bias: float = 0.0005,
                 lp_cutoff: float = 20.0,
                 min_rom: float = 0.05,
                 min_rep_duration: float = 0.25,
                 min_peak_velocity: float = 0.08,
                 reversal_smooth_window: int = 15,
                 rest_drift_rate: float = 3.0):
        self.cal = cal
        self.dt_nominal = dt_nominal

        # -- Rep validation thresholds --
        self.min_rom = min_rom                       # metres
        self.min_rep_duration = min_rep_duration      # seconds
        self.min_peak_velocity = min_peak_velocity    # m/s
        self.reversal_smooth_window = reversal_smooth_window
        self.rest_drift_rate = rest_drift_rate        # velocity decay at rest (1/s)

        # -- Orientation state --
        self._init_orientation_from_gravity(cal.gravity_dir)
        self.bg = cal.gyro_bias.copy()

        # -- Covariance (7x7) --
        self.P = np.diag([
            1e-4, 1e-4, 1e-4, 1e-4,
            1e-2, 1e-2, 1e-2,
        ])

        # -- Noise tuning --
        self.Q_gyro  = (sigma_gyro * DEG2RAD) ** 2
        self.Q_gbias = (sigma_gyro_bias * DEG2RAD) ** 2
        self.R_accel_base = (sigma_accel * G_MPS2) ** 2

        # -- ZUPT --
        self.zupt = ZUPTDetector()
        self.is_stationary = True

        # -- Low-pass filter for accel --
        fs = 1.0 / dt_nominal
        self.accel_lp = Butter2LP(lp_cutoff, fs, n_ch=3)

        # -- Continuous velocity/displacement (not gated by ZUPT) --
        self.v = np.zeros(3)
        self.pos = np.zeros(3)
        self._vz = 0.0         # vertical velocity (continuously integrated)
        self._dz = 0.0         # vertical displacement
        self._in_movement = False

        # -- Displacement-reversal rep detector --
        #   Reps are detected by finding peaks and valleys in the
        #   smoothed vertical displacement signal.  A full rep is
        #   a valley->peak->valley or peak->valley->peak cycle.
        self._vz_buf: list[float] = []     # velocity history
        self._t_buf: list[float] = []      # time history
        self._az_buf: list[float] = []     # accel history
        self._disp_buf: list[float] = []   # raw displacement history
        self._smooth_dz_buf: list[float] = []  # smoothed displacement

        # Reversal tracking:  we need 3 consecutive alternating reversals
        # to form one full rep (e.g. valley -> peak -> valley).
        self._reversals: list[tuple[int, int, float]] = []  # (buf_idx, type +1/-1, smooth_dz)
        self._reversal_hysteresis = 0.015  # metres: ignore reversals < this from last

        # -- Rep tracking --
        self._rep_count = 0

        # -- Drift correction state --
        self._rest_samples = 0
        self._vz_bias = 0.0    # estimated velocity bias (slow drift)

        # -- Bookkeeping --
        self.t_prev: float | None = None

    # -- Initialisation helpers ------------------------------------------------

    def _init_orientation_from_gravity(self, g_dir: np.ndarray) -> None:
        """Set initial quaternion so world Z aligns with gravity."""
        gn = g_dir / (np.linalg.norm(g_dir) + 1e-12)
        target = np.array([0.0, 0.0, -1.0])
        dot = np.clip(np.dot(gn, target), -1.0, 1.0)
        cross = np.cross(gn, target)
        cn = np.linalg.norm(cross)

        if cn < 1e-6:
            if dot > 0:
                self.q = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                self.q = np.array([0.0, 1.0, 0.0, 0.0])
        else:
            axis = cross / cn
            angle = math.acos(dot)
            s = math.sin(angle / 2)
            self.q = np.array([math.cos(angle/2), axis[0]*s, axis[1]*s, axis[2]*s])

        self.q = _qnorm(self.q)

    # -- Public interface ------------------------------------------------------

    def update(self, sample: IMUSample) -> EstimatorState:
        """Process one IMU sample.  Returns current state."""
        # -- Time step --
        if self.t_prev is None:
            self.t_prev = sample.t
            dt = self.dt_nominal
        else:
            dt = sample.t - self.t_prev
            if dt <= 0 or dt > 0.1:
                dt = self.dt_nominal
            self.t_prev = sample.t

        # -- Sensor readings (bias-corrected) --
        gyro_raw = np.array([sample.gx, sample.gy, sample.gz])
        accel_raw = np.array([sample.ax, sample.ay, sample.az])

        gyro = (gyro_raw - self.bg) * DEG2RAD
        accel_cal = accel_raw - self.cal.accel_bias

        # -- ZUPT detection --
        accel_mag = np.linalg.norm(accel_cal)
        gyro_mag  = np.linalg.norm(gyro_raw - self.bg)
        self.is_stationary = self.zupt.update(accel_mag, gyro_mag)

        # -- EKF PREDICT (orientation only) --
        self._predict_orientation(gyro, dt)

        # -- EKF CORRECT (gravity reference) --
        self._correct_accel(accel_cal)

        # -- GRAVITY REMOVAL --
        R = _q2dcm(self.q)
        accel_world = R @ (accel_cal * G_MPS2)
        accel_world[2] += G_MPS2  # remove gravity (Z up)

        # Low-pass filter
        accel_world_filt = self.accel_lp(accel_world)

        # -- CONTINUOUS VELOCITY INTEGRATION with adaptive drift control --
        az = accel_world_filt[2]
        rep_metrics = self._integrate_and_detect(az, sample.t, dt)

        # Update 3D velocity/position for display
        self.v[2] = self._vz
        self.pos[2] = self._dz
        self._in_movement = not self.is_stationary

        return EstimatorState(
            t=sample.t,
            q=self.q.copy(),
            euler=self._q2euler(self.q),
            velocity=self.v.copy(),
            position=self.pos.copy(),
            accel_world=accel_world_filt.copy(),
            speed=float(np.linalg.norm(self.v)),
            vertical_velocity=self._vz,
            vertical_displacement=self._dz,
            is_stationary=self.is_stationary,
            is_moving=self._in_movement,
            gyro_bias=self.bg.copy(),
            accel_bias=self.cal.accel_bias.copy(),
            rep_metrics=rep_metrics,
        )

    # -- Continuous integration + displacement-reversal rep detection -----------

    def _integrate_and_detect(self, az: float, t: float,
                              dt: float) -> Optional[RepMetrics]:
        """
        Continuously integrate acceleration -> velocity -> displacement.
        Detect reps via displacement reversals (peaks/valleys).

        This replaces the old ZUPT-gated dead-reckoning and works for:
        - Touch-and-go reps (no rest between reps)
        - Continuous tempo sets
        - All exercise types (squat, bench, deadlift, rows, etc.)

        Drift control (multi-layer):
        1. ZUPT rest: aggressive exponential decay + bias learning
        2. Low-accel soft decay: decays velocity when bar isn't
           accelerating, even if ZUPT hasn't triggered yet
        3. Velocity clamp: hard limit at ±5 m/s (physical sanity)
        4. Full bias subtraction during integration

        Rep detection:
        - Smooth displacement with moving-average window
        - Detect peaks (+1) and valleys (-1) with hysteresis
        - A full rep = 3 consecutive alternating reversals forming a
          concentric + eccentric cycle (or vice-versa)
        - Reversals are CONSUMED after rep detection to prevent
          double-counting
        """
        rep_metrics = None

        # -- Drift control --
        if self.is_stationary:
            # Hard ZUPT: aggressive decay
            self._rest_samples += 1
            decay = math.exp(-self.rest_drift_rate * dt)
            self._vz *= decay
            # Learn velocity bias from rest (slow EMA)
            alpha = min(0.02, 1.0 / max(self._rest_samples, 1))
            self._vz_bias = (1.0 - alpha) * self._vz_bias + alpha * az
            # Snap to zero once very small
            if abs(self._vz) < 0.002:
                self._vz = 0.0
            # Decay displacement drift after sustained rest
            if self._rest_samples > 100:
                self._dz *= decay
        else:
            self._rest_samples = 0
            # Integrate with full bias subtraction
            self._vz += (az - self._vz_bias) * dt

        # Soft decay: when acceleration is very low (bar not accelerating),
        # apply mild velocity decay even if ZUPT hasn't triggered.
        # This prevents runaway drift from tiny gravity-removal errors.
        if not self.is_stationary and abs(az) < _LOW_ACCEL_THRESH:
            soft_decay = math.exp(-1.0 * dt)  # mild 1/s decay
            self._vz *= soft_decay

        # Hard velocity clamp -- no human exercise exceeds ±5 m/s
        self._vz = max(-_VZ_CLAMP, min(_VZ_CLAMP, self._vz))

        self._dz += self._vz * dt

        # -- Append to buffers --
        self._vz_buf.append(self._vz)
        self._t_buf.append(t)
        self._az_buf.append(az)
        self._disp_buf.append(self._dz)

        # Smoothed displacement for peak/valley detection
        w = self.reversal_smooth_window
        if len(self._disp_buf) >= w:
            smooth_dz = sum(self._disp_buf[-w:]) / w
        else:
            smooth_dz = self._dz
        self._smooth_dz_buf.append(smooth_dz)

        # -- Detect displacement reversals with hysteresis --
        n = len(self._smooth_dz_buf)
        look_back = w // 2 + 1
        if n < look_back + 2:
            return None

        # Examine the point 'look_back' samples ago (causal delay)
        idx = n - 1 - look_back
        if idx < 1:
            return None

        # Use 5-point neighbourhood for robust peak/valley detection
        half_nb = min(3, idx, n - 1 - idx)
        if half_nb < 1:
            return None

        centre = self._smooth_dz_buf[idx]
        left_max = max(self._smooth_dz_buf[idx - half_nb:idx])
        left_min = min(self._smooth_dz_buf[idx - half_nb:idx])
        right_max = max(self._smooth_dz_buf[idx + 1:idx + 1 + half_nb])
        right_min = min(self._smooth_dz_buf[idx + 1:idx + 1 + half_nb])

        is_peak = centre >= left_max and centre >= right_max
        is_valley = centre <= left_min and centre <= right_min

        if not (is_peak or is_valley):
            return None

        reversal_type = 1 if is_peak else -1

        # Hysteresis: ignore reversal if too close in displacement to the last
        if self._reversals:
            last_rev = self._reversals[-1]
            # Must be opposite type
            if reversal_type == last_rev[1]:
                # Same type: update if this one is more extreme
                if (reversal_type == 1 and centre > last_rev[2]) or \
                   (reversal_type == -1 and centre < last_rev[2]):
                    self._reversals[-1] = (idx, reversal_type, centre)
                return None
            # Must exceed hysteresis threshold
            if abs(centre - last_rev[2]) < self._reversal_hysteresis:
                return None

        self._reversals.append((idx, reversal_type, centre))

        # -- Check for a full rep (3 reversals = 2 phases = 1 rep) --
        if len(self._reversals) >= 3:
            r0 = self._reversals[-3]  # first reversal
            r1 = self._reversals[-2]  # middle reversal (turnaround)
            r2 = self._reversals[-1]  # end reversal

            # r0 and r2 should be same type, r1 opposite
            if r0[1] == r2[1] and r0[1] != r1[1]:
                rep_metrics = self._process_full_rep(r0, r1, r2)
                # CONSUME used reversals -- keep only r2 as seed for
                # the next rep.  Without this, the next reversal r3
                # would form (r1,r2,r3) and double-count.
                self._reversals = [r2]

        # -- Memory management: trim old buffers --
        max_buf = 5000
        if len(self._disp_buf) > max_buf:
            trim = max_buf // 2
            self._disp_buf = self._disp_buf[trim:]
            self._vz_buf = self._vz_buf[trim:]
            self._t_buf = self._t_buf[trim:]
            self._az_buf = self._az_buf[trim:]
            self._smooth_dz_buf = self._smooth_dz_buf[trim:]
            # Adjust reversal indices
            self._reversals = [
                (max(0, ri - trim), rt, rv)
                for ri, rt, rv in self._reversals
                if ri >= trim
            ]

        return rep_metrics

    def _process_phase(self, start_idx: int, end_idx: int
                       ) -> tuple[float, float, float, float]:
        """
        Compute velocity metrics for a single phase (concentric or eccentric).

        Returns (mean_v, peak_v, rom, duration).
        Uses re-integrated accel with linear drift correction for accuracy.
        """
        seg_t = self._t_buf[start_idx:end_idx + 1]
        seg_az = self._az_buf[start_idx:end_idx + 1]
        seg_dz = self._disp_buf[start_idx:end_idx + 1]

        if len(seg_t) < 3:
            return 0.0, 0.0, 0.0, 0.0

        t_arr = np.array(seg_t)
        az_arr = np.array(seg_az)
        n = len(t_arr)
        dt_arr = np.diff(t_arr)
        t_span = t_arr[-1] - t_arr[0]
        if t_span <= 0:
            return 0.0, 0.0, 0.0, 0.0

        # Re-integrate accel -> velocity within this phase
        vz = np.zeros(n)
        for i in range(1, n):
            vz[i] = vz[i - 1] + 0.5 * (az_arr[i-1] + az_arr[i]) * dt_arr[i-1]

        # Linear drift correction (endpoint constraint)
        drift_rate = vz[-1] / t_span
        t_rel = t_arr - t_arr[0]
        vz -= drift_rate * t_rel

        # Displacement from re-integrated velocity
        dz = np.zeros(n)
        for i in range(1, n):
            dz[i] = dz[i-1] + 0.5 * (vz[i-1] + vz[i]) * dt_arr[i-1]

        speeds = np.abs(vz)
        mean_v = float(np.mean(speeds))
        peak_v = float(np.max(speeds))
        rom = abs(dz[-1] - dz[0])
        if rom < 0.005:  # fallback to raw displacement
            rom = abs(seg_dz[-1] - seg_dz[0])

        return mean_v, peak_v, rom, t_span

    def _process_full_rep(
            self,
            r0: tuple[int, int, float],
            r1: tuple[int, int, float],
            r2: tuple[int, int, float]
    ) -> Optional[RepMetrics]:
        """
        Process a full rep cycle from 3 reversal points.

        r0, r1, r2 = (buffer_index, type, smooth_dz)
        type: +1 = peak (top), -1 = valley (bottom)

        Two phases:
          Phase A: r0 -> r1
          Phase B: r1 -> r2

        One of these is concentric (valley->peak), the other eccentric
        (peak->valley).  We compute metrics for each phase and combine
        them into one RepMetrics.
        """
        idx0, type0, _ = r0
        idx1, type1, _ = r1
        idx2, type2, _ = r2

        if idx1 <= idx0 + 3 or idx2 <= idx1 + 3:
            return None

        # Compute metrics for each phase
        mA, pA, romA, durA = self._process_phase(idx0, idx1)
        mB, pB, romB, durB = self._process_phase(idx1, idx2)

        total_rom = max(romA, romB)
        total_dur = durA + durB

        # Validate the whole rep
        if total_rom < self.min_rom:
            return None
        if total_dur < self.min_rep_duration:
            return None
        if max(pA, pB) < self.min_peak_velocity:
            return None

        # Determine which phase is concentric, which is eccentric
        # valley(-1) -> peak(+1) = concentric (upward)
        # peak(+1) -> valley(-1) = eccentric (downward)
        if type0 == -1 and type1 == 1:
            # Phase A = concentric, Phase B = eccentric
            mcv, pcv, con_rom, con_dur = mA, pA, romA, durA
            mev, pev, ecc_rom, ecc_dur = mB, pB, romB, durB
        elif type0 == 1 and type1 == -1:
            # Phase A = eccentric, Phase B = concentric
            mev, pev, ecc_rom, ecc_dur = mA, pA, romA, durA
            mcv, pcv, con_rom, con_dur = mB, pB, romB, durB
        else:
            return None  # shouldn't happen

        self._rep_count += 1

        overall_mean = (mA * durA + mB * durB) / total_dur if total_dur > 0 else 0.0
        overall_peak = max(pA, pB)

        return RepMetrics(
            rep_number=self._rep_count,
            mean_concentric_velocity=mcv,
            peak_concentric_velocity=pcv,
            mean_eccentric_velocity=mev,
            peak_eccentric_velocity=pev,
            concentric_rom=con_rom,
            eccentric_rom=ecc_rom,
            total_rom=total_rom,
            concentric_duration=con_dur,
            eccentric_duration=ecc_dur,
            rep_duration=total_dur,
            mean_velocity=overall_mean,
            peak_velocity=overall_peak,
        )

    # -- Orientation-only EKF predict ------------------------------------------

    def _predict_orientation(self, gyro: np.ndarray, dt: float) -> None:
        """Propagate orientation state and covariance."""
        q = self.q

        Om = _omega_matrix(gyro)
        q_new = q + 0.5 * Om @ q * dt
        self.q = _qnorm(q_new)

        F = np.eye(_NX)
        F[0:4, 0:4] = np.eye(4) + 0.5 * Om * dt

        Xi = 0.5 * dt * np.array([
            [-q[1], -q[2], -q[3]],
            [ q[0], -q[3],  q[2]],
            [ q[3],  q[0], -q[1]],
            [-q[2],  q[1],  q[0]],
        ]) * (-DEG2RAD)
        F[_SQ, _SBG] = Xi

        Q = np.zeros((_NX, _NX))
        Q[0:4, 0:4] = np.eye(4) * self.Q_gyro * dt**2
        Q[4:7, 4:7] = np.eye(3) * self.Q_gbias * dt

        self.P = F @ self.P @ F.T + Q

    # -- Orientation-only EKF correct: accelerometer ---------------------------

    def _correct_accel(self, accel_g: np.ndarray) -> None:
        """Use accelerometer as gravity reference to correct orientation."""
        R = _q2dcm(self.q)
        g_pred = R.T @ np.array([0.0, 0.0, -1.0])

        amag = np.linalg.norm(accel_g)
        if amag < 0.5 or amag > 1.8:
            return
        g_meas = accel_g / amag

        z = g_meas - g_pred

        H = np.zeros((3, _NX))
        eps = 1e-5
        for i in range(4):
            qp = self.q.copy(); qm = self.q.copy()
            qp[i] += eps; qm[i] -= eps
            hp = _q2dcm(_qnorm(qp)).T @ np.array([0.0, 0.0, -1.0])
            hm = _q2dcm(_qnorm(qm)).T @ np.array([0.0, 0.0, -1.0])
            H[:, i] = (hp - hm) / (2 * eps)

        a_dev = abs(amag - 1.0)
        r_scale = 1.0 + (a_dev / 0.05) ** 2
        r_scale = min(r_scale, 100.0)
        R_meas = np.eye(3) * self.R_accel_base * r_scale

        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ z
        self.q  = _qnorm(self.q + dx[_SQ])
        self.bg += dx[_SBG] * RAD2DEG

        I_KH = np.eye(_NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T

    # -- Utilities -------------------------------------------------------------

    @staticmethod
    def _q2euler(q: np.ndarray) -> np.ndarray:
        """Quaternion -> Euler angles [roll, pitch, yaw] in degrees."""
        w, x, y, z = q
        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (w*y - z*x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = math.asin(sinp)
        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(siny, cosy)
        return np.array([roll, pitch, yaw]) * RAD2DEG
