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

  * **Per-rep dead-reckoning** -- velocity is obtained by trapezoidal
    integration of world-frame linear acceleration *only during detected
    movement windows*.  Between reps, velocity is hard-zeroed.

  * **Phase segmentation** -- each rep is split into concentric (up) and
    eccentric (down) phases via vertical velocity zero-crossing.  Mean
    Concentric Velocity (MCV) and Peak Concentric Velocity (PCV) are the
    gold-standard VBT metrics.

  * **Per-phase linear drift correction** -- endpoint constraint forces
    v=0 at rep boundaries, linearly distributing residual drift across
    the movement window (standard INS technique).

  * **Butterworth low-pass on accel** -- 20 Hz 2nd-order IIR removes
    sensor noise without lagging the velocity waveform excessively.

State vector  (7)
    q  = [q0, q1, q2, q3]   orientation quaternion  (scalar-first)
    bg = [bgx, bgy, bgz]    gyro bias  (deg/s)

Velocity & position are NOT part of the EKF -- they are computed per-rep
via dead-reckoning with endpoint drift correction.
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
                 movement_accel_thresh: float = 0.4,
                 min_movement_samples: int = 40):
        self.cal = cal
        self.dt_nominal = dt_nominal
        self.movement_accel_thresh = movement_accel_thresh
        self.min_movement_samples = min_movement_samples

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

        # -- Dead-reckoning state --
        self._in_movement = False
        self._movement_buf_az: list[float] = []
        self._movement_buf_t: list[float] = []
        self._movement_buf_aworld: list[np.ndarray] = []
        self._rest_counter = 0
        self._rest_threshold = 15

        # -- Current sample velocity/position (for display) --
        self.v = np.zeros(3)
        self.pos = np.zeros(3)
        self._current_vz = 0.0
        self._current_dz = 0.0

        # -- Rep tracking --
        self._rep_count = 0
        self._pending_rep: Optional[RepMetrics] = None

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

        # -- PER-REP DEAD-RECKONING --
        az = accel_world_filt[2]
        rep_metrics = self._dead_reckon(az, accel_world_filt, sample.t, dt)

        return EstimatorState(
            t=sample.t,
            q=self.q.copy(),
            euler=self._q2euler(self.q),
            velocity=self.v.copy(),
            position=self.pos.copy(),
            accel_world=accel_world_filt.copy(),
            speed=float(np.linalg.norm(self.v)),
            vertical_velocity=self._current_vz,
            vertical_displacement=self._current_dz,
            is_stationary=self.is_stationary,
            is_moving=self._in_movement,
            gyro_bias=self.bg.copy(),
            accel_bias=self.cal.accel_bias.copy(),
            rep_metrics=rep_metrics,
        )

    # -- Dead-reckoning with per-rep drift correction --------------------------

    def _dead_reckon(self, az: float, accel_world: np.ndarray,
                     t: float, dt: float) -> Optional[RepMetrics]:
        """
        Per-rep dead-reckoning.

        Accumulates world-frame vertical acceleration during movement,
        then at end of rep applies trapezoidal integration with
        linear drift correction (endpoint constraint: v_start = v_end = 0).
        """
        rep_metrics = None

        if self.is_stationary:
            if self._in_movement:
                self._rest_counter += 1
                self._movement_buf_az.append(az)
                self._movement_buf_t.append(t)
                self._movement_buf_aworld.append(accel_world.copy())

                if self._rest_counter >= self._rest_threshold:
                    trim = self._rest_threshold
                    buf_az = self._movement_buf_az[:-trim] if trim > 0 else self._movement_buf_az
                    buf_t  = self._movement_buf_t[:-trim] if trim > 0 else self._movement_buf_t
                    buf_aw = self._movement_buf_aworld[:-trim] if trim > 0 else self._movement_buf_aworld

                    if len(buf_az) >= self.min_movement_samples:
                        rep_metrics = self._process_rep(buf_az, buf_t, buf_aw)

                    self._in_movement = False
                    self._movement_buf_az.clear()
                    self._movement_buf_t.clear()
                    self._movement_buf_aworld.clear()
                    self._rest_counter = 0
                    self.v = np.zeros(3)
                    self.pos = np.zeros(3)
                    self._current_vz = 0.0
                    self._current_dz = 0.0
            else:
                self.v = np.zeros(3)
                self.pos = np.zeros(3)
                self._current_vz = 0.0
                self._current_dz = 0.0
                self._rest_counter = 0
        else:
            self._rest_counter = 0
            if not self._in_movement:
                self._in_movement = True
                self._movement_buf_az.clear()
                self._movement_buf_t.clear()
                self._movement_buf_aworld.clear()

            self._movement_buf_az.append(az)
            self._movement_buf_t.append(t)
            self._movement_buf_aworld.append(accel_world.copy())

            # Live (uncorrected) velocity for display
            self._current_vz += az * dt
            self._current_dz += self._current_vz * dt
            self.v[2] = self._current_vz
            self.pos[2] = self._current_dz

        return rep_metrics

    def _process_rep(self, buf_az: list[float], buf_t: list[float],
                     buf_aw: list[np.ndarray]) -> RepMetrics:
        """
        Process a completed movement window into VBT rep metrics.

        Uses trapezoidal integration with linear drift correction.
        """
        self._rep_count += 1
        n = len(buf_az)
        az_arr = np.array(buf_az)
        t_arr  = np.array(buf_t)

        # -- Compute dt array --
        dt_arr = np.diff(t_arr)
        if len(dt_arr) == 0:
            dt_arr = np.array([self.dt_nominal])

        # -- Trapezoidal integration: accel -> velocity --
        vz = np.zeros(n)
        for i in range(1, n):
            vz[i] = vz[i-1] + 0.5 * (az_arr[i-1] + az_arr[i]) * dt_arr[i-1]

        # -- Linear drift correction --
        # Endpoint constraint: v should be 0 at both ends
        t_span = t_arr[-1] - t_arr[0]
        drift_rate = vz[-1] / t_span if t_span > 0 else 0.0
        t_rel = t_arr - t_arr[0]
        vz_corrected = vz - drift_rate * t_rel

        # -- Trapezoidal integration: velocity -> displacement --
        dz = np.zeros(n)
        for i in range(1, n):
            dz[i] = dz[i-1] + 0.5 * (vz_corrected[i-1] + vz_corrected[i]) * dt_arr[i-1]

        # -- Phase segmentation --
        max_disp = dz.max()
        min_disp = dz.min()

        if abs(max_disp) >= abs(min_disp):
            con_mask = vz_corrected > 0
            ecc_mask = vz_corrected < 0
        else:
            con_mask = vz_corrected < 0
            ecc_mask = vz_corrected > 0

        # -- Concentric metrics --
        con_speeds = np.abs(vz_corrected[con_mask]) if con_mask.any() else np.array([0.0])
        mcv = float(np.mean(con_speeds)) if len(con_speeds) > 0 else 0.0
        pcv = float(np.max(con_speeds)) if len(con_speeds) > 0 else 0.0

        con_indices = np.where(con_mask)[0]
        if len(con_indices) > 1:
            con_rom = abs(dz[con_indices[-1]] - dz[con_indices[0]])
            con_dur = t_arr[con_indices[-1]] - t_arr[con_indices[0]]
        else:
            con_rom = 0.0
            con_dur = 0.0

        # -- Eccentric metrics --
        ecc_speeds = np.abs(vz_corrected[ecc_mask]) if ecc_mask.any() else np.array([0.0])
        mev = float(np.mean(ecc_speeds)) if len(ecc_speeds) > 0 else 0.0
        pev = float(np.max(ecc_speeds)) if len(ecc_speeds) > 0 else 0.0

        ecc_indices = np.where(ecc_mask)[0]
        if len(ecc_indices) > 1:
            ecc_rom = abs(dz[ecc_indices[-1]] - dz[ecc_indices[0]])
            ecc_dur = t_arr[ecc_indices[-1]] - t_arr[ecc_indices[0]]
        else:
            ecc_rom = 0.0
            ecc_dur = 0.0

        # -- Overall metrics --
        total_rom = abs(max_disp - min_disp)
        rep_dur = t_arr[-1] - t_arr[0] if n > 1 else 0.0
        mean_v = float(np.mean(np.abs(vz_corrected)))
        peak_v = float(np.max(np.abs(vz_corrected)))

        self._pending_rep = RepMetrics(
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
            rep_duration=rep_dur,
            mean_velocity=mean_v,
            peak_velocity=peak_v,
        )
        return self._pending_rep

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
