#!/usr/bin/env python3
"""
estimator.py — Tactical-grade Extended Kalman Filter for VBT.

Drift-mitigation techniques (informed by 2024–2025 research):
  • **SHOE detector** — Stance Hypothesis Optimal dEtection (Skog et al.)
    uses variance-based test statistic instead of fragile max-threshold.
  • **13-state EKF** — tracks accelerometer bias online so gravity
    subtraction stays accurate over time.
  • **Tight ZUPT** — R_zupt = 0.001² for hard velocity reset at rest.
  • **Adaptive accel R** — measurement noise scales with |a|-deviation
    from 1 g, smoothly de-weighting during high dynamics.
  • **Exponential velocity decay** — gentle high-pass between ZUPTs
    attenuates low-frequency integration drift.

State vector  (13)
    q  = [q0, q1, q2, q3]   orientation quaternion  (scalar-first)
    bg = [bgx, bgy, bgz]    gyro bias  (°/s)
    ba = [bax, bay, baz]    accel bias (g)
    v  = [vx, vy, vz]       velocity in world frame  (m/s)

Position is tracked outside the EKF (simple integration of v) because
ZUPT already controls velocity drift, and keeping the state small keeps
the filter fast enough for real-time on PC.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .calibration import CalibrationResult
from .imu_driver import IMUSample

# ── Physical constants ──────────────────────────────────────────────────────
G_MPS2 = 9.80665          # m/s²
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# ── State indices ───────────────────────────────────────────────────────────
_SQ = slice(0, 4)     # quaternion
_SBG = slice(4, 7)    # gyro bias
_SBA = slice(7, 10)   # accel bias  (NEW)
_SV = slice(10, 13)   # velocity
_NX = 13              # total state dimension


# ── Quaternion helpers (scalar-first: q = [w, x, y, z]) ────────────────────

def _qnorm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product  a ⊗ b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector *v* by quaternion *q*:  v' = q ⊗ [0,v] ⊗ q*."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    return _qmul(_qmul(q, qv), qc)[1:]


def _qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _q2dcm(q: np.ndarray) -> np.ndarray:
    """Quaternion → 3×3 rotation matrix  (body→world)."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def _omega_matrix(w: np.ndarray) -> np.ndarray:
    """Ω(ω) for quaternion derivative:  q̇ = ½ Ω(ω) q."""
    wx, wy, wz = w
    return np.array([
        [ 0,  -wx, -wy, -wz],
        [ wx,  0,   wz, -wy],
        [ wy, -wz,  0,   wx],
        [ wz,  wy, -wx,  0 ],
    ])


def _skew(v: np.ndarray) -> np.ndarray:
    """3-vector → skew-symmetric matrix [v]×."""
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ])


# ── SHOE-based ZUPT detector ───────────────────────────────────────────────

class ZUPTDetector:
    """
    Zero-Velocity Update detector using the SHOE test statistic.

    SHOE (Stance Hypothesis Optimal dEtection) from Skog et al. (2010)
    uses a variance-based test over a sliding window instead of a brittle
    max-threshold.  A single outlier sample no longer vetoes the entire
    window.

    The test computes:
      • accel component — mean squared deviation of |a| from 1 g
      • gyro  component — mean squared gyro magnitude

    Both must be below their respective thresholds² for zero-velocity.

    Parameters
    ----------
    window : int
        Sliding window length (samples).
    accel_thresh : float
        Threshold on RMS accel deviation from 1 g (g).
    gyro_thresh : float
        Threshold on RMS gyro magnitude (°/s).
    """
    def __init__(self, window: int = 20,
                 accel_thresh: float = 0.06,
                 gyro_thresh: float = 2.5):
        self.window = window
        self.accel_thresh = accel_thresh   # g
        self.gyro_thresh  = gyro_thresh    # °/s
        self._accel_buf: list[float] = []
        self._gyro_buf:  list[float] = []

    def update(self, accel_mag: float, gyro_mag: float) -> bool:
        """
        Return True if sensor is currently stationary.

        Uses variance-based SHOE statistic over the sliding window.
        """
        self._accel_buf.append(accel_mag)
        self._gyro_buf.append(gyro_mag)
        if len(self._accel_buf) > self.window:
            self._accel_buf.pop(0)
            self._gyro_buf.pop(0)
        if len(self._accel_buf) < self.window:
            return True  # assume stationary during fill-up

        # SHOE test: accel — mean squared deviation of |a| from 1 g
        a_arr = np.array(self._accel_buf)
        a_var = float(np.mean((a_arr - 1.0) ** 2))

        # SHOE test: gyro — mean squared magnitude
        g_arr = np.array(self._gyro_buf)
        g_var = float(np.mean(g_arr ** 2))

        # Both must be below threshold²
        a_ok = a_var < self.accel_thresh ** 2
        g_ok = g_var < self.gyro_thresh ** 2
        return a_ok and g_ok


# ── EKF Estimator ──────────────────────────────────────────────────────────

@dataclass
class EstimatorState:
    """Readable snapshot of the estimator output."""
    t: float               # timestamp (s)
    q: np.ndarray          # orientation quaternion [w,x,y,z]
    euler: np.ndarray      # [roll, pitch, yaw]  (deg)
    velocity: np.ndarray   # [vx, vy, vz]  world frame  (m/s)
    position: np.ndarray   # [px, py, pz]  world frame  (m)
    accel_world: np.ndarray  # linear accel in world frame (m/s²), gravity removed
    speed: float           # |v|  (m/s)
    is_stationary: bool    # ZUPT active
    gyro_bias: np.ndarray  # estimated gyro bias (°/s)
    accel_bias: np.ndarray # estimated accel bias (g)


class IMUEstimator:
    """
    13-state Extended Kalman Filter for tactical-grade VBT.

    State: [q(4), gyro_bias(3), accel_bias(3), velocity(3)]

    Parameters
    ----------
    cal : CalibrationResult
        Bias offsets from static calibration.
    dt_nominal : float
        Expected sample period (s).  Default 1/446 ≈ 2.24 ms.
    sigma_gyro : float
        Gyro noise density  (°/s).
    sigma_accel : float
        Accelerometer noise density  (g).
    sigma_gyro_bias : float
        Gyro bias random-walk  (°/s per √s).
    sigma_accel_bias : float
        Accel bias random-walk  (g per √s).
    vel_decay : float
        Exponential velocity decay rate (1/s).  Attenuates low-freq drift
        between ZUPTs.  0 = disabled, 0.5–2.0 typical.
    """

    def __init__(self,
                 cal: CalibrationResult,
                 dt_nominal: float = 1.0 / 446.0,
                 sigma_gyro: float = 0.3,
                 sigma_accel: float = 0.03,
                 sigma_gyro_bias: float = 0.0005,
                 sigma_accel_bias: float = 0.00005,
                 vel_decay: float = 0.8):
        self.cal = cal
        self.dt_nominal = dt_nominal
        self.vel_decay = vel_decay

        # ── State: [q(4), bg(3), ba(3), v(3)] = 13 ──
        self._init_orientation_from_gravity(cal.gravity_dir)
        self.bg = cal.gyro_bias.copy()
        self.ba = cal.accel_bias.copy()
        self.v  = np.zeros(3)
        self.pos = np.zeros(3)

        # ── Covariance (13×13) ──
        self.P = np.diag([
            1e-4, 1e-4, 1e-4, 1e-4,       # quaternion
            1e-2, 1e-2, 1e-2,               # gyro bias  (°/s)²
            1e-4, 1e-4, 1e-4,               # accel bias (g)²
            1e-4, 1e-4, 1e-4,               # velocity   (m/s)²
        ])

        # ── Noise tuning ──
        self.Q_gyro  = (sigma_gyro * DEG2RAD) ** 2
        self.Q_accel = (sigma_accel * G_MPS2) ** 2
        self.Q_gbias = (sigma_gyro_bias * DEG2RAD) ** 2
        self.Q_abias = (sigma_accel_bias * G_MPS2) ** 2
        self.R_accel_base = (sigma_accel * G_MPS2) ** 2
        self.R_zupt  = 0.001 ** 2   # tight ZUPT: 1 mm/s noise (was 10 mm/s)

        # ── ZUPT ──
        self.zupt = ZUPTDetector()
        self.is_stationary = True

        # ── Bookkeeping ──
        self.t_prev: float | None = None

    # ── Initialisation helpers ──────────────────────────────────────────────

    def _init_orientation_from_gravity(self, g_dir: np.ndarray) -> None:
        """
        Set initial quaternion so that the world-frame Z axis aligns with
        the measured gravity direction in the sensor frame.

        This assumes the sensor is stationary and the only acceleration is
        gravity.  The yaw component is unobservable and set to zero.
        """
        # g_dir is the unit gravity vector in the sensor frame
        # We want a rotation R such that R @ g_dir = [0, 0, 1]  (world down=+Z)
        # or  R @ g_dir = [0, 0, -1]  depending on convention.
        # Convention: world Z = up,  gravity = [0, 0, -1] in world.
        # So  R^T @ [0, 0, -1] = g_dir  →  R @ g_dir = [0, 0, -1]

        gn = g_dir / (np.linalg.norm(g_dir) + 1e-12)

        # Desired: R @ gn = [0, 0, -1]
        # Rotation axis = gn × [0,0,-1],  angle = arccos(gn · [0,0,-1])
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
            self.q = np.array([math.cos(angle / 2), axis[0]*s, axis[1]*s, axis[2]*s])

        self.q = _qnorm(self.q)

    # ── Public interface ────────────────────────────────────────────────────

    def update(self, sample: IMUSample) -> EstimatorState:
        """
        Process one IMU sample through the EKF pipeline.

        Returns
        -------
        EstimatorState
            Current orientation, velocity, position, and diagnostics.
        """
        # ── Time step ──
        if self.t_prev is None:
            self.t_prev = sample.t
            dt = self.dt_nominal
        else:
            dt = sample.t - self.t_prev
            if dt <= 0 or dt > 0.1:
                dt = self.dt_nominal
            self.t_prev = sample.t

        # ── Sensor readings (corrected for bias) ──
        gyro_raw = np.array([sample.gx, sample.gy, sample.gz])   # °/s
        accel_raw = np.array([sample.ax, sample.ay, sample.az])   # g

        gyro = (gyro_raw - self.bg) * DEG2RAD     # rad/s, bias-corrected
        accel_g = accel_raw - self.ba              # g, bias-corrected (tracked online)

        # ── ZUPT detection (SHOE) ──
        # Use calibration-corrected accel for ZUPT, NOT EKF-tracked bias,
        # to avoid feedback loops where a drifted ba prevents ZUPT activation.
        accel_cal = accel_raw - self.cal.accel_bias
        accel_mag_zupt = np.linalg.norm(accel_cal)
        gyro_mag  = np.linalg.norm(gyro_raw - self.bg)
        self.is_stationary = self.zupt.update(accel_mag_zupt, gyro_mag)

        # ══════════════════════════════════════════════════════════════════
        # PREDICT
        # ══════════════════════════════════════════════════════════════════
        self._predict(gyro, accel_g, dt)

        # ══════════════════════════════════════════════════════════════════
        # CORRECT — accelerometer (gravity reference)
        # ══════════════════════════════════════════════════════════════════
        self._correct_accel(accel_g)

        # ══════════════════════════════════════════════════════════════════
        # CORRECT — ZUPT  (zero velocity when stationary)
        # ══════════════════════════════════════════════════════════════════
        if self.is_stationary:
            self._correct_zupt()
            self.pos = np.zeros(3)  # reset displacement each rest phase

        # ── Velocity decay (high-pass) — attenuate low-freq drift ──
        if not self.is_stationary and self.vel_decay > 0:
            decay = math.exp(-self.vel_decay * dt)
            self.v *= decay

        # ── Integrate position ──
        self.pos += self.v * dt

        # ── Build output ──
        R = _q2dcm(self.q)
        accel_world = R @ (accel_g * G_MPS2) - np.array([0.0, 0.0, -G_MPS2])

        return EstimatorState(
            t=sample.t,
            q=self.q.copy(),
            euler=self._q2euler(self.q),
            velocity=self.v.copy(),
            position=self.pos.copy(),
            accel_world=accel_world,
            speed=float(np.linalg.norm(self.v)),
            is_stationary=self.is_stationary,
            gyro_bias=self.bg.copy(),
            accel_bias=self.ba.copy(),
        )

    # ── EKF predict ─────────────────────────────────────────────────────────

    def _predict(self, gyro: np.ndarray, accel_g: np.ndarray,
                 dt: float) -> None:
        """Propagate state and covariance one time step."""
        q = self.q

        # -- Quaternion propagation --
        Om = _omega_matrix(gyro)
        q_new = q + 0.5 * Om @ q * dt
        self.q = _qnorm(q_new)

        # -- Velocity propagation --
        R = _q2dcm(self.q)
        accel_world = R @ (accel_g * G_MPS2)      # m/s² in world frame
        accel_world[2] += G_MPS2                   # remove gravity (Z = up)
        self.v += accel_world * dt

        # -- State transition Jacobian F (13×13) --
        F = np.eye(_NX)

        # ∂q/∂q
        F[0:4, 0:4] = np.eye(4) + 0.5 * Om * dt

        # ∂q/∂bg (quaternion sensitivity to gyro bias)
        Xi = 0.5 * dt * np.array([
            [-q[1], -q[2], -q[3]],
            [ q[0], -q[3],  q[2]],
            [ q[3],  q[0], -q[1]],
            [-q[2],  q[1],  q[0]],
        ]) * (-DEG2RAD)
        F[_SQ, _SBG] = Xi

        # ∂v/∂ba — velocity sensitivity to accel bias
        # v += R @ (accel_g * G) * dt, accel_g = a_raw - ba
        # ∂v/∂ba = -R * G * dt
        F[10:13, 7:10] = -R * G_MPS2 * dt

        # -- Process noise Q --
        Q = np.zeros((_NX, _NX))
        Q[0:4, 0:4]   = np.eye(4) * self.Q_gyro * dt**2
        Q[4:7, 4:7]   = np.eye(3) * self.Q_gbias * dt
        Q[7:10, 7:10] = np.eye(3) * self.Q_abias * dt   # accel bias random walk
        Q[10:13, 10:13] = np.eye(3) * self.Q_accel * dt**2

        self.P = F @ self.P @ F.T + Q

    # ── EKF correct: accelerometer ──────────────────────────────────────────

    def _correct_accel(self, accel_g: np.ndarray) -> None:
        """
        Use accelerometer as a gravity reference to correct orientation
        and accel bias.

        Measurement model:  h(q) = R(q)^T @ [0, 0, -1]  ≈  accel / |accel|
        """
        R = _q2dcm(self.q)
        g_pred = R.T @ np.array([0.0, 0.0, -1.0])

        amag = np.linalg.norm(accel_g)
        if amag < 0.5 or amag > 1.8:
            return  # reject outlier
        g_meas = accel_g / amag

        # Innovation
        z = g_meas - g_pred

        # ── Measurement Jacobian H (3×13) ──
        H = np.zeros((3, _NX))

        # ∂(R^T g_w)/∂q via numerical perturbation (4 cols)
        eps = 1e-5
        for i in range(4):
            qp = self.q.copy(); qm = self.q.copy()
            qp[i] += eps; qm[i] -= eps
            hp = _q2dcm(_qnorm(qp)).T @ np.array([0.0, 0.0, -1.0])
            hm = _q2dcm(_qnorm(qm)).T @ np.array([0.0, 0.0, -1.0])
            H[:, i] = (hp - hm) / (2 * eps)

        # ∂h/∂ba — accel bias affects the measurement through accel_g
        # g_meas = (a_raw - ba) / |a_raw - ba|
        # first-order: ∂g_meas/∂ba ≈ -(I - g_meas g_meas^T) / amag
        I3 = np.eye(3)
        H[:, 7:10] = -(I3 - np.outer(g_meas, g_meas)) / amag

        # ── Adaptive R: smooth quadratic scaling with |a|-deviation ──
        a_dev = abs(amag - 1.0)
        r_scale = 1.0 + (a_dev / 0.05) ** 2
        r_scale = min(r_scale, 100.0)
        R_meas = np.eye(3) * self.R_accel_base * r_scale

        # Kalman gain
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)

        # State update
        dx = K @ z
        self.q  = _qnorm(self.q + dx[_SQ])
        self.bg += dx[_SBG] * RAD2DEG       # convert back to °/s
        self.ba += dx[_SBA]                  # g
        self.v  += dx[_SV]

        # Covariance update (Joseph form)
        I_KH = np.eye(_NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T

    # ── EKF correct: ZUPT ──────────────────────────────────────────────────

    def _correct_zupt(self) -> None:
        """Apply zero-velocity update: v = 0 when stationary."""
        z = -self.v

        H = np.zeros((3, _NX))
        H[0, 10] = 1.0
        H[1, 11] = 1.0
        H[2, 12] = 1.0

        R_meas = np.eye(3) * self.R_zupt
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = K @ z
        self.q  = _qnorm(self.q + dx[_SQ])
        self.bg += dx[_SBG] * RAD2DEG
        self.ba += dx[_SBA]
        self.v  += dx[_SV]

        I_KH = np.eye(_NX) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_meas @ K.T

    # ── Utilities ───────────────────────────────────────────────────────────

    @staticmethod
    def _q2euler(q: np.ndarray) -> np.ndarray:
        """Quaternion → Euler angles [roll, pitch, yaw] in degrees."""
        w, x, y, z = q
        # Roll  (X)
        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(sinr, cosr)
        # Pitch (Y)
        sinp = 2.0 * (w*y - z*x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = math.asin(sinp)
        # Yaw   (Z)
        siny = 2.0 * (w*z + x*y)
        cosy = 1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(siny, cosy)
        return np.array([roll, pitch, yaw]) * RAD2DEG
