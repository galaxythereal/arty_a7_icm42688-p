#!/usr/bin/env python3
"""
test_estimator.py — Offline tests for the EKF estimator module.

Run:  python3 -m pytest host/tests/test_estimator.py -v
"""

import math
import numpy as np
import pytest

# Allow running from repo root
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from host.imu_driver import IMUSample
from host.calibration import CalibrationResult, calibrate
from host.estimator import (
    IMUEstimator, _qnorm, _qmul, _qrot, _q2dcm, _omega_matrix,
    G_MPS2, DEG2RAD, RAD2DEG, ZUPTDetector,
)


# ── Fixtures ────────────────────────────────────────────────────────────────

def _ideal_cal() -> CalibrationResult:
    """Perfect calibration: no bias, gravity along -Z sensor axis."""
    return CalibrationResult(
        gyro_bias=np.zeros(3),
        accel_bias=np.zeros(3),
        gravity_mag=1.0,
        gravity_dir=np.array([0.0, 0.0, -1.0]),
        n_samples=500,
        std_gyro=np.zeros(3),
        std_accel=np.zeros(3),
    )


def _make_sample(t: float, ax=0.0, ay=0.0, az=-1.0,
                 gx=0.0, gy=0.0, gz=0.0, seq=0) -> IMUSample:
    return IMUSample(
        t=t, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz,
        temp=25.0, seq=seq,
    )


# ── Quaternion unit tests ──────────────────────────────────────────────────

class TestQuaternionOps:
    def test_identity_rotation(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        v = np.array([1.0, 2.0, 3.0])
        vr = _qrot(q, v)
        np.testing.assert_allclose(vr, v, atol=1e-12)

    def test_90deg_z_rotation(self):
        angle = math.pi / 2
        q = np.array([math.cos(angle/2), 0, 0, math.sin(angle/2)])
        v = np.array([1.0, 0.0, 0.0])
        vr = _qrot(q, v)
        np.testing.assert_allclose(vr, [0, 1, 0], atol=1e-12)

    def test_qnorm(self):
        q = np.array([3.0, 4.0, 0.0, 0.0])
        qn = _qnorm(q)
        assert abs(np.linalg.norm(qn) - 1.0) < 1e-12

    def test_qmul_identity(self):
        qi = np.array([1.0, 0.0, 0.0, 0.0])
        q = np.array([0.5, 0.5, 0.5, 0.5])
        np.testing.assert_allclose(_qmul(qi, q), q, atol=1e-12)

    def test_dcm_identity(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        R = _q2dcm(q)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_dcm_rotation(self):
        angle = math.pi / 2
        q = np.array([math.cos(angle/2), 0, 0, math.sin(angle/2)])
        R = _q2dcm(q)
        v = R @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(v, [0, 1, 0], atol=1e-12)


# ── ZUPT detector ──────────────────────────────────────────────────────────

class TestZUPT:
    def test_stationary_at_rest(self):
        z = ZUPTDetector(window=5, accel_thresh=0.1, gyro_thresh=5.0)
        for _ in range(10):
            assert z.update(1.0, 0.1) is True

    def test_moving_detected(self):
        z = ZUPTDetector(window=5, accel_thresh=0.1, gyro_thresh=5.0)
        # Fill window
        for _ in range(5):
            z.update(1.0, 0.1)
        # Now add high accel
        for _ in range(5):
            result = z.update(1.5, 0.1)
        assert result is False

    def test_high_gyro_not_stationary(self):
        z = ZUPTDetector(window=5, accel_thresh=0.1, gyro_thresh=5.0)
        for _ in range(5):
            z.update(1.0, 0.1)
        for _ in range(5):
            result = z.update(1.0, 10.0)
        assert result is False


# ── Calibration ─────────────────────────────────────────────────────────────

class TestCalibration:
    def test_ideal_calibration(self):
        samples = [_make_sample(i * 0.002, az=-1.0) for i in range(200)]
        cal = calibrate(samples)
        np.testing.assert_allclose(cal.gyro_bias, [0, 0, 0], atol=1e-10)
        assert abs(cal.gravity_mag - 1.0) < 0.01

    def test_biased_gyro(self):
        samples = [_make_sample(i * 0.002, gx=0.5, gy=-0.3, gz=0.1, az=-1.0)
                    for i in range(200)]
        cal = calibrate(samples)
        np.testing.assert_allclose(cal.gyro_bias, [0.5, -0.3, 0.1], atol=1e-10)

    def test_too_few_samples(self):
        samples = [_make_sample(i * 0.002) for i in range(10)]
        with pytest.raises(ValueError):
            calibrate(samples)

    def test_tilted_gravity(self):
        # Gravity along -Y  (board tilted 90°)
        samples = [_make_sample(i * 0.002, ax=0.0, ay=-1.0, az=0.0)
                    for i in range(200)]
        cal = calibrate(samples)
        assert abs(cal.gravity_mag - 1.0) < 0.01
        np.testing.assert_allclose(cal.gravity_dir, [0, -1, 0], atol=0.01)


# ── EKF integration tests ──────────────────────────────────────────────────

class TestEKF:
    def test_stationary_velocity_zero(self):
        """At rest, velocity should stay near zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(500):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.speed < 0.02  # m/s

    def test_stationary_position_zero(self):
        """At rest, position should stay near zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(500):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        disp = float(np.linalg.norm(state.position))
        assert disp < 0.01  # m

    def test_orientation_gravity_aligned(self):
        """With gravity along -Z, pitch and roll should be ~0."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(200):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert abs(state.euler[0]) < 2.0  # roll < 2°
        assert abs(state.euler[1]) < 2.0  # pitch < 2°

    def test_zupt_resets_velocity(self):
        """After a burst of motion, ZUPT should pull velocity back to zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)

        # Rest phase
        for i in range(100):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            ekf.update(s)

        # Motion phase (simulate +2g upward for 50 samples)
        for i in range(100, 150):
            s = _make_sample(i * 0.002, az=-3.0, seq=i)  # -1g gravity + -2g motion
            state = ekf.update(s)
        assert state.speed > 0.1  # should have non-zero velocity

        # Back to rest — ZUPT should kick in
        for i in range(150, 300):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.speed < 0.05  # velocity should be near zero

    def test_tilted_orientation(self):
        """Gravity along -Y should produce ~90° roll or pitch."""
        cal = CalibrationResult(
            gyro_bias=np.zeros(3),
            accel_bias=np.zeros(3),
            gravity_mag=1.0,
            gravity_dir=np.array([0.0, -1.0, 0.0]),
            n_samples=500,
            std_gyro=np.zeros(3),
            std_accel=np.zeros(3),
        )
        ekf = IMUEstimator(cal)
        for i in range(300):
            s = _make_sample(i * 0.002, ax=0.0, ay=-1.0, az=0.0, seq=i)
            state = ekf.update(s)
        # One of roll/pitch should be near ±90°
        angles = np.abs(state.euler[:2])
        assert max(angles) > 80.0

    def test_output_fields(self):
        """EstimatorState should have all expected fields."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        s = _make_sample(0.0, az=-1.0)
        state = ekf.update(s)
        assert hasattr(state, "q")
        assert hasattr(state, "euler")
        assert hasattr(state, "velocity")
        assert hasattr(state, "position")
        assert hasattr(state, "speed")
        assert hasattr(state, "is_stationary")
        assert hasattr(state, "accel_world")
        assert hasattr(state, "gyro_bias")
        assert state.q.shape == (4,)
        assert state.euler.shape == (3,)
        assert state.velocity.shape == (3,)
        assert state.position.shape == (3,)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
