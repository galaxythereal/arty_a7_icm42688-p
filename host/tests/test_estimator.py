#!/usr/bin/env python3
"""
test_estimator.py -- Tests for the per-rep dead-reckoning VBT estimator.

Tests cover:
  * Quaternion math helpers
  * ZUPT (SHOE) detector
  * Calibration module
  * 7-state orientation-only EKF (predict + correct)
  * Butterworth low-pass filter
  * Per-rep dead-reckoning with drift correction
  * Phase segmentation and RepMetrics computation
  * Long-duration drift bounds

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
    IMUEstimator, EstimatorState, _qnorm, _qmul, _qrot, _q2dcm,
    _omega_matrix, _skew,
    G_MPS2, DEG2RAD, RAD2DEG, ZUPTDetector, Butter2LP, RepMetrics, _NX,
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

    def test_qrot_180deg(self):
        """180-degree rotation about X axis."""
        q = np.array([0.0, 1.0, 0.0, 0.0])
        v = np.array([0.0, 1.0, 0.0])
        vr = _qrot(q, v)
        np.testing.assert_allclose(vr, [0.0, -1.0, 0.0], atol=1e-12)

    def test_qmul_inverse_gives_identity(self):
        """q * q_conj = identity."""
        q = _qnorm(np.array([1.0, 2.0, 3.0, 4.0]))
        qc = np.array([q[0], -q[1], -q[2], -q[3]])
        result = _qmul(q, qc)
        np.testing.assert_allclose(result, [1, 0, 0, 0], atol=1e-12)


# ── ZUPT detector ──────────────────────────────────────────────────────────

class TestZUPT:
    def test_stationary_at_rest(self):
        z = ZUPTDetector(window=5, accel_thresh=0.1, gyro_thresh=5.0)
        for _ in range(10):
            assert z.update(1.0, 0.1) is True

    def test_moving_detected(self):
        """High accel deviation should trigger non-stationary."""
        z = ZUPTDetector(window=5, accel_thresh=0.1, gyro_thresh=5.0)
        for _ in range(5):
            z.update(1.5, 0.1)
        result = z.update(1.5, 0.1)
        assert result is False

    def test_high_gyro_not_stationary(self):
        z = ZUPTDetector(window=5, accel_thresh=0.1, gyro_thresh=5.0)
        for _ in range(5):
            z.update(1.0, 0.1)
        for _ in range(5):
            result = z.update(1.0, 10.0)
        assert result is False

    def test_shoe_single_outlier_robust(self):
        """SHOE should be robust to a single outlier."""
        z = ZUPTDetector(window=10, accel_thresh=0.15, gyro_thresh=5.0)
        for _ in range(9):
            z.update(1.0, 0.1)
        result = z.update(1.3, 0.1)
        assert result is True


# ── Butterworth filter ─────────────────────────────────────────────────────

class TestButter2LP:
    def test_dc_passthrough(self):
        """DC signal should pass through unattenuated."""
        lp = Butter2LP(fc=20.0, fs=446.0, n_ch=3)
        signal = np.array([1.0, 2.0, 3.0])
        # Settle the filter
        for _ in range(500):
            y = lp(signal)
        np.testing.assert_allclose(y, signal, atol=0.01)

    def test_high_freq_attenuation(self):
        """High-frequency signal should be attenuated."""
        lp = Butter2LP(fc=20.0, fs=446.0, n_ch=1)
        # Generate 100 Hz sine wave (well above cutoff)
        dt = 1.0 / 446.0
        peak_out = 0.0
        for i in range(1000):
            x = np.array([math.sin(2 * math.pi * 100.0 * i * dt)])
            y = lp(x)
            if i > 200:  # skip transient
                peak_out = max(peak_out, abs(y[0]))
        # 100 Hz should be attenuated to < 10% of input amplitude
        assert peak_out < 0.10

    def test_low_freq_passthrough(self):
        """Low-frequency signal should pass with minimal attenuation."""
        lp = Butter2LP(fc=20.0, fs=446.0, n_ch=1)
        dt = 1.0 / 446.0
        peak_out = 0.0
        for i in range(2000):
            x = np.array([math.sin(2 * math.pi * 5.0 * i * dt)])
            y = lp(x)
            if i > 500:  # skip transient
                peak_out = max(peak_out, abs(y[0]))
        # 5 Hz should pass with > 90% amplitude
        assert peak_out > 0.90


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
        samples = [_make_sample(i * 0.002, ax=0.0, ay=-1.0, az=0.0)
                    for i in range(200)]
        cal = calibrate(samples)
        assert abs(cal.gravity_mag - 1.0) < 0.01
        np.testing.assert_allclose(cal.gravity_dir, [0, -1, 0], atol=0.01)


# ── EKF integration tests ──────────────────────────────────────────────────

class TestEKF:
    def test_covariance_7x7(self):
        """EKF should use 7x7 covariance for orientation-only state."""
        assert _NX == 7
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        assert ekf.P.shape == (7, 7)
        s = _make_sample(0.0, az=-1.0)
        ekf.update(s)
        assert ekf.P.shape == (7, 7)

    def test_stationary_velocity_zero(self):
        """At rest, velocity should stay near zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(500):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.speed < 0.02

    def test_stationary_position_zero(self):
        """At rest, position should stay near zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(500):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        disp = float(np.linalg.norm(state.position))
        assert disp < 0.01

    def test_orientation_gravity_aligned(self):
        """With gravity along -Z, pitch and roll should be ~0."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(200):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert abs(state.euler[0]) < 2.0
        assert abs(state.euler[1]) < 2.0

    def test_zupt_zeros_velocity(self):
        """After motion followed by rest, velocity should be hard-zeroed."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)

        # Rest phase
        for i in range(100):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            ekf.update(s)

        # Motion phase (simulate +2g upward for 50 samples)
        for i in range(100, 150):
            s = _make_sample(i * 0.002, az=-3.0, seq=i)
            state = ekf.update(s)

        # Back to rest -- velocity should be zeroed by dead-reckoning
        for i in range(150, 300):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.speed < 0.01  # hard-zeroed between reps

    def test_tilted_orientation(self):
        """Gravity along -Y should produce ~90 deg roll or pitch."""
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
        angles = np.abs(state.euler[:2])
        assert max(angles) > 80.0

    def test_output_fields(self):
        """EstimatorState should have all expected VBT fields."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        s = _make_sample(0.0, az=-1.0)
        state = ekf.update(s)

        # Core fields
        assert hasattr(state, "q")
        assert hasattr(state, "euler")
        assert hasattr(state, "velocity")
        assert hasattr(state, "position")
        assert hasattr(state, "speed")
        assert hasattr(state, "is_stationary")
        assert hasattr(state, "accel_world")
        assert hasattr(state, "gyro_bias")
        assert hasattr(state, "accel_bias")

        # New VBT fields
        assert hasattr(state, "vertical_velocity")
        assert hasattr(state, "vertical_displacement")
        assert hasattr(state, "is_moving")
        assert hasattr(state, "rep_metrics")

        # Shapes
        assert state.q.shape == (4,)
        assert state.euler.shape == (3,)
        assert state.velocity.shape == (3,)
        assert state.position.shape == (3,)
        assert state.accel_bias.shape == (3,)

    def test_long_stationary_drift_bounded(self):
        """5000 samples at rest -- velocity drift should stay < 5 mm/s."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(5000):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.speed < 0.005

    def test_rest_state_is_stationary(self):
        """At rest, is_stationary should be True and is_moving False."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(200):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.is_stationary is True
        assert state.is_moving is False

    def test_vertical_velocity_zero_at_rest(self):
        """At rest, vertical_velocity should be hard-zeroed."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(500):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert abs(state.vertical_velocity) < 1e-9

    def test_no_rep_at_rest(self):
        """At rest, no RepMetrics should be emitted."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(1000):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.rep_metrics is None

    def test_gyro_bias_stored_from_cal(self):
        """Gyro bias should be initialised from calibration."""
        cal = _ideal_cal()
        cal.gyro_bias = np.array([0.5, -0.3, 0.1])
        ekf = IMUEstimator(cal)
        np.testing.assert_allclose(ekf.bg, [0.5, -0.3, 0.1])


# ── Per-rep dead-reckoning tests ────────────────────────────────────────────

class TestDeadReckoning:
    def _simulate_rep(self, ekf: IMUEstimator, n_rest_before=200,
                      n_motion=120, n_rest_after=200,
                      motion_az=-3.0) -> list[EstimatorState]:
        """Simulate rest -> motion -> rest and return all states."""
        states = []
        i = 0

        # Rest before
        for _ in range(n_rest_before):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            states.append(ekf.update(s))
            i += 1

        # Motion: half concentric (accel up), half eccentric (accel down)
        half = n_motion // 2
        for _ in range(half):
            s = _make_sample(i * 0.002, az=motion_az, seq=i)
            states.append(ekf.update(s))
            i += 1
        for _ in range(n_motion - half):
            # Decelerate / eccentric: less gravity removal -> net down
            s = _make_sample(i * 0.002, az=-1.0 + (1.0 - motion_az), seq=i)
            states.append(ekf.update(s))
            i += 1

        # Rest after
        for _ in range(n_rest_after):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            states.append(ekf.update(s))
            i += 1

        return states

    def test_rep_detected_after_motion(self):
        """A motion window followed by rest should produce a RepMetrics."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states = self._simulate_rep(ekf)
        # Find any state with rep_metrics
        reps = [s.rep_metrics for s in states if s.rep_metrics is not None]
        assert len(reps) == 1
        assert reps[0].rep_number == 1

    def test_rep_metrics_fields(self):
        """RepMetrics should have all expected fields."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states = self._simulate_rep(ekf)
        reps = [s.rep_metrics for s in states if s.rep_metrics is not None]
        assert len(reps) >= 1
        rm = reps[0]

        assert rm.rep_number >= 1
        assert rm.mean_concentric_velocity >= 0
        assert rm.peak_concentric_velocity >= 0
        assert rm.mean_eccentric_velocity >= 0
        assert rm.peak_eccentric_velocity >= 0
        assert rm.total_rom >= 0
        assert rm.rep_duration > 0
        assert rm.mean_velocity >= 0
        assert rm.peak_velocity >= 0

    def test_two_reps_counted(self):
        """Two motion windows should produce two reps."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states1 = self._simulate_rep(ekf)
        states2 = self._simulate_rep(ekf)
        all_states = states1 + states2

        reps = [s.rep_metrics for s in all_states if s.rep_metrics is not None]
        assert len(reps) == 2
        assert reps[0].rep_number == 1
        assert reps[1].rep_number == 2

    def test_short_motion_not_a_rep(self):
        """A movement shorter than min_movement_samples should NOT be a rep."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal, min_movement_samples=40)

        states = []
        i = 0

        # Rest
        for _ in range(200):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            states.append(ekf.update(s))
            i += 1

        # Very short motion (10 samples < 40 threshold)
        for _ in range(10):
            s = _make_sample(i * 0.002, az=-3.0, seq=i)
            states.append(ekf.update(s))
            i += 1

        # Rest
        for _ in range(200):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            states.append(ekf.update(s))
            i += 1

        reps = [s.rep_metrics for s in states if s.rep_metrics is not None]
        assert len(reps) == 0

    def test_velocity_zeroed_between_reps(self):
        """Between reps (at rest), velocity should be exactly zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states = self._simulate_rep(ekf)

        # Get last rest state
        rest_states = [s for s in states[-50:] if s.is_stationary]
        assert len(rest_states) > 0
        for s in rest_states:
            assert abs(s.vertical_velocity) < 1e-9
            assert abs(s.vertical_displacement) < 1e-9

    def test_pcv_greater_or_equal_mcv(self):
        """Peak concentric velocity should be >= mean concentric velocity."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states = self._simulate_rep(ekf)
        reps = [s.rep_metrics for s in states if s.rep_metrics is not None]
        if len(reps) > 0:
            rm = reps[0]
            assert rm.peak_concentric_velocity >= rm.mean_concentric_velocity


# ── Omega and skew matrices ────────────────────────────────────────────────

class TestMatrixOps:
    def test_omega_antisymmetric(self):
        w = np.array([1.0, 2.0, 3.0])
        Om = _omega_matrix(w)
        np.testing.assert_allclose(Om, -Om.T, atol=1e-12)

    def test_skew_antisymmetric(self):
        v = np.array([1.0, 2.0, 3.0])
        S = _skew(v)
        np.testing.assert_allclose(S, -S.T, atol=1e-12)

    def test_skew_cross_product(self):
        """Skew matrix should implement cross product: [v]x * u = v x u."""
        v = np.array([1.0, 2.0, 3.0])
        u = np.array([4.0, 5.0, 6.0])
        S = _skew(v)
        np.testing.assert_allclose(S @ u, np.cross(v, u), atol=1e-12)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
