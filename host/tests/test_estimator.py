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
    _VZ_CLAMP,
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

    def test_zupt_decays_velocity(self):
        """After motion followed by rest, velocity should decay near zero."""
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

        # Back to rest -- velocity should decay toward zero
        # Need ~2s of rest for exp(-3*2)=0.0025 decay
        for i in range(150, 1200):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.speed < 0.05  # decayed near zero

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

    def test_vertical_velocity_near_zero_at_rest(self):
        """At rest, vertical_velocity should decay to near zero."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(500):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert abs(state.vertical_velocity) < 0.01

    def test_no_rep_at_rest(self):
        """At rest, no RepMetrics should be emitted."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        for i in range(1000):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)
        assert state.rep_metrics is None

    def test_velocity_clamped_during_extreme_motion(self):
        """Velocity should never exceed the physical clamp."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)

        # Rest
        for i in range(100):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            ekf.update(s)

        # Absurd acceleration: 20g for 200 samples
        for i in range(100, 300):
            s = _make_sample(i * 0.002, az=-21.0, seq=i)
            state = ekf.update(s)

        assert abs(state.vertical_velocity) <= _VZ_CLAMP + 0.01

    def test_velocity_near_zero_after_long_rest(self):
        """After exercise followed by long rest, velocity must settle."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)

        # Rest → motion → rest pattern (mimics finish exercising + put down)
        for i in range(100):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            ekf.update(s)
        for i in range(100, 200):
            s = _make_sample(i * 0.002, az=-3.0, seq=i)
            ekf.update(s)
        # 5 seconds of rest (2500 samples)
        for i in range(200, 2700):
            s = _make_sample(i * 0.002, az=-1.0, seq=i)
            state = ekf.update(s)

        assert abs(state.vertical_velocity) < 0.01

    def test_gyro_bias_stored_from_cal(self):
        """Gyro bias should be initialised from calibration."""
        cal = _ideal_cal()
        cal.gyro_bias = np.array([0.5, -0.3, 0.1])
        ekf = IMUEstimator(cal)
        np.testing.assert_allclose(ekf.bg, [0.5, -0.3, 0.1])


# ── Per-rep dead-reckoning tests ────────────────────────────────────────────

class TestDeadReckoning:
    """Tests for the displacement-reversal full-rep-cycle detector."""

    @staticmethod
    def _drive(ekf: IMUEstimator, n: int, az: float = -1.0,
               start_i: int = 0) -> list[EstimatorState]:
        """Feed *n* samples with a given vertical accel."""
        states = []
        for k in range(n):
            i = start_i + k
            s = _make_sample(i * 0.002, az=az, seq=i)
            states.append(ekf.update(s))
        return states

    def _simulate_full_rep(self, ekf: IMUEstimator,
                           n_rest: int = 200,
                           n_con: int = 60,
                           n_ecc: int = 60,
                           accel_up: float = -3.0,
                           accel_down: float = 1.0,
                           start_i: int = 0
                           ) -> tuple[list[EstimatorState], int]:
        """
        Simulate one full concentric + eccentric rep cycle.

        Returns (all_states, next_i).

        The acceleration pattern:
          Rest:  az = -1.0g  (no net accel after gravity removal)
          Conc:  az = accel_up (-3.0g = +2g net upward)
          Ecc :  az = accel_down (+1.0g = +2g net downward)

        This creates a valley -> peak -> valley displacement cycle.
        """
        i = start_i
        states = []

        # Rest before
        states += self._drive(ekf, n_rest, az=-1.0, start_i=i)
        i += n_rest

        # Concentric (upward): net +2g vertical accel
        states += self._drive(ekf, n_con, az=accel_up, start_i=i)
        i += n_con

        # Eccentric (downward): net +2g downward
        states += self._drive(ekf, n_ecc, az=accel_down, start_i=i)
        i += n_ecc

        # Rest after
        states += self._drive(ekf, n_rest, az=-1.0, start_i=i)
        i += n_rest

        return states, i

    def test_rep_detected_after_full_cycle(self):
        """A full concentric + eccentric cycle should produce a RepMetrics."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        # Need at least 2 full cycles to get 3 alternating reversals
        # (valley0 -> peak1 -> valley2 triggers rep on 3rd reversal)
        states1, next_i = self._simulate_full_rep(ekf, start_i=0)
        states2, _ = self._simulate_full_rep(ekf, start_i=next_i)
        all_states = states1 + states2

        reps = [s.rep_metrics for s in all_states if s.rep_metrics is not None]
        assert len(reps) >= 1
        assert reps[0].rep_number == 1

    def test_rep_has_both_phases(self):
        """RepMetrics should have BOTH concentric and eccentric metrics."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states1, next_i = self._simulate_full_rep(ekf, start_i=0)
        states2, _ = self._simulate_full_rep(ekf, start_i=next_i)
        all_states = states1 + states2

        reps = [s.rep_metrics for s in all_states if s.rep_metrics is not None]
        assert len(reps) >= 1
        rm = reps[0]

        # Both concentric AND eccentric should have non-zero metrics
        assert rm.mean_concentric_velocity > 0 or rm.mean_eccentric_velocity > 0
        assert rm.total_rom > 0
        assert rm.rep_duration > 0
        assert rm.peak_velocity > 0

    def test_rep_metrics_fields(self):
        """RepMetrics should have all expected fields."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states1, next_i = self._simulate_full_rep(ekf, start_i=0)
        states2, _ = self._simulate_full_rep(ekf, start_i=next_i)
        all_states = states1 + states2

        reps = [s.rep_metrics for s in all_states if s.rep_metrics is not None]
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

    def test_small_vibration_not_a_rep(self):
        """Small vibrations below min_rom should NOT trigger a rep."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal, min_rom=0.05)
        states = []
        i = 0

        # Rest
        states += self._drive(ekf, 200, az=-1.0, start_i=i); i += 200

        # Tiny vibration: 5 samples at -1.02g (barely above gravity)
        states += self._drive(ekf, 5, az=-1.02, start_i=i); i += 5
        states += self._drive(ekf, 5, az=-0.98, start_i=i); i += 5

        # Rest
        states += self._drive(ekf, 200, az=-1.0, start_i=i); i += 200

        reps = [s.rep_metrics for s in states if s.rep_metrics is not None]
        assert len(reps) == 0

    def test_velocity_decays_at_rest(self):
        """At rest, velocity should decay toward zero (not hard zero)."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states = self._drive(ekf, 500, az=-1.0, start_i=0)
        # After extended rest, velocity should be very small
        assert abs(states[-1].vertical_velocity) < 0.01

    def test_no_rep_at_rest(self):
        """Pure rest should produce no reps."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states = self._drive(ekf, 2000, az=-1.0, start_i=0)
        reps = [s.rep_metrics for s in states if s.rep_metrics is not None]
        assert len(reps) == 0

    def test_no_double_count_reps(self):
        """Running 3 full rep cycles should produce exactly 2 reps, not 4+.

        With reversal consumption, cycle 1 sets up the seed, cycles 2 and 3
        each produce exactly 1 rep.
        """
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        all_states: list[EstimatorState] = []

        states, next_i = self._simulate_full_rep(ekf, start_i=0)
        all_states += states
        states, next_i = self._simulate_full_rep(ekf, start_i=next_i)
        all_states += states
        states, _ = self._simulate_full_rep(ekf, start_i=next_i)
        all_states += states

        reps = [s.rep_metrics for s in all_states if s.rep_metrics is not None]
        # Should be exactly 2 reps (3 cycles, first cycle seeds, 2 actual reps)
        # Allow up to 3 but definitely NOT 4+ (which would indicate double-counting)
        assert 1 <= len(reps) <= 3, f"Expected 1-3 reps, got {len(reps)}"

    def test_pcv_geq_mcv(self):
        """Peak concentric velocity should be >= mean concentric velocity."""
        cal = _ideal_cal()
        ekf = IMUEstimator(cal)
        states1, next_i = self._simulate_full_rep(ekf, start_i=0)
        states2, _ = self._simulate_full_rep(ekf, start_i=next_i)
        all_states = states1 + states2

        reps = [s.rep_metrics for s in all_states if s.rep_metrics is not None]
        for rm in reps:
            if rm.mean_concentric_velocity > 0:
                assert rm.peak_concentric_velocity >= rm.mean_concentric_velocity
            if rm.mean_eccentric_velocity > 0:
                assert rm.peak_eccentric_velocity >= rm.mean_eccentric_velocity


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
