#!/usr/bin/env python3
"""
calibration.py — Static bias estimation for ICM-42688-P.

Collects *n* samples while the sensor is at rest and computes:
  • gyro bias   (°/s)  — mean of gyro readings
  • accel bias  (g)    — deviation of measured gravity from [0, 0, ±1]

The caller must ensure the sensor is **stationary** during calibration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

from .imu_driver import IMUSample


@dataclass
class CalibrationResult:
    """Bias offsets estimated from a static capture."""
    gyro_bias: np.ndarray     # [bgx, bgy, bgz]  °/s
    accel_bias: np.ndarray    # [bax, bay, baz]  g  (deviation from unit gravity)
    gravity_mag: float        # measured |g|  (should be ≈1.0)
    gravity_dir: np.ndarray   # unit vector pointing toward gravity in sensor frame
    n_samples: int
    std_gyro: np.ndarray      # per-axis gyro std  (noise floor metric)
    std_accel: np.ndarray     # per-axis accel std

    def summary(self) -> str:
        bg = self.gyro_bias
        ba = self.accel_bias
        return (
            f"Calibration ({self.n_samples} samples)\n"
            f"  Gyro bias : [{bg[0]:+.4f}, {bg[1]:+.4f}, {bg[2]:+.4f}] °/s\n"
            f"  Accel bias: [{ba[0]:+.4f}, {ba[1]:+.4f}, {ba[2]:+.4f}] g\n"
            f"  |g|       : {self.gravity_mag:.4f} g\n"
            f"  Gravity   : [{self.gravity_dir[0]:+.4f}, {self.gravity_dir[1]:+.4f}, "
            f"{self.gravity_dir[2]:+.4f}]\n"
            f"  σ gyro    : [{self.std_gyro[0]:.4f}, {self.std_gyro[1]:.4f}, "
            f"{self.std_gyro[2]:.4f}] °/s\n"
            f"  σ accel   : [{self.std_accel[0]:.4f}, {self.std_accel[1]:.4f}, "
            f"{self.std_accel[2]:.4f}] g\n"
        )


def calibrate(samples: List[IMUSample]) -> CalibrationResult:
    """
    Compute bias offsets from a list of *stationary* samples.

    Parameters
    ----------
    samples : list[IMUSample]
        At least 100 samples collected while the sensor is perfectly still.

    Returns
    -------
    CalibrationResult
    """
    if len(samples) < 50:
        raise ValueError(f"Need ≥50 samples for calibration, got {len(samples)}")

    accel = np.array([[s.ax, s.ay, s.az] for s in samples])   # (N, 3)
    gyro  = np.array([[s.gx, s.gy, s.gz] for s in samples])   # (N, 3)

    # ── Gyro bias: mean of readings at rest (ideally 0 °/s) ──
    gyro_bias = gyro.mean(axis=0)
    std_gyro  = gyro.std(axis=0)

    # ── Accel: measured gravity vector ──
    accel_mean = accel.mean(axis=0)
    g_mag = np.linalg.norm(accel_mean)
    g_dir = accel_mean / g_mag if g_mag > 0.5 else np.array([0.0, 0.0, 1.0])

    # Bias = measured − expected.  Expected gravity = g_dir * 1.0 g.
    accel_bias = accel_mean - g_dir   # ideally [0,0,0] if |g|==1
    std_accel  = accel.std(axis=0)

    return CalibrationResult(
        gyro_bias=gyro_bias,
        accel_bias=accel_bias,
        gravity_mag=g_mag,
        gravity_dir=g_dir,
        n_samples=len(samples),
        std_gyro=std_gyro,
        std_accel=std_accel,
    )
