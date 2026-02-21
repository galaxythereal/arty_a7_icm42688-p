#!/usr/bin/env python3
"""
vbt.py -- Velocity-Based Training application.

Reads IMU packets from the FPGA, runs the per-rep dead-reckoning estimator,
and streams real-time velocity / rep metrics to the console.

Usage
-----
  python3 -m host.vbt                       # auto-detect port
  python3 -m host.vbt /dev/ttyUSB1          # explicit port
  python3 -m host.vbt --csv > session.csv   # log to CSV
  python3 -m host.vbt --cal-samples 500     # longer calibration

VBT Metrics (per rep)
---------------------
  MCV   Mean Concentric Velocity (m/s) -- gold standard
  PCV   Peak Concentric Velocity (m/s)
  MEV   Mean Eccentric Velocity  (m/s)
  PEV   Peak Eccentric Velocity  (m/s)
  ROM   Range of Motion (m)
  Dur   Rep Duration (s)

Workflow
--------
1. Hold sensor STILL at startup -> calibration (~1 s).
2. Perform reps -> per-rep metrics are printed after each rep completes.
3. Each rest phase resets the dead-reckoning (zero drift by design).
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

import numpy as np

from .calibration import calibrate, CalibrationResult
from .estimator import IMUEstimator, EstimatorState, RepMetrics
from .imu_driver import IMUSample, stream, find_port


# -- Velocity zone classification (Gonzalez-Badillo et al.) --------------------

def velocity_zone(mcv: float) -> str:
    """Classify rep by mean concentric velocity zone."""
    if mcv >= 1.30:
        return "EXPLOSIVE"
    elif mcv >= 1.00:
        return "SPEED-STR"
    elif mcv >= 0.75:
        return "STR-SPEED"
    elif mcv >= 0.50:
        return "STRENGTH"
    elif mcv >= 0.35:
        return "ACCEL-STR"
    elif mcv >= 0.17:
        return "MAX-STR"
    else:
        return "GRIND"


def estimated_1rm_pct(mcv: float) -> float:
    """
    Estimate %1RM from MCV using the Jidovtseff load-velocity relationship.

    Based on the linear regression:  MCV = 1.75 - 0.0165 * %1RM
    Rearranged:  %1RM = (1.75 - MCV) / 0.0165

    Returns value clamped to [30, 100].
    """
    pct = (1.75 - mcv) / 0.0165
    return max(30.0, min(100.0, pct))


# -- Fatigue monitor -----------------------------------------------------------

class FatigueMonitor:
    """
    Track velocity loss across a set to detect fatigue.

    Velocity loss = (MCV_best - MCV_current) / MCV_best * 100

    Research recommends stopping the set when velocity loss exceeds
    20-30% for strength, 10-15% for power training.
    """
    def __init__(self):
        self.best_mcv = 0.0
        self.reps_in_set: list[float] = []

    def add_rep(self, mcv: float) -> tuple[float, float]:
        """
        Add a rep and return (velocity_loss_pct, avg_mcv).
        """
        self.reps_in_set.append(mcv)
        if mcv > self.best_mcv:
            self.best_mcv = mcv

        if self.best_mcv > 0:
            loss = (self.best_mcv - mcv) / self.best_mcv * 100.0
        else:
            loss = 0.0

        avg = sum(self.reps_in_set) / len(self.reps_in_set)
        return loss, avg

    def reset_set(self) -> None:
        self.best_mcv = 0.0
        self.reps_in_set.clear()


# -- Main ----------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="VBT -- Velocity-Based Training")
    ap.add_argument("port", nargs="?", help="Serial port (auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=115_200)
    ap.add_argument("--cal-samples", type=int, default=500,
                    help="Calibration samples (default 500)")
    ap.add_argument("--csv", action="store_true", help="CSV output mode")
    ap.add_argument("--fatigue-limit", type=float, default=25.0,
                    help="Velocity loss %% to warn (default 25)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("ERROR: No serial port found.")

    stop = False
    def _sigint(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, _sigint)

    # -- Phase 1: Calibration --
    print(f"\n{'='*62}")
    print(f"  VBT -- Velocity-Based Training (Per-Rep Dead-Reckoning)")
    print(f"{'='*62}")
    print(f"  Port: {port}")
    print(f"{'='*62}\n")
    print(f"  >> Hold sensor STILL -- calibrating ({args.cal_samples} samples) ...",
          flush=True)

    cal_samples: list[IMUSample] = []
    imu = stream(port, args.baud)

    for sample in imu:
        cal_samples.append(sample)
        if len(cal_samples) >= args.cal_samples:
            break
        if stop:
            sys.exit("\nAborted during calibration.")

    cal = calibrate(cal_samples)
    print(f"  >> Calibration done.\n")
    print(cal.summary())

    # -- Phase 2: Streaming --
    ekf = IMUEstimator(cal)
    fatigue = FatigueMonitor()
    count = 0
    t0 = time.monotonic()

    if args.csv:
        print("seq,t,vz,speed,dz,roll,pitch,yaw,stationary,moving,"
              "rep,mcv,pcv,mev,pev,rom,dur,zone,pct_1rm,vloss")
    else:
        print(f"  Waiting for reps... (move the barbell)\n")
        print(f"{'Rep':>4}  {'MCV':>6} {'PCV':>6} {'MEV':>6}  "
              f"{'ROM':>5}  {'Dur':>4}  {'Zone':<10}  "
              f"{'~%1RM':>5}  {'VLoss':>5}")
        print("-" * 72)

    for sample in imu:
        if stop:
            break

        state = ekf.update(sample)
        count += 1

        # -- Rep completed --
        rm = state.rep_metrics
        if rm is not None:
            vloss, avg_mcv = fatigue.add_rep(rm.mean_concentric_velocity)
            zone = velocity_zone(rm.mean_concentric_velocity)
            pct = estimated_1rm_pct(rm.mean_concentric_velocity)

            if args.csv:
                e = state.euler
                print(f"{count},{sample.t:.6f},{state.vertical_velocity:.6f},"
                      f"{state.speed:.6f},{state.vertical_displacement:.6f},"
                      f"{e[0]:.2f},{e[1]:.2f},{e[2]:.2f},"
                      f"{1 if state.is_stationary else 0},"
                      f"{1 if state.is_moving else 0},"
                      f"{rm.rep_number},{rm.mean_concentric_velocity:.4f},"
                      f"{rm.peak_concentric_velocity:.4f},"
                      f"{rm.mean_eccentric_velocity:.4f},"
                      f"{rm.peak_eccentric_velocity:.4f},"
                      f"{rm.total_rom:.4f},{rm.rep_duration:.3f},"
                      f"{zone},{pct:.1f},{vloss:.1f}")
            else:
                line = (
                    f"{rm.rep_number:4d}  "
                    f"{rm.mean_concentric_velocity:6.3f} "
                    f"{rm.peak_concentric_velocity:6.3f} "
                    f"{rm.mean_eccentric_velocity:6.3f}  "
                    f"{rm.total_rom:5.3f}  "
                    f"{rm.rep_duration:4.2f}  "
                    f"{zone:<10s}  "
                    f"{pct:5.1f}  "
                    f"{vloss:5.1f}%"
                )
                print(line)

                # Fatigue warning
                if vloss >= args.fatigue_limit:
                    print(f"  !! FATIGUE WARNING: {vloss:.0f}% velocity loss "
                          f"(limit {args.fatigue_limit:.0f}%)")

        # -- Live display (non-CSV, non-rep) --
        elif not args.csv:
            v = state.vertical_velocity
            d = state.vertical_displacement
            marker = "REST" if state.is_stationary else "MOVE"
            live = (f"\r  [{marker:4s}]  Vz={v:+7.3f} m/s  "
                    f"Dz={d:+7.4f} m  "
                    f"({count} samples)")
            sys.stdout.write(live)
            sys.stdout.flush()

    # -- Shutdown --
    elapsed = time.monotonic() - t0
    hz = count / elapsed if elapsed > 0 else 0
    if not args.csv:
        print(f"\n\n{'='*62}")
        print(f"  Processed {count} samples in {elapsed:.1f} s  ({hz:.0f} Hz)")
        n_reps = fatigue.reps_in_set
        if n_reps:
            avg = sum(n_reps) / len(n_reps)
            print(f"  Reps: {len(n_reps)}  |  Avg MCV: {avg:.3f} m/s")
        print(f"{'='*62}\n")


if __name__ == "__main__":
    main()
