#!/usr/bin/env python3
"""
vbt.py — Velocity-Based Training application.

Reads IMU packets from the FPGA, runs the EKF estimator, and streams
real-time velocity / position / rep metrics to the console.

Usage
-----
  python3 -m host.vbt                       # auto-detect port
  python3 -m host.vbt /dev/ttyUSB1          # explicit port
  python3 -m host.vbt --csv > session.csv   # log to CSV
  python3 -m host.vbt --cal-samples 500     # longer calibration

Workflow
--------
1. Hold sensor **perfectly still** at startup → calibration runs (~1 s).
2. Move freely → velocity and displacement are tracked per rep.
3. At each rest phase the ZUPT resets drift and a rep summary is printed.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time

import numpy as np

from .calibration import calibrate, CalibrationResult
from .estimator import IMUEstimator, EstimatorState
from .imu_driver import IMUSample, stream, find_port

# ── Rep tracker ─────────────────────────────────────────────────────────────

class RepTracker:
    """
    Detect reps and compute per-rep peak/mean velocity.

    A "rep" is a movement phase bracketed by two stationary (ZUPT) phases.
    """
    def __init__(self):
        self.in_rep = False
        self.rep_count = 0
        self.peak_speed = 0.0
        self.speed_sum = 0.0
        self.speed_n = 0
        self.peak_disp = 0.0
        self._last_was_stationary = True

    def update(self, state: EstimatorState) -> str | None:
        """
        Feed one estimator state.  Returns a rep summary string when a rep
        completes, otherwise None.
        """
        if state.is_stationary:
            if self.in_rep:
                # Rep just ended
                self.in_rep = False
                mean_v = self.speed_sum / max(self.speed_n, 1)
                summary = (
                    f"  ▸ Rep {self.rep_count}:  "
                    f"peak {self.peak_speed:.3f} m/s  "
                    f"mean {mean_v:.3f} m/s  "
                    f"disp {self.peak_disp:.3f} m"
                )
                # Reset for next rep
                self.peak_speed = 0.0
                self.speed_sum = 0.0
                self.speed_n = 0
                self.peak_disp = 0.0
                return summary
            self._last_was_stationary = True
        else:
            if self._last_was_stationary and not self.in_rep:
                # Movement started → new rep
                self.in_rep = True
                self.rep_count += 1
                self._last_was_stationary = False

            if self.in_rep:
                self.peak_speed = max(self.peak_speed, state.speed)
                self.speed_sum += state.speed
                self.speed_n += 1
                disp = float(np.linalg.norm(state.position))
                self.peak_disp = max(self.peak_disp, disp)

        return None


# ── Main ────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="VBT — Velocity-Based Training")
    ap.add_argument("port", nargs="?", help="Serial port (auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=115_200)
    ap.add_argument("--cal-samples", type=int, default=500,
                    help="Calibration samples (default 500 ≈ 1.1 s)")
    ap.add_argument("--csv", action="store_true", help="CSV output mode")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("ERROR: No serial port found.")

    # ── Graceful exit on Ctrl-C ──
    stop = False
    def _sigint(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, _sigint)

    # ── Phase 1: Calibration ────────────────────────────────────────────
    print(f"\n╔{'═'*58}╗")
    print(f"║  VBT — Velocity-Based Training                           ║")
    print(f"╠{'═'*58}╣")
    print(f"║  Port: {port:<51s}║")
    print(f"╚{'═'*58}╝\n")
    print(f"  ▶ Hold sensor STILL — calibrating ({args.cal_samples} samples) …", flush=True)

    cal_samples: list[IMUSample] = []
    imu = stream(port, args.baud)

    for sample in imu:
        cal_samples.append(sample)
        if len(cal_samples) >= args.cal_samples:
            break
        if stop:
            sys.exit("\nAborted during calibration.")

    cal = calibrate(cal_samples)
    print(f"  ✓ Calibration done.\n")
    print(cal.summary())

    # ── Phase 2: Streaming ──────────────────────────────────────────────
    ekf = IMUEstimator(cal)
    tracker = RepTracker()
    count = 0
    t0 = time.monotonic()

    if args.csv:
        print("seq,t,vx,vy,vz,speed,px,py,pz,roll,pitch,yaw,stationary")
    else:
        print(f"{'#':>6}  {'Speed':>8} {'Vx':>8} {'Vy':>8} {'Vz':>8}  "
              f"{'Disp':>7}  {'Roll':>7} {'Pitch':>7}  {'State':>5}")
        print("─" * 90)

    for sample in imu:
        if stop:
            break

        state = ekf.update(sample)
        count += 1

        # ── Rep tracking ──
        rep_msg = tracker.update(state)
        if rep_msg and not args.csv:
            sys.stdout.write(f"\n{rep_msg}\n")

        # ── Display ──
        if args.csv:
            e = state.euler
            v = state.velocity
            p = state.position
            st = 1 if state.is_stationary else 0
            print(f"{count},{sample.t:.6f},"
                  f"{v[0]:.6f},{v[1]:.6f},{v[2]:.6f},{state.speed:.6f},"
                  f"{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},"
                  f"{e[0]:.2f},{e[1]:.2f},{e[2]:.2f},{st}")
        else:
            v = state.velocity
            disp = float(np.linalg.norm(state.position))
            e = state.euler
            marker = "REST" if state.is_stationary else "MOVE"
            line = (f"{count:6d}  {state.speed:8.4f} "
                    f"{v[0]:+8.4f} {v[1]:+8.4f} {v[2]:+8.4f}  "
                    f"{disp:7.4f}  "
                    f"{e[0]:+7.1f} {e[1]:+7.1f}  {marker:>5s}")
            sys.stdout.write(f"\r{line}")
            sys.stdout.flush()
            if count % 100 == 0:
                sys.stdout.write("\n")

    # ── Shutdown ──
    elapsed = time.monotonic() - t0
    hz = count / elapsed if elapsed > 0 else 0
    if not args.csv:
        print(f"\n\n{'═'*58}")
        print(f"  Processed {count} samples in {elapsed:.1f} s  ({hz:.0f} Hz)")
        print(f"  Reps detected: {tracker.rep_count}")
        print(f"{'═'*58}\n")


if __name__ == "__main__":
    main()
