#!/usr/bin/env python3
"""
dashboard.py — Real-time VBT dashboard with matplotlib.

Four-panel live visualization:
  ┌─────────────────┬─────────────────┐
  │  Linear Accel   │    Velocity     │
  │  (world, m/s²)  │    (m/s)        │
  ├─────────────────┼─────────────────┤
  │  Orientation    │   Rep Metrics   │
  │  (roll/pitch °) │   (bar chart)   │
  └─────────────────┴─────────────────┘

Usage
-----
  python3 -m host.dashboard                    # auto-detect port
  python3 -m host.dashboard /dev/ttyUSB1       # explicit port
  python3 -m host.dashboard --cal-samples 500  # longer calibration
  python3 -m host.dashboard --window 3.0       # 3 s rolling window
"""

from __future__ import annotations

import argparse
import collections
import signal
import sys
import time
import threading

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

from .calibration import calibrate
from .estimator import IMUEstimator, EstimatorState, RepMetrics
from .imu_driver import IMUSample, stream, find_port

# ── Circular buffer for rolling plots ──────────────────────────────────────

class RingBuffer:
    """Fixed-size ring buffer backed by numpy arrays."""
    def __init__(self, maxlen: int, ncols: int = 1):
        self.maxlen = maxlen
        self.ncols = ncols
        self.data = np.zeros((maxlen, ncols))
        self.idx = 0
        self.full = False

    def append(self, row: np.ndarray | list | tuple) -> None:
        self.data[self.idx] = row
        self.idx += 1
        if self.idx >= self.maxlen:
            self.idx = 0
            self.full = True

    def get(self) -> np.ndarray:
        if self.full:
            return np.roll(self.data, -self.idx, axis=0)
        return self.data[:self.idx]


# ── Shared state between IMU thread and plot thread ────────────────────────

class SharedState:
    def __init__(self, ring_len: int):
        self.lock = threading.Lock()
        self.accel = RingBuffer(ring_len, 3)    # ax, ay, az  world (m/s²)
        self.velocity = RingBuffer(ring_len, 2) # vz, |v|
        self.orient = RingBuffer(ring_len, 2)   # roll, pitch
        self.time_buf = RingBuffer(ring_len, 1) # relative time (s)
        self.speed_now = 0.0
        self.vz_now = 0.0
        self.is_stationary = True
        self.is_moving = False
        self.rep_count = 0
        self.rep_mcv: list[float] = []           # MCV per rep
        self.rep_pcv: list[float] = []           # PCV per rep
        self.rep_mev: list[float] = []           # MEV per rep
        self.t0: float | None = None
        self.count = 0
        self.hz = 0.0
        self.cal_done = False
        self.cal_progress = 0.0


# ── IMU processing thread ──────────────────────────────────────────────────

def _imu_thread(shared: SharedState, port: str, baud: int, cal_n: int) -> None:
    try:
        _imu_thread_inner(shared, port, baud, cal_n)
    except Exception as e:
        import traceback
        print(f"\n  [ERROR] IMU thread stopped: {e}", file=sys.stderr)
        traceback.print_exc()


def _imu_thread_inner(shared: SharedState, port: str, baud: int, cal_n: int) -> None:
    imu = stream(port, baud)

    # ── Calibration ──
    cal_samples: list[IMUSample] = []
    for sample in imu:
        cal_samples.append(sample)
        with shared.lock:
            shared.cal_progress = len(cal_samples) / cal_n
        if len(cal_samples) >= cal_n:
            break

    cal = calibrate(cal_samples)
    ekf = IMUEstimator(cal)

    with shared.lock:
        shared.cal_done = True

    t_start = time.monotonic()

    for sample in imu:
        state = ekf.update(sample)
        t_rel = time.monotonic() - t_start

        with shared.lock:
            shared.count += 1
            shared.speed_now = state.speed
            shared.vz_now = state.vertical_velocity
            shared.is_stationary = state.is_stationary
            shared.is_moving = state.is_moving

            shared.time_buf.append([t_rel])
            shared.accel.append(state.accel_world)
            shared.velocity.append([state.vertical_velocity, state.speed])
            shared.orient.append([state.euler[0], state.euler[1]])

            # Rep completed -- record VBT metrics
            if state.rep_metrics is not None:
                rm = state.rep_metrics
                shared.rep_count = rm.rep_number
                shared.rep_mcv.append(rm.mean_concentric_velocity)
                shared.rep_pcv.append(rm.peak_concentric_velocity)
                shared.rep_mev.append(rm.mean_eccentric_velocity)

            elapsed = t_rel
            if elapsed > 0:
                shared.hz = shared.count / elapsed


# ── Dashboard ──────────────────────────────────────────────────────────────

def _build_dashboard(shared: SharedState, window_sec: float):
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(14, 8))
    fig.canvas.manager.set_window_title("VBT Dashboard — ICM-42688-P")

    gs = gridspec.GridSpec(2, 2, hspace=0.35, wspace=0.30,
                           left=0.08, right=0.96, top=0.92, bottom=0.08)

    # ── Panel 1: Acceleration ──
    ax_acc = fig.add_subplot(gs[0, 0])
    ax_acc.set_title("Linear Acceleration (world)", fontsize=11, pad=8)
    ax_acc.set_ylabel("m/s²")
    ax_acc.set_xlabel("time (s)")
    ax_acc.set_ylim(-30, 30)
    line_ax, = ax_acc.plot([], [], lw=1, color="#FF6B6B", label="X")
    line_ay, = ax_acc.plot([], [], lw=1, color="#51CF66", label="Y")
    line_az, = ax_acc.plot([], [], lw=1, color="#339AF0", label="Z")
    ax_acc.legend(loc="upper right", fontsize=8)
    ax_acc.grid(alpha=0.2)

    # ── Panel 2: Vertical Velocity (VBT signal) ──
    ax_vel = fig.add_subplot(gs[0, 1])
    ax_vel.set_title("Vertical Velocity (VBT)", fontsize=11, pad=8)
    ax_vel.set_ylabel("m/s")
    ax_vel.set_xlabel("time (s)")
    ax_vel.set_ylim(-2, 2)
    line_vz, = ax_vel.plot([], [], lw=2, color="#339AF0", label="Vz")
    line_spd, = ax_vel.plot([], [], lw=1, color="#FCC419", alpha=0.6, label="|V|")
    ax_vel.legend(loc="upper right", fontsize=8)
    ax_vel.grid(alpha=0.2)

    # ── Panel 3: Orientation ──
    ax_ori = fig.add_subplot(gs[1, 0])
    ax_ori.set_title("Orientation", fontsize=11, pad=8)
    ax_ori.set_ylabel("degrees")
    ax_ori.set_xlabel("time (s)")
    ax_ori.set_ylim(-180, 180)
    line_roll, = ax_ori.plot([], [], lw=1.5, color="#DA77F2", label="Roll")
    line_pitch, = ax_ori.plot([], [], lw=1.5, color="#20C997", label="Pitch")
    ax_ori.legend(loc="upper right", fontsize=8)
    ax_ori.grid(alpha=0.2)

    # ── Panel 4: Rep metrics ──
    ax_rep = fig.add_subplot(gs[1, 1])
    ax_rep.set_title("Rep Metrics", fontsize=11, pad=8)
    ax_rep.set_ylabel("m/s")
    ax_rep.set_xlabel("Rep #")
    ax_rep.grid(alpha=0.2, axis="y")

    # ── Status bar ──
    status_text = fig.text(0.5, 0.97, "Calibrating...", ha="center", fontsize=12,
                           color="#FCC419", fontweight="bold")

    # ── Animation update ──
    def _update(frame):
        with shared.lock:
            t = shared.time_buf.get().flatten()
            acc = shared.accel.get()
            vel = shared.velocity.get()
            ori = shared.orient.get()
            speed = shared.speed_now
            vz_now = shared.vz_now
            stationary = shared.is_stationary
            is_moving = shared.is_moving
            reps = shared.rep_count
            mcv_list = list(shared.rep_mcv)
            pcv_list = list(shared.rep_pcv)
            mev_list = list(shared.rep_mev)
            hz = shared.hz
            cal_done = shared.cal_done
            cal_pct = shared.cal_progress
            count = shared.count

        if not cal_done:
            status_text.set_text(f"[CAL]  Calibrating ... {cal_pct*100:.0f}%  --  hold sensor still")
            status_text.set_color("#FCC419")
            return []

        # ── Time window ──
        if len(t) > 1:
            t_max = t[-1]
            t_min = max(0, t_max - window_sec)
        else:
            t_min, t_max = 0, window_sec

        # ── Accel ──
        if len(acc) > 0:
            line_ax.set_data(t, acc[:, 0])
            line_ay.set_data(t, acc[:, 1])
            line_az.set_data(t, acc[:, 2])
            ax_acc.set_xlim(t_min, t_max)
            a_max = max(np.abs(acc).max() * 1.2, 5)
            ax_acc.set_ylim(-a_max, a_max)

        # ── Velocity ──
        if len(vel) > 0:
            line_vz.set_data(t, vel[:, 0])
            line_spd.set_data(t, vel[:, 1])
            ax_vel.set_xlim(t_min, t_max)
            v_max = max(np.abs(vel).max() * 1.3, 0.5)
            ax_vel.set_ylim(-v_max, v_max)

        # ── Orientation ──
        if len(ori) > 0:
            line_roll.set_data(t, ori[:, 0])
            line_pitch.set_data(t, ori[:, 1])
            ax_ori.set_xlim(t_min, t_max)

        # ── Rep bars ──
        if len(mcv_list) > 0:
            ax_rep.cla()
            ax_rep.set_title("VBT Rep Metrics", fontsize=11, pad=8)
            ax_rep.set_ylabel("m/s")
            ax_rep.set_xlabel("Rep #")
            ax_rep.grid(alpha=0.2, axis="y")
            x = np.arange(1, len(mcv_list) + 1)
            w = 0.25
            ax_rep.bar(x - w, pcv_list, w, color="#FF6B6B", label="PCV", alpha=0.9)
            ax_rep.bar(x,     mcv_list, w, color="#339AF0", label="MCV", alpha=0.9)
            ax_rep.bar(x + w, mev_list, w, color="#51CF66", label="MEV", alpha=0.9)
            ax_rep.set_xticks(x)
            ax_rep.legend(fontsize=8)
        else:
            ax_rep.cla()
            ax_rep.set_title("VBT Rep Metrics", fontsize=11, pad=8)
            ax_rep.set_ylabel("m/s")
            ax_rep.set_xlabel("Rep #")
            ax_rep.grid(alpha=0.2, axis="y")
            ax_rep.text(0.5, 0.5, "Waiting for first rep...",
                        ha="center", va="center", transform=ax_rep.transAxes,
                        fontsize=11, color="#868E96")

        # ── Status ──
        state_str = "MOVE" if is_moving else "REST"
        state_col = "#FF6B6B" if is_moving else "#51CF66"
        status_text.set_text(
            f"Vz: {vz_now:+.3f} m/s   │   State: {state_str}   │   "
            f"Reps: {reps}   │   {hz:.0f} Hz   │   Samples: {count}"
        )
        status_text.set_color(state_col)

        return []

    ani = FuncAnimation(fig, _update, interval=50, blit=False, cache_frame_data=False)
    return fig, ani


# ── Main ────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="VBT Dashboard — real-time visualization")
    ap.add_argument("port", nargs="?", help="Serial port (auto-detect)")
    ap.add_argument("-b", "--baud", type=int, default=115_200)
    ap.add_argument("--cal-samples", type=int, default=500,
                    help="Calibration samples (default 500 ≈ 1.1 s)")
    ap.add_argument("--window", type=float, default=5.0,
                    help="Rolling plot window in seconds (default 5)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("ERROR: No serial port found.")

    # Ring buffer size: ~window * sample_rate
    ring_len = int(args.window * 500)
    shared = SharedState(ring_len)

    # ── Start IMU thread ──
    t = threading.Thread(target=_imu_thread,
                         args=(shared, port, args.baud, args.cal_samples),
                         daemon=True)
    t.start()

    print(f"\n  VBT Dashboard — {port} @ {args.baud} baud")
    print(f"  Hold sensor still for calibration …\n")

    # ── Launch plot (blocks on main thread) ──
    fig, ani = _build_dashboard(shared, args.window)
    plt.show()


if __name__ == "__main__":
    main()
