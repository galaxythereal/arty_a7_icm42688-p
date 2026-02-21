#!/usr/bin/env python3
"""
stream_imu.py — Read & verify ICM-42688-P binary packets from Arty A7 UART

Packet format (18 bytes):
  [0xAA][0x55]                           — header
  [AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L]  — accel  (signed 16-bit, ±16g)
  [GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L]  — gyro   (signed 16-bit, ±2000dps)
  [T_H][T_L]                             — temp   (signed 16-bit)
  [0x0D][0x0A]                           — CRLF

Usage:
  python3 scripts/stream_imu.py                    # live stream (Ctrl+C to stop)
  python3 scripts/stream_imu.py /dev/ttyUSB1       # explicit port
  python3 scripts/stream_imu.py -n 200             # stop after 200 packets
  python3 scripts/stream_imu.py --verify           # capture & verify report
  python3 scripts/stream_imu.py --raw              # hex dump
  python3 scripts/stream_imu.py --csv > data.csv   # CSV output
"""

import sys, time, signal, argparse, glob
import serial, serial.tools.list_ports

# ── Constants ────────────────────────────────────────────────────────────────
BAUD      = 115_200
PKT_LEN   = 18
HEADER    = b'\xAA\x55'
TRAILER   = b'\x0D\x0A'
ACCEL_LSB = 2048.0       # LSB/g  (±16g)  (datasheet Table 2)
GYRO_LSB  = 16.384       # LSB/dps (±2000dps) 32768/2000
TEMP_SENS = 132.48        # LSB/°C
TEMP_OFF  = 25.0           # °C offset


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 0x10000 if v >= 0x8000 else v


def find_port():
    for p in serial.tools.list_ports.comports():
        d = ((p.description or "") + (p.manufacturer or "")).lower()
        if any(k in d for k in ("ftdi", "ft2232", "digilent", "arty", "uart")):
            return p.device
    usbs = sorted(glob.glob("/dev/ttyUSB*"))
    return usbs[1] if len(usbs) >= 2 else (usbs[0] if usbs else None)


class Stats:
    def __init__(self):
        self.total = self.good = self.bad_hdr = self.bad_trail = self.bad_range = 0
        self.t0 = self.tN = None
        self.last_vals = None

    def rate(self):
        if self.t0 and self.tN and self.good > 1:
            dt = self.tN - self.t0
            return (self.good - 1) / dt if dt > 0 else 0
        return 0

    def report(self):
        r = self.rate()
        ok = self.bad_trail == 0 and self.bad_range < max(1, self.total * 0.05)
        return (
            f"\n{'═'*56}\n"
            f"  STREAM VERIFICATION SUMMARY\n"
            f"{'═'*56}\n"
            f"  Packets received  : {self.total}\n"
            f"  Valid             : {self.good}\n"
            f"  Bad trailer       : {self.bad_trail}\n"
            f"  Out-of-range      : {self.bad_range}\n"
            f"  Skipped bytes     : {self.bad_hdr}\n"
            f"  Packet rate       : {r:.1f} Hz\n"
            f"{'═'*56}\n"
            f"  {'✅ ALL GOOD' if self.total > 0 and ok else '❌ ISSUES DETECTED' if self.total > 0 else '❌ NO PACKETS — check wiring & FPGA'}\n"
        )


def range_ok(ax, ay, az, gx, gy, gz, tc):
    return (all(abs(a) <= 20 for a in (ax, ay, az)) and
            all(abs(g) <= 2200 for g in (gx, gy, gz)) and
            -40 <= tc <= 105)


def main():
    ap = argparse.ArgumentParser(description="ICM-42688 IMU stream viewer")
    ap.add_argument("port", nargs="?", help="Serial port (auto-detect if omitted)")
    ap.add_argument("-n", "--num", type=int, default=0, help="Stop after N good packets (0=forever)")
    ap.add_argument("-b", "--baud", type=int, default=BAUD)
    ap.add_argument("--raw", action="store_true", help="Hex dump mode")
    ap.add_argument("--csv", action="store_true", help="CSV output")
    ap.add_argument("--verify", action="store_true", help="Capture 500 packets & report")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("ERROR: No serial port found. Is the Arty connected?")
    if args.verify and args.num == 0:
        args.num = 500

    ser = serial.Serial(port, args.baud, timeout=0.5)
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.reset_input_buffer()

    st = Stats()
    stop = False
    signal.signal(signal.SIGINT, lambda *_: setattr(sys.modules[__name__], '_stop', True))

    if args.csv:
        print("pkt,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c")
    elif not args.raw:
        print(f"\nOpening {port} @ {args.baud} baud …\n")
        print(f"{'#':>6}  {'Ax(g)':>9} {'Ay(g)':>9} {'Az(g)':>9}  "
              f"{'Gx(°/s)':>10} {'Gy(°/s)':>10} {'Gz(°/s)':>10}  {'T(°C)':>7}  {'Hz':>6}")
        print("─" * 100)

    buf = bytearray()

    while not getattr(sys.modules[__name__], '_stop', False):
        # ── Read available bytes into buffer ──
        waiting = ser.in_waiting
        chunk = ser.read(max(waiting, 1))
        if not chunk:
            continue
        buf.extend(chunk)

        # ── Scan buffer for complete packets ──
        while len(buf) >= PKT_LEN:
            # Find header
            idx = buf.find(HEADER)
            if idx < 0:
                st.bad_hdr += len(buf) - 1
                buf = buf[-1:]      # keep last byte (could be 0xAA)
                break
            if idx > 0:
                st.bad_hdr += idx
                buf = buf[idx:]     # discard bytes before header
            if len(buf) < PKT_LEN:
                break               # need more data

            pkt = bytes(buf[:PKT_LEN])
            buf = buf[PKT_LEN:]

            # ── Verify trailer ──
            if pkt[-2:] != TRAILER:
                st.bad_trail += 1
                # Don't consume — rescan from byte after this header
                buf = bytearray(pkt[2:]) + buf
                continue

            # ── Parse ──
            st.total += 1
            now = time.monotonic()
            if st.t0 is None:
                st.t0 = now
            st.tN = now

            ax_r = s16(pkt[2],  pkt[3])
            ay_r = s16(pkt[4],  pkt[5])
            az_r = s16(pkt[6],  pkt[7])
            gx_r = s16(pkt[8],  pkt[9])
            gy_r = s16(pkt[10], pkt[11])
            gz_r = s16(pkt[12], pkt[13])
            t_r  = s16(pkt[14], pkt[15])

            ax = ax_r / ACCEL_LSB
            ay = ay_r / ACCEL_LSB
            az = az_r / ACCEL_LSB
            gx = gx_r / GYRO_LSB
            gy = gy_r / GYRO_LSB
            gz = gz_r / GYRO_LSB
            tc = t_r  / TEMP_SENS + TEMP_OFF

            if not range_ok(ax, ay, az, gx, gy, gz, tc):
                st.bad_range += 1

            st.good += 1
            st.last_vals = (ax, ay, az, gx, gy, gz, tc)

            # ── Display ──
            hz = st.rate()
            if args.raw:
                print(f"[{st.good:5d}] {pkt.hex(' ')}")
            elif args.csv:
                print(f"{st.good},{ax:.4f},{ay:.4f},{az:.4f},{gx:.2f},{gy:.2f},{gz:.2f},{tc:.2f}")
            else:
                line = (f"{st.good:6d}  {ax:+9.4f} {ay:+9.4f} {az:+9.4f}  "
                        f"{gx:+10.2f} {gy:+10.2f} {gz:+10.2f}  {tc:+7.2f}  {hz:6.0f}")
                sys.stdout.write(f"\r{line}")
                sys.stdout.flush()
                # Print full line every 100 packets so scroll shows history
                if st.good % 100 == 0:
                    sys.stdout.write("\n")

            # ── Stop condition ──
            if 0 < args.num <= st.good:
                break

        if 0 < args.num <= st.good:
            break

    ser.close()
    if not args.csv:
        print()
    print(st.report())

_stop = False
if __name__ == "__main__":
    main()
