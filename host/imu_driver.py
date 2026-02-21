#!/usr/bin/env python3
"""
imu_driver.py — Decode ICM-42688-P binary packets from Arty A7 UART.

Packet (18 bytes):
  [0xAA][0x55][AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L]
  [GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L][T_H][T_L][0x0D][0x0A]

Provides a generator that yields timestamped, scaled IMU samples.
"""

from __future__ import annotations

import glob
import struct
import time
from dataclasses import dataclass, field
from typing import Generator, Optional

import serial
import serial.tools.list_ports

# ── Sensor constants ────────────────────────────────────────────────────────
BAUD       = 115_200
PKT_LEN    = 18
HEADER     = b"\xAA\x55"
TRAILER    = b"\x0D\x0A"
ACCEL_LSB  = 2048.0     # LSB/g   (±16 g)
GYRO_LSB   = 16.384     # LSB/dps (±2000 dps)  32768/2000
TEMP_SENS  = 132.48     # LSB/°C
TEMP_OFF   = 25.0       # °C


@dataclass
class IMUSample:
    """One decoded IMU measurement."""
    t: float              # monotonic timestamp (s)
    ax: float = 0.0       # accel  X  (g)
    ay: float = 0.0       # accel  Y  (g)
    az: float = 0.0       # accel  Z  (g)
    gx: float = 0.0       # gyro   X  (°/s)
    gy: float = 0.0       # gyro   Y  (°/s)
    gz: float = 0.0       # gyro   Z  (°/s)
    temp: float = 0.0     # die temperature (°C)
    seq: int = 0          # packet sequence number

    # Raw values (for calibration / logging)
    ax_raw: int = 0
    ay_raw: int = 0
    az_raw: int = 0
    gx_raw: int = 0
    gy_raw: int = 0
    gz_raw: int = 0
    temp_raw: int = 0


def _s16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v >= 0x8000 else v


def find_port() -> Optional[str]:
    """Auto-detect the Arty A7 serial port."""
    for p in serial.tools.list_ports.comports():
        d = ((p.description or "") + (p.manufacturer or "")).lower()
        if any(k in d for k in ("ftdi", "ft2232", "digilent", "arty", "uart")):
            return p.device
    usbs = sorted(glob.glob("/dev/ttyUSB*"))
    return usbs[1] if len(usbs) >= 2 else (usbs[0] if usbs else None)


def stream(port: Optional[str] = None,
           baud: int = BAUD) -> Generator[IMUSample, None, None]:
    """
    Open *port* and yield one :class:`IMUSample` per valid packet.

    Handles header sync, trailer verification, and byte-level resync.
    """
    port = port or find_port()
    if port is None:
        raise RuntimeError("No serial port found.  Is the Arty connected?")

    ser = serial.Serial(port, baud, timeout=0.5)
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.reset_input_buffer()

    buf = bytearray()
    seq = 0

    try:
        while True:
            waiting = ser.in_waiting
            chunk = ser.read(max(waiting, 1))
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= PKT_LEN:
                idx = buf.find(HEADER)
                if idx < 0:
                    buf = buf[-1:]
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < PKT_LEN:
                    break

                pkt = bytes(buf[:PKT_LEN])
                buf = buf[PKT_LEN:]

                if pkt[-2:] != TRAILER:
                    buf = bytearray(pkt[2:]) + buf
                    continue

                seq += 1
                now = time.monotonic()

                ax_r = _s16(pkt[2],  pkt[3])
                ay_r = _s16(pkt[4],  pkt[5])
                az_r = _s16(pkt[6],  pkt[7])
                gx_r = _s16(pkt[8],  pkt[9])
                gy_r = _s16(pkt[10], pkt[11])
                gz_r = _s16(pkt[12], pkt[13])
                t_r  = _s16(pkt[14], pkt[15])

                yield IMUSample(
                    t=now, seq=seq,
                    ax=ax_r / ACCEL_LSB, ay=ay_r / ACCEL_LSB, az=az_r / ACCEL_LSB,
                    gx=gx_r / GYRO_LSB,  gy=gy_r / GYRO_LSB,  gz=gz_r / GYRO_LSB,
                    temp=t_r / TEMP_SENS + TEMP_OFF,
                    ax_raw=ax_r, ay_raw=ay_r, az_raw=az_r,
                    gx_raw=gx_r, gy_raw=gy_r, gz_raw=gz_r,
                    temp_raw=t_r,
                )
    finally:
        ser.close()
