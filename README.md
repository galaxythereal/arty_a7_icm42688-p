# ICM-42688-P IMU Streaming on Arty A7-100T (Verilog)

A complete FPGA-based IMU streaming pipeline for the Arty A7-100T: SPI sensor interface, continuous burst polling, and UART packet streaming at ~1 kHz. This repo includes a full open-source toolchain flow, unit/integration testbenches, and a Python host decoder.

**Status:** 123/123 RTL tests + 19/19 estimator tests passing ✅

---

## Highlights
- Fully open-source toolchain (Yosys → nextpnr-xilinx → prjxray → openFPGALoader)
- Deterministic SPI Mode 0 polling with continuous burst reads
- Robust UART packetizer with fixed header/trailer and host decoder
- Verified endianness handling (INTF_CONFIG0 bit4 = 1)
- Clean testbench suite (unit + integration)
- **Host-side EKF** — 13-state tactical-grade orientation, velocity, and position estimation with SHOE ZUPT, online accel-bias tracking, and velocity decay
- **ZUPT** (zero-velocity updates) for drift-free rep-by-rep velocity tracking
- **VBT app** — real-time rep detection with peak/mean velocity metrics
- **Live dashboard** — 4-panel matplotlib visualization (accel, velocity, orientation, rep bars)

---

## Architecture

```
           ┌──────────────────────────────────────┐
           │              top.v                   │
           │  ┌──────────────────┐  ┌──────────┐  │
IMU ◄─SPI──┤  │ icm42688_ctrl.v │  │  uart_tx │  ├──► USB UART
ICM-42688-P│  │  ┌────────────┐  │  │          │  │    115200 baud
           │  │  │ spi_master │  │  └──────────┘  │
           │  │  └────────────┘  │  ┌────────────┐│
           │  └──────────────────┘  │imu_data_   ││
           │                        │stream.v    ││
           │                        └────────────┘│
           └──────────────────────────────────────┘
```

---

## Hardware Connections

| FPGA Pin | Pmod JA | IMU Signal |
|----------|---------|------------|
| G13      | JA1     | MOSI (SDI) |
| B11      | JA2     | MISO (SDO) |
| A11      | JA3     | SCK        |
| D12      | JA4     | CS_N       |
| D10      | UART    | TX → PC    |

**IMU Power:** VDD and VDDIO to 3.3V (Pmod VCC), GND to GND.

---

## IMU Configuration
- **Gyroscope:** ±2000 dps, 1 kHz ODR, Low-Noise
- **Accelerometer:** ±16g, 1 kHz ODR, Low-Noise
- **SPI:** Mode 0, 1 MHz, 4-wire
- **Endianness:** Big-endian data (INTF_CONFIG0 bit4 = 1)

---

## UART Packet Format
Binary packet, 18 bytes:
```
[0xAA][0x55][AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L]
[GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L][T_H][T_L][0x0D][0x0A]
```

### Converting Raw Values
- **Accel (±16g):** `value_g = raw / 2048.0`
- **Gyro (±2000dps):** `value_dps = raw / 16.384`
- **Temp:** `value_C = (raw / 132.48) + 25.0`

---

## Quick Start (Open-Source Toolchain)

```bash
# First time: copy pre-built chipdb (or build from scratch)
make chipdb-copy

# Full build: synth → place & route → bitstream
make

# Program FPGA (volatile)
make program

# Run simulations
make sim
```

### Required Tools (snap)
```bash
snap install yosys
snap install openxc7 --classic
apt install openfpgaloader
```

---

## Streaming Demo

```bash
python3 scripts/stream_imu.py
```

Expected at rest: |accel| ≈ 1g (direction depends on board orientation), gyro near 0 °/s, temperature near ambient.

---

## Velocity-Based Training (VBT)

The host-side pipeline turns raw IMU data into real-time velocity and position:

```
FPGA (raw packets) ──UART──► imu_driver ──► calibration ──► EKF estimator ──► VBT app
                              (decode)       (bias removal)  (orientation +    (rep detect,
                                                              velocity +       peak/mean v)
                                                              ZUPT)
```

### Quick Start

```bash
pip3 install numpy pyserial matplotlib

# Run VBT app (hold sensor still for ~1 s calibration, then move)
python3 -m host.vbt

# Launch real-time dashboard
python3 -m host.dashboard

# Log session to CSV
python3 -m host.vbt --csv > session.csv
```

### How It Works
1. **Calibration** — collects 500 samples at rest, estimates gyro bias, accel bias, and gravity direction.
2. **13-state EKF** — fuses gyro (prediction) + accelerometer gravity reference (correction) to track orientation as a quaternion. Tracks gyro bias, accel bias, and velocity online. Rotates accel into the world frame and subtracts gravity to get linear acceleration.
3. **Velocity integration** — integrates linear acceleration for velocity and position. Exponential decay attenuates low-frequency drift between ZUPTs.
4. **SHOE ZUPT** — Stance Hypothesis Optimal dEtection (Skog et al.) uses variance-based test statistic over sliding window — far more robust than max-threshold. Detects stationary phases and resets velocity to zero with tight (0.001 m/s) noise.
5. **Adaptive accel correction** — measurement noise scales quadratically with |a| deviation from 1g, smoothly de-weighting during high dynamics.
6. **Rep tracking** — each movement phase between two rest phases is a "rep", with peak and mean velocity reported.

### EKF State Vector (13)
| Index | State | Description |
|-------|-------|-------------|
| 0–3   | q     | Orientation quaternion (scalar-first) |
| 4–6   | bg    | Gyro bias (°/s) |
| 7–9   | ba    | Accel bias (g) — tracked online |
| 10–12 | v     | Velocity in world frame (m/s) |

### Estimator Tests

```bash
python3 -m pytest host/tests/test_estimator.py -v
```

| Test Suite | Tests | Status |
|---|---:|---|
| Quaternion ops | 6 | ALL PASS |
| ZUPT detector (SHOE) | 4 | ALL PASS |
| Calibration | 4 | ALL PASS |
| EKF integration | 10 | ALL PASS |

---

## Simulation Suite (Icarus Verilog)

```bash
make sim
```

**Testbench breakdown:**
| Testbench | Tests | Status |
|---|---:|---|
| tb_top_extended | 44 | ALL PASS |
| tb_spi_master | 31 | ALL PASS |
| tb_uart_tx | 23 | ALL PASS |
| tb_icm42688_ctrl | 10 | ALL PASS |
| tb_top | 15 | ALL PASS |

---

## Project Structure

```
rtl/
  icm42688_regs.vh     - Register address definitions
  spi_master.v         - SPI Mode 0 master, configurable CLK_DIV
  icm42688_ctrl.v      - IMU FSM: init + continuous polling
  uart_tx.v            - 8N1 UART transmitter
  imu_data_stream.v    - Assembles binary packets → UART
  top.v                - Top-level for Arty A7-100T

tb/
  tb_spi_master.v      - SPI master unit tests
  tb_uart_tx.v         - UART transmitter unit tests
  tb_icm42688_ctrl.v   - IMU controller with SPI slave model
  tb_top.v             - Integration test + UART receiver
  tb_top_extended.v    - Extended integration test

constraints/
  arty_a7_100t.xdc     - Pin constraints (nextpnr-xilinx)
  arty_a7_100t_vivado.xdc - Pin constraints (Vivado)

scripts/
  run_sim.sh           - Run all simulation testbenches
  stream_imu.py        - UART stream decoder + live metrics

host/                    - Host-side VBT pipeline (Python)
  __init__.py
  imu_driver.py        - Packet decode from FPGA UART
  calibration.py       - Static bias estimation
  estimator.py         - EKF: orientation + velocity + ZUPT
  vbt.py               - VBT app: rep detection + metrics
  dashboard.py         - Real-time 4-panel matplotlib dashboard
  tests/
    test_estimator.py  - Offline tests for the estimator

chipdb/
  xc7a100tcsg324-1.bba - nextpnr-xilinx chip database (optional)

build/                 - Build output (generated, ignored)
Makefile               - Build and simulation targets
```

---

## LEDs
| LED | Meaning |
|-----|---------|
| LD0 (green) | Init done |
| LD1 (blue) | Packet being sent |
| LD2 (red) | Error (WHO_AM_I fail) |

---

## License
MIT License. See LICENSE.
