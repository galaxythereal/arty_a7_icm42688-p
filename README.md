# ICM-42688-P IMU Streaming on Arty A7-100T (Verilog)

A complete FPGA-based IMU streaming pipeline for the Arty A7-100T: SPI sensor interface, continuous burst polling, and UART packet streaming at ~1 kHz. This repo includes a full open-source toolchain flow, unit/integration testbenches, and a Python host decoder.

**Status:** 123/123 tests passing ✅

---

## Highlights
- Fully open-source toolchain (Yosys → nextpnr-xilinx → prjxray → openFPGALoader)
- Deterministic SPI Mode 0 polling with continuous burst reads
- Robust UART packetizer with fixed header/trailer and host decoder
- Verified endianness handling (INTF_CONFIG0 bit4 = 1)
- Clean testbench suite (unit + integration)

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
MIT License. See LICENSE.# ICM-42688-P IMU Streaming — Arty A7-100T (Verilog)

## Test Results: 79/79 PASS ✅

| Testbench | Tests | Status |
|---|---|---|
| `tb_spi_master` | 31 | ALL PASS |
| `tb_uart_tx` | 23 | ALL PASS |
| `tb_icm42688_ctrl` | 10 | ALL PASS |
| `tb_top` (integration) | 15 | ALL PASS |

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

## Files

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
  tb_top.v             - Full integration test + UART receiver

constraints/
  arty_a7_100t.xdc     - Pin constraints (nextpnr-xilinx)
  arty_a7_100t_vivado.xdc - Pin constraints (Vivado)

scripts/
  run_sim.sh           - Run all simulation testbenches

chipdb/
  xc7a100tcsg324-1.bin - nextpnr-xilinx chip database

build/                   - Build output (generated)
Makefile                 - Open-source toolchain build
```

## Hardware Connections

| FPGA Pin | Pmod JA | IMU Signal |
|----------|---------|------------|
| G13      | JA1     | MOSI (SDI) |
| B11      | JA2     | MISO (SDO) |
| A11      | JA3     | SCK        |
| D12      | JA4     | CS_N       |
| D10      | UART    | TX → PC    |

**IMU Power:** Connect VDD and VDDIO to 3.3V (Pmod VCC), GND to GND.

## IMU Configuration
- **Gyroscope:** ±2000 dps, 1 kHz ODR, Low-Noise mode
- **Accelerometer:** ±16g, 1 kHz ODR, Low-Noise mode
- **SPI:** Mode 0, 1 MHz, 4-wire

## UART Output Format
Binary packet, 18 bytes, at ~1 kHz:
```
[0xAA][0x55][AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L]
[GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L][T_H][T_L][0x0D][0x0A]
```

### Converting Raw Values
- **Accel (±16g):** `value_g = raw / 2048.0`
- **Gyro (±2000dps):** `value_dps = raw / 16.4`
- **Temp:** `value_C = (raw / 132.48) + 25.0`

## Build (Open-Source Toolchain)

Uses **Yosys** → **nextpnr-xilinx** → **prjxray** → **openFPGALoader**.

```bash
# First time: copy pre-built chipdb (or 'make chipdb' to generate from scratch)
make chipdb-copy

# Full build: synthesis → place & route → bitstream
make

# Program FPGA (volatile, lost on power cycle)
make program

# Write to SPI flash (persistent)
make flash

# See all targets
make help
```

### Required Tools (snap)
```bash
snap install yosys
snap install openxc7 --classic
apt install openfpgaloader
```

## Build (Vivado, alternative)

1. Create project targeting `xc7a100tcsg324-1`
2. Add all `rtl/` files and `constraints/arty_a7_100t_vivado.xdc`
3. Set `top` as top module
4. Run synthesis → implementation → generate bitstream
5. Program via USB-JTAG

## Simulation (Icarus Verilog)

```bash
# Run all testbenches
make sim

# Or run individually
make sim-spi
make sim-uart
make sim-ctrl
make sim-top
```

## LEDs
| LED | Meaning |
|-----|---------|
| LD0 (green) | Init done |
| LD1 (blue) | Packet being sent |
| LD2 (red) | Error (WHO_AM_I fail) |
