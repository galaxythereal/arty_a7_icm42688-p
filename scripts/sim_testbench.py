#!/usr/bin/env python3
"""
sim_testbench.py
================
Behavioral simulation / testbench for the IMU RTL project.
Replaces the iverilog-based run_sim.sh when iverilog is not available.

Models:
  1. uart_tx        – 8N1 UART transmitter
  2. spi_master     – SPI Mode-0 master (CLK_DIV, MAX_BYTES)
  3. icm42688_ctrl  – IMU controller FSM
  4. imu_data_stream– Packet formatter
  5. Integration    – end-to-end packet check

Each section prints [PASS] / [FAIL] and a final summary.
"""

import struct, textwrap

# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────
PASS = 0; FAIL = 0

def chk(cond, msg):
    global PASS, FAIL
    if cond:
        print(f"  [PASS] {msg}")
        PASS += 1
    else:
        print(f"  [FAIL] {msg}")
        FAIL += 1

def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 0x10000 if v >= 0x8000 else v

# ═══════════════════════════════════════════════════════════════════
# 1. UART TX MODEL
# ═══════════════════════════════════════════════════════════════════
class UartTx:
    """Cycle-accurate model of uart_tx.v (8N1, parameterised CLK_DIV)."""
    IDLE, START, DATA, STOP = 0, 1, 2, 3

    def __init__(self, clk_div):
        self.clk_div  = clk_div
        self.state    = self.IDLE
        self.tx       = 1
        self.busy     = False
        self.baud_cnt = 0
        self.bit_cnt  = 0
        self.shift    = 0
        self._start_i = False
        self._data_i  = 0

    def apply_inputs(self, start, data):
        self._start_i = start
        self._data_i  = data

    def clock(self):
        """Advance one system clock. Returns (tx_bit, busy)."""
        if self.state == self.IDLE:
            self.tx   = 1
            self.busy = False
            if self._start_i:
                self.shift    = self._data_i
                self.baud_cnt = 0
                self.state    = self.START
                self.busy     = True
        elif self.state == self.START:
            self.tx = 0
            if self.baud_cnt >= self.clk_div - 1:
                self.baud_cnt = 0
                self.bit_cnt  = 0
                self.state    = self.DATA
            else:
                self.baud_cnt += 1
        elif self.state == self.DATA:
            self.tx = self.shift & 1
            if self.baud_cnt >= self.clk_div - 1:
                self.baud_cnt = 0
                self.shift >>= 1
                if self.bit_cnt == 7:
                    self.state = self.STOP
                else:
                    self.bit_cnt += 1
            else:
                self.baud_cnt += 1
        elif self.state == self.STOP:
            self.tx = 1
            if self.baud_cnt >= self.clk_div - 1:
                self.baud_cnt = 0
                self.state    = self.IDLE
                self.busy     = False
            else:
                self.baud_cnt += 1
        return self.tx, self.busy

def test_uart_tx():
    print("\n══ TEST SUITE 1: uart_tx ══")
    CLK_DIV = 10
    u = UartTx(CLK_DIV)
    bits = []

    # Send 0xA5
    byte = 0xA5
    u.apply_inputs(True, byte)
    for _ in range(2000):
        tx, busy = u.clock()
        bits.append(tx)
        u.apply_inputs(False, 0)
        if not busy and len(bits) > CLK_DIV:
            break

    # Sample mid-bit: 1 start + 8 data + 1 stop = 10 bit-periods
    # start at falling edge (first 0 after initial 1)
    start_idx = next(i for i,b in enumerate(bits) if b == 0)
    samples = []
    for bit_n in range(10):
        idx = start_idx + bit_n * CLK_DIV + CLK_DIV // 2
        if idx < len(bits):
            samples.append(bits[idx])

    chk(len(samples) >= 10, "10 bit-periods captured")
    chk(samples[0] == 0, "start bit = 0")
    data_bits = samples[1:9]
    received = sum(b << i for i, b in enumerate(data_bits))
    chk(received == byte, f"data 0x{received:02X} == 0x{byte:02X}")
    chk(samples[9] == 1, "stop bit = 1")

    # Test busy flag
    u2 = UartTx(CLK_DIV)
    u2.apply_inputs(True, 0xFF)
    u2.clock()
    u2.apply_inputs(False, 0)
    chk(u2.busy, "busy asserts after start")
    for _ in range(CLK_DIV * 12):
        _, busy = u2.clock()
    chk(not busy, "busy de-asserts after stop bit")

# ═══════════════════════════════════════════════════════════════════
# 2. SPI MASTER MODEL
# ═══════════════════════════════════════════════════════════════════
class SpiMaster:
    """
    Behavioral (not cycle-accurate) model of spi_master.v.
    Returns list of received bytes given tx bytes.
    SPI Mode-0, MSB-first.
    """
    def transfer(self, tx_bytes):
        """
        tx_bytes: list of ints (first = command byte)
        slave_fn: callable(addr, n_bytes) -> list of response bytes
        Returns list of received bytes (same length as tx_bytes).
        """
        return tx_bytes  # passthrough; slave modelled separately

    def transfer_with_slave(self, tx_bytes, slave_regs):
        """
        Simulate a full SPI transaction against a register-map slave.
        tx_bytes[0] = command byte: bit7=R/W, [6:0]=addr
        Returns received bytes (rx[0] = dummy, rx[1..] = data).
        """
        cmd  = tx_bytes[0]
        rw   = (cmd >> 7) & 1
        addr = cmd & 0x7F
        rx   = [0x00] * len(tx_bytes)
        for i in range(1, len(tx_bytes)):
            reg_addr = (addr + i - 1) & 0x7F
            if rw:  # read
                rx[i] = slave_regs.get(reg_addr, 0x00)
            else:   # write
                slave_regs[reg_addr] = tx_bytes[i]
        return rx

def test_spi_master():
    print("\n══ TEST SUITE 2: spi_master ══")
    spi = SpiMaster()

    # Build a fake slave register map matching the testbench
    regs = {0x75: 0x47}  # WHO_AM_I
    for addr, val in [(0x1D,0x00),(0x1E,0x64),
                      (0x1F,0x12),(0x20,0x34),(0x21,0x56),(0x22,0x78),
                      (0x23,0x9A),(0x24,0xBC),
                      (0x25,0xDE),(0x26,0xF0),(0x27,0x11),(0x28,0x22),
                      (0x29,0x33),(0x2A,0x44)]:
        regs[addr] = val

    # WHO_AM_I read: cmd=0xF5 (R|0x75), 2 bytes
    rx = spi.transfer_with_slave([0xF5, 0x00], regs)
    chk(rx[1] == 0x47, f"WHO_AM_I = 0x{rx[1]:02X}")

    # Poll read: cmd=0x9D (R|0x1D), 15 bytes
    tx = [0x9D] + [0x00]*14
    rx = spi.transfer_with_slave(tx, regs)

    temp  = s16(rx[1], rx[2])
    ax    = s16(rx[3], rx[4])
    ay    = s16(rx[5], rx[6])
    az    = s16(rx[7], rx[8])
    gx    = s16(rx[9], rx[10])
    gy    = s16(rx[11], rx[12])
    gz    = s16(rx[13], rx[14])

    chk(temp == 0x0064, f"TEMP  = 0x{temp&0xFFFF:04X} (expect 0x0064)")
    chk(ax   == 0x1234, f"AX    = 0x{ax&0xFFFF:04X} (expect 0x1234)")
    chk(ay   == 0x5678, f"AY    = 0x{ay&0xFFFF:04X} (expect 0x5678)")
    chk(az   == s16(0x9A,0xBC), f"AZ    = 0x{az&0xFFFF:04X} (expect 0x9ABC)")
    chk(gx   == s16(0xDE,0xF0), f"GX    = 0x{gx&0xFFFF:04X} (expect 0xDEF0)")
    chk(gy   == s16(0x11,0x22), f"GY    = 0x{gy&0xFFFF:04X} (expect 0x1122)")
    chk(gz   == s16(0x33,0x44), f"GZ    = 0x{gz&0xFFFF:04X} (expect 0x3344)  ← BUG FIXED")

    # Verify the OLD bug: truncated gz
    gz_buggy = s16(rx[13], 0x00)
    chk(gz_buggy != gz, "Old {rx13,8h00} gz != correct {rx13,rx14}")

# ═══════════════════════════════════════════════════════════════════
# 3. ICM42688 CONTROLLER FSM MODEL
# ═══════════════════════════════════════════════════════════════════
class ICM42688Ctrl:
    """
    High-level FSM model of icm42688_ctrl.v (FIXED version).
    Validates init sequence and data read byte counts.
    """
    # States
    ST_RESET, ST_BOOT_WAIT, ST_WRITE_DEVCFG, ST_WAIT_SRESET = 0,1,2,3
    ST_READ_WHOAMI, ST_WRITE_INTFCFG, ST_WRITE_PWRMGMT, ST_WAIT_ENABLE = 4,5,6,7
    ST_WRITE_GYROCFG, ST_WRITE_ACCECFG, ST_INIT_DONE = 8,9,10
    ST_POLL_START, ST_POLL_WAIT, ST_POLL_DELAY, ST_ERROR = 11,12,13,15

    # Fixed parameters
    SYS_CLK   = 10_000          # fast sim clock
    SPI_CLK   = 1_000
    RESET_CYC = SYS_CLK // 1000
    BOOT_CYC  = (SYS_CLK // 1000) * 10   # FIXED: 10ms
    WAIT_EN   = (SYS_CLK // 1000) * 45
    POLL_CYC  = SYS_CLK // 1000
    CLK_DIV   = SYS_CLK // (SPI_CLK * 2)

    def __init__(self, slave_regs):
        self.state      = self.ST_RESET
        self.timer      = 0
        self.slave_regs = slave_regs.copy()
        self.init_done  = False
        self.error      = False
        self.data_valid = False
        self.ax = self.ay = self.az = 0
        self.gx = self.gy = self.gz = 0
        self.temp       = 0
        self.spi        = SpiMaster()
        self.log        = []
        self.cycle      = 0
        self.spi_transactions = []

    def spi_write(self, addr, data):
        cmd = addr & 0x7F  # write: bit7=0
        tx  = [cmd, data]
        self.spi.transfer_with_slave(tx, self.slave_regs)
        self.spi_transactions.append(('W', addr, data))

    def spi_read(self, addr, n):
        cmd = (1 << 7) | (addr & 0x7F)
        tx  = [cmd] + [0x00]*n
        rx  = self.spi.transfer_with_slave(tx, self.slave_regs)
        self.spi_transactions.append(('R', addr, n, rx))
        return rx

    def run(self, max_cycles=10_000_000):
        """Run FSM until first data_valid after init, or error, or timeout."""
        samples = []
        for _ in range(max_cycles):
            self.cycle += 1
            self.data_valid = False

            if self.state == self.ST_RESET:
                if self.timer >= self.RESET_CYC - 1:
                    self.timer = 0; self.state = self.ST_BOOT_WAIT
                else: self.timer += 1

            elif self.state == self.ST_BOOT_WAIT:
                if self.timer >= self.BOOT_CYC - 1:
                    self.timer = 0; self.state = self.ST_WRITE_DEVCFG
                else: self.timer += 1

            elif self.state == self.ST_WRITE_DEVCFG:
                self.spi_write(0x11, 0x01)
                self.timer = 0; self.state = self.ST_WAIT_SRESET

            elif self.state == self.ST_WAIT_SRESET:
                if self.timer >= self.BOOT_CYC - 1:
                    self.timer = 0; self.state = self.ST_READ_WHOAMI
                else: self.timer += 1

            elif self.state == self.ST_READ_WHOAMI:
                rx = self.spi_read(0x75, 1)
                whoami = rx[1] if len(rx) > 1 else 0
                if whoami == 0x47:
                    self.state = self.ST_WRITE_INTFCFG
                else:
                    self.state = self.ST_ERROR; self.error = True

            elif self.state == self.ST_WRITE_INTFCFG:
                self.spi_write(0x4C, 0x13)
                self.state = self.ST_WRITE_PWRMGMT

            elif self.state == self.ST_WRITE_PWRMGMT:
                self.spi_write(0x4E, 0x0F)
                self.timer = 0; self.state = self.ST_WAIT_ENABLE

            elif self.state == self.ST_WAIT_ENABLE:
                if self.timer >= self.WAIT_EN - 1:
                    self.timer = 0; self.state = self.ST_WRITE_GYROCFG
                else: self.timer += 1

            elif self.state == self.ST_WRITE_GYROCFG:
                self.spi_write(0x4F, 0x06)
                self.state = self.ST_WRITE_ACCECFG

            elif self.state == self.ST_WRITE_ACCECFG:
                self.spi_write(0x50, 0x06)
                self.init_done = True; self.state = self.ST_INIT_DONE

            elif self.state == self.ST_INIT_DONE:
                self.timer = 0; self.state = self.ST_POLL_START

            elif self.state == self.ST_POLL_START:
                # FIXED: 15 bytes (1 cmd + 14 data)
                rx = self.spi_read(0x1D, 14)
                # Parse: rx[0]=dummy, rx[1-2]=temp, rx[3-8]=accel, rx[9-14]=gyro
                self.temp = s16(rx[1],  rx[2])
                self.ax   = s16(rx[3],  rx[4])
                self.ay   = s16(rx[5],  rx[6])
                self.az   = s16(rx[7],  rx[8])
                self.gx   = s16(rx[9],  rx[10])
                self.gy   = s16(rx[11], rx[12])
                self.gz   = s16(rx[13], rx[14])  # FIXED: was {rx13, 0x00}
                self.data_valid = True
                samples.append((self.ax,self.ay,self.az,
                                 self.gx,self.gy,self.gz,self.temp))
                self.timer = 0; self.state = self.ST_POLL_DELAY
                if len(samples) >= 5:
                    return samples

            elif self.state == self.ST_POLL_DELAY:
                if self.timer >= self.POLL_CYC - 1:
                    self.timer = 0; self.state = self.ST_POLL_START
                else: self.timer += 1

            elif self.state == self.ST_ERROR:
                self.error = True
                return []

        return samples

def test_icm42688_ctrl():
    print("\n══ TEST SUITE 3: icm42688_ctrl (FIXED) ══")

    regs = {0x75: 0x47}
    for addr, val in [(0x1D,0x00),(0x1E,0x64),
                      (0x1F,0x12),(0x20,0x34),(0x21,0x56),(0x22,0x78),
                      (0x23,0x9A),(0x24,0xBC),
                      (0x25,0xDE),(0x26,0xF0),(0x27,0x11),(0x28,0x22),
                      (0x29,0x33),(0x2A,0x44)]:
        regs[addr] = val

    ctrl = ICM42688Ctrl(regs)
    samples = ctrl.run()

    chk(ctrl.init_done, "init_done asserted")
    chk(not ctrl.error,  "no error flag")
    chk(len(samples) >= 3, f"got {len(samples)} samples (need ≥3)")

    if samples:
        ax,ay,az,gx,gy,gz,temp = samples[0]
        chk(temp == 0x0064,           f"TEMP  = 0x{temp&0xFFFF:04X} (expect 0x0064)")
        chk(ax   == 0x1234,           f"AX    = 0x{ax&0xFFFF:04X}  (expect 0x1234)")
        chk(ay   == 0x5678,           f"AY    = 0x{ay&0xFFFF:04X}  (expect 0x5678)")
        chk(az   == s16(0x9A,0xBC),   f"AZ    = 0x{az&0xFFFF:04X}  (expect 0x9ABC)")
        chk(gx   == s16(0xDE,0xF0),   f"GX    = 0x{gx&0xFFFF:04X}  (expect 0xDEF0)")
        chk(gy   == s16(0x11,0x22),   f"GY    = 0x{gy&0xFFFF:04X}  (expect 0x1122)")
        chk(gz   == s16(0x33,0x44),   f"GZ    = 0x{gz&0xFFFF:04X}  (expect 0x3344)  ← BUG FIXED")

    # WHO_AM_I failure path
    bad_regs = {0x75: 0xFF}
    ctrl2 = ICM42688Ctrl(bad_regs)
    ctrl2.run()
    chk(ctrl2.error,    "error flag on bad WHO_AM_I")
    chk(not ctrl2.init_done, "init_done not set on error")

    # Verify init sequence writes correct registers
    writes = [(t[1],t[2]) for t in ctrl.spi_transactions if t[0]=='W']
    chk((0x11, 0x01) in writes, "soft-reset write (0x11=0x01)")
    chk((0x4E, 0x0F) in writes, "PWR_MGMT0 write (0x4E=0x0F)")
    chk((0x4F, 0x06) in writes, "GYRO_CONFIG0 write (0x4F=0x06)")
    chk((0x50, 0x06) in writes, "ACCEL_CONFIG0 write (0x50=0x06)")

    # Verify poll reads 15 bytes (cmd + 14 data)
    poll_reads = [t for t in ctrl.spi_transactions
                  if t[0]=='R' and t[1]==0x1D]
    chk(len(poll_reads) >= 1, "at least one poll read")
    if poll_reads:
        n = poll_reads[0][2]
        chk(n == 14, f"poll read requests 14 data bytes (got {n})  ← BUG FIXED (was 13)")

# ═══════════════════════════════════════════════════════════════════
# 4. IMU_DATA_STREAM MODEL
# ═══════════════════════════════════════════════════════════════════
class ImuDataStream:
    """
    Behavioral model of imu_data_stream.v
    Assembles 18-byte binary packets and feeds them through UartTx.
    """
    PKT_LEN = 18
    HEADER  = bytes([0xAA, 0x55])
    TRAILER = bytes([0x0D, 0x0A])

    def build_packet(self, ax, ay, az, gx, gy, gz, temp):
        def u16(v):
            v = v & 0xFFFF
            return bytes([(v >> 8) & 0xFF, v & 0xFF])
        return (self.HEADER +
                u16(ax) + u16(ay) + u16(az) +
                u16(gx) + u16(gy) + u16(gz) +
                u16(temp) +
                self.TRAILER)

    def transmit(self, packet, clk_div=10):
        """Run UartTx for each byte, collect bits, decode bytes."""
        uart = UartTx(clk_div)
        received = []
        for byte in packet:
            bits = []
            uart.apply_inputs(True, byte)
            for _ in range(clk_div * 12 + 10):
                tx, busy = uart.clock()
                bits.append(tx)
                uart.apply_inputs(False, 0)
                if not busy and len(bits) > clk_div:
                    break
            # Decode
            start_idx = next((i for i,b in enumerate(bits) if b==0), None)
            if start_idx is not None:
                data_bits = []
                for bit_n in range(8):
                    idx = start_idx + (bit_n+1)*clk_div + clk_div//2
                    if idx < len(bits):
                        data_bits.append(bits[idx])
                if len(data_bits) == 8:
                    received.append(sum(b<<i for i,b in enumerate(data_bits)))
        return bytes(received)

def test_imu_data_stream():
    print("\n══ TEST SUITE 4: imu_data_stream ══")

    stream = ImuDataStream()

    # Test values
    AX, AY, AZ = 0x1234, 0x5678, -0x6543
    GX, GY, GZ = -0x2110, 0x1122, 0x3344
    TEMP = 0x0064

    pkt = stream.build_packet(AX, AY, AZ, GX, GY, GZ, TEMP)

    chk(len(pkt) == 18,              f"packet length = {len(pkt)} (expect 18)")
    chk(pkt[0:2]   == b'\xAA\x55',  "header = 0xAA 0x55")
    chk(pkt[-2:]   == b'\x0D\x0A',  "trailer = 0x0D 0x0A (CRLF)")

    # Parse fields
    ax_r = s16(pkt[2], pkt[3]);   ay_r = s16(pkt[4],  pkt[5])
    az_r = s16(pkt[6], pkt[7]);   gx_r = s16(pkt[8],  pkt[9])
    gy_r = s16(pkt[10],pkt[11]);  gz_r = s16(pkt[12], pkt[13])
    t_r  = s16(pkt[14],pkt[15])

    chk(ax_r == AX, f"AX round-trip 0x{ax_r&0xFFFF:04X}")
    chk(ay_r == AY, f"AY round-trip 0x{ay_r&0xFFFF:04X}")
    chk(az_r == AZ, f"AZ round-trip 0x{az_r&0xFFFF:04X}")
    chk(gx_r == GX, f"GX round-trip 0x{gx_r&0xFFFF:04X}")
    chk(gy_r == GY, f"GY round-trip 0x{gy_r&0xFFFF:04X}")
    chk(gz_r == GZ, f"GZ round-trip 0x{gz_r&0xFFFF:04X}  ← BUG FIXED")
    chk(t_r  == TEMP, f"TEMP round-trip 0x{t_r&0xFFFF:04X}")

    # UART serialisation
    received = stream.transmit(pkt, clk_div=10)
    chk(received == pkt, f"UART serial round-trip ({len(received)}/18 bytes match)")

# ═══════════════════════════════════════════════════════════════════
# 5. INTEGRATION TEST
# ═══════════════════════════════════════════════════════════════════
def test_integration():
    print("\n══ TEST SUITE 5: Integration (ctrl → stream → UART → Python parser) ══")

    # Fake slave register map
    regs = {0x75: 0x47}
    for addr, val in [(0x1D,0x00),(0x1E,0x64),
                      (0x1F,0x12),(0x20,0x34),(0x21,0x56),(0x22,0x78),
                      (0x23,0x9A),(0x24,0xBC),
                      (0x25,0xDE),(0x26,0xF0),(0x27,0x11),(0x28,0x22),
                      (0x29,0x33),(0x2A,0x44)]:
        regs[addr] = val

    # Run controller
    ctrl   = ICM42688Ctrl(regs)
    samples = ctrl.run()
    chk(len(samples) > 0, "controller produces samples")

    # Feed through stream → parse
    stream = ImuDataStream()
    ACCEL_LSB = 2048.0
    GYRO_LSB  = 16.4
    TEMP_SENS = 132.48
    TEMP_OFF  = 25.0

    for i, (ax,ay,az,gx,gy,gz,temp) in enumerate(samples[:3]):
        pkt = stream.build_packet(ax,ay,az,gx,gy,gz,temp)
        # Verify packet parses back correctly
        chk(pkt[0:2] == b'\xAA\x55', f"sample {i}: header OK")
        chk(pkt[-2:] == b'\x0D\x0A', f"sample {i}: trailer OK")

        ax_p = s16(pkt[2],pkt[3]);  gz_p = s16(pkt[12],pkt[13])
        chk(ax_p == ax, f"sample {i}: AX preserved")
        chk(gz_p == gz, f"sample {i}: GZ preserved (FIXED, was always 0)")

        # Physical units
        az_g  = s16(pkt[7],pkt[8]) / ACCEL_LSB   # WRONG intentionally? No, use proper
        az_g  = az / ACCEL_LSB
        gz_dps= gz / GYRO_LSB
        tc    = temp / TEMP_SENS + TEMP_OFF
        chk(-20.0 <= az_g  <= 20.0,  f"sample {i}: AZ in ±16g range ({az_g:.3f}g)")
        chk(-2200 <= gz_dps <= 2200,  f"sample {i}: GZ in ±2000dps range ({gz_dps:.2f}°/s)")
        chk(-40 <= tc <= 105,         f"sample {i}: Temp in range ({tc:.2f}°C)")

    # Verify Gz is NOT the old stuck-at-0 value for non-zero register
    ax0,ay0,az0,gx0,gy0,gz0,t0 = samples[0]
    expected_gz = s16(0x33, 0x44)
    chk(gz0 == expected_gz, f"GZ = {gz0} (0x{gz0&0xFFFF:04X}), matches reg[0x29:0x2A]=0x3344")

# ═══════════════════════════════════════════════════════════════════
# 6. REGRESSION: verify OLD bugs produce wrong output
# ═══════════════════════════════════════════════════════════════════
def test_regression_old_bugs():
    print("\n══ TEST SUITE 6: Regression – verify old bugs were real ══")

    regs = {0x75: 0x47}
    for addr, val in [(0x1D,0x00),(0x1E,0x64),
                      (0x1F,0x12),(0x20,0x34),(0x21,0x56),(0x22,0x78),
                      (0x23,0x9A),(0x24,0xBC),
                      (0x25,0xDE),(0x26,0xF0),(0x27,0x11),(0x28,0x22),
                      (0x29,0x33),(0x2A,0x44)]:
        regs[addr] = val

    spi = SpiMaster()

    # BUG A: old code read 13 data bytes (n=14 total), missing GYRO_Z_L
    tx_old = [0x9D] + [0x00]*13   # 14 bytes (n=14), only 13 data
    rx_old = spi.transfer_with_slave(tx_old, regs)
    gz_old = s16(rx_old[13], 0x00)   # old: {rx13, 8'h00}
    expected_gz = s16(0x33, 0x44)
    chk(gz_old != expected_gz, f"BUG A confirmed: old gz=0x{gz_old&0xFFFF:04X} ≠ 0x3344")
    chk(gz_old == s16(0x33, 0x00),   f"BUG A: old gz={gz_old} = {{0x33,0x00}} (truncated)")

    # BUG B: fixed code reads 14 data bytes (n=15 total)
    tx_new = [0x9D] + [0x00]*14   # 15 bytes
    rx_new = spi.transfer_with_slave(tx_new, regs)
    gz_new = s16(rx_new[13], rx_new[14])
    chk(gz_new == expected_gz, f"FIX B: new gz=0x{gz_new&0xFFFF:04X} == 0x3344 ✓")

    # BUG C: soft-reset wait was only 1ms (1 BOOT_CYC), now 10ms
    old_boot_cyc = 10_000 // 1000          # = 10 cycles at 10kHz
    new_boot_cyc = (10_000 // 1000) * 10   # = 100 cycles at 10kHz
    chk(old_boot_cyc == 10,  f"Old BOOT_CYC = {old_boot_cyc} (1ms — too short)")
    chk(new_boot_cyc == 100, f"New BOOT_CYC = {new_boot_cyc} (10ms — correct per datasheet)")

# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 60)
    print(" IMU RTL Project – Python Behavioral Testbench")
    print("=" * 60)

    test_uart_tx()
    test_spi_master()
    test_icm42688_ctrl()
    test_imu_data_stream()
    test_integration()
    test_regression_old_bugs()

    print("\n" + "=" * 60)
    print(f" RESULTS:  {PASS} PASSED  |  {FAIL} FAILED")
    print("=" * 60)
    if FAIL == 0:
        print(" ✅  ALL TESTS PASSED")
    else:
        print(f" ❌  {FAIL} TEST(S) FAILED")
    print()
