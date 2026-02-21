// =============================================================================
// tb_top_extended.v
// Extended full integration testbench for top.v
//
// Specifically targets the FPGA symptom: "constant value for all readings,
// LEDs don't light up"
//
// Key failure modes investigated:
//  A) Wrong SPI write data (state transition before req_next fires)
//  B) First poll returning zero (data not ready when first sample fires)
//  C) LED never toggling (transmission too short to see by eye)
//  D) Incorrect data byte mapping (byte-swap)
//  E) WHO_AM_I mismatch (wrong ID expected or wrong address)
//  F) Clock domain issues (wrong polarity resets, etc.)
//
// Tests:
//   1.  Init sequence visible on LED[0] (init_done / led_init)
//   2.  No init with reset held
//   3.  LED[2] (error) stays low with correct WHO_AM_I
//   4.  LED[2] goes high with wrong WHO_AM_I
//   5.  UART packets received correctly with known IMU data
//   6.  accel_x in correct packet position
//   7.  All 7 channels correct in packet
//   8.  Multiple packets received
//   9.  LED[1] (data) lights during transmission
//  10.  LED[1] goes off after packet complete
//  11.  Packet header = 0xAA 0x55
//  12.  Packet footer = 0x0D 0x0A
//  13.  DEVICE_CONFIG soft-reset sent correctly (addr=0x11, val=0x01)
//  14.  PWR_MGMT0 written with 0x0F (accel+gyro LN mode)
//  15.  GYRO_CONFIG0 written with 0x06 (2000dps, 1kHz)
//  16.  ACCEL_CONFIG0 written with 0x06 (16g, 1kHz)
//  17.  Poll reads 15 bytes starting from 0x1D (TEMP_DATA1)
//  18.  Negative values propagate correctly
//  19.  Reset clears outputs and restarts correctly
//  20.  System re-inits after reset without intervention
// =============================================================================
`timescale 1ns/1ps
`include "icm42688_regs.vh"

module tb_top_extended;

// -----------------------------------------------------------------------
// DUT
// -----------------------------------------------------------------------
// Scaled-down clocks for fast simulation
localparam SIM_SYS_CLK  = 10_000;   // 10kHz sys clock
localparam SIM_UART_BAUD = 1_000;    // 1kbaud UART

// Instantiate top with scaled parameters via its submodules
// Since top.v has hardcoded params, we need to instantiate submodules directly
// for fast simulation. We'll use the ICM ctrl and data stream directly.

reg  clk, rst_n;
wire signed [15:0] accel_x, accel_y, accel_z;
wire signed [15:0] gyro_x, gyro_y, gyro_z;
wire signed [15:0] temp_data;
wire data_valid, init_done, imu_error;
wire spi_sck, spi_mosi, spi_cs_n;
reg  spi_miso;

wire uart_tx;
wire led_init, led_data, led_error;

icm42688_ctrl #(
    .SYS_CLK_HZ (SIM_SYS_CLK),
    .SPI_CLK_HZ (SIM_SYS_CLK / 10),
    .CLK_DIV    (5)
) u_imu (
    .clk_i       (clk),
    .rst_ni      (rst_n),
    .accel_x_o   (accel_x),
    .accel_y_o   (accel_y),
    .accel_z_o   (accel_z),
    .gyro_x_o    (gyro_x),
    .gyro_y_o    (gyro_y),
    .gyro_z_o    (gyro_z),
    .temp_o      (temp_data),
    .data_valid_o(data_valid),
    .init_done_o (init_done),
    .error_o     (imu_error),
    .spi_sck_o   (spi_sck),
    .spi_mosi_o  (spi_mosi),
    .spi_miso_i  (spi_miso),
    .spi_cs_n_o  (spi_cs_n)
);

imu_data_stream #(
    .CLK_HZ    (SIM_SYS_CLK),
    .BAUD_RATE (SIM_UART_BAUD)
) u_stream (
    .clk_i       (clk),
    .rst_ni      (rst_n),
    .accel_x_i   (accel_x),
    .accel_y_i   (accel_y),
    .accel_z_i   (accel_z),
    .gyro_x_i    (gyro_x),
    .gyro_y_i    (gyro_y),
    .gyro_z_i    (gyro_z),
    .temp_i      (temp_data),
    .data_valid_i(data_valid),
    .init_done_i (init_done),
    .error_i     (imu_error),
    .uart_tx_o   (uart_tx),
    .led_init_o  (led_init),
    .led_data_o  (led_data),
    .led_error_o (led_error)
);

// 10kHz clock: 50µs period
initial clk = 0;
always #50000 clk = ~clk;

// -----------------------------------------------------------------------
// Pass/fail
// -----------------------------------------------------------------------
integer pass_cnt = 0;
integer fail_cnt = 0;

task chk;
    input condition;
    input [511:0] msg;
    begin
        if (condition) begin
            $display("[PASS] %s", msg);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] %s  @time=%0t", msg, $time);
            fail_cnt = fail_cnt + 1;
        end
    end
endtask

// -----------------------------------------------------------------------
// Accurate SPI slave model
// -----------------------------------------------------------------------
reg [7:0] slave_regs [0:127];
reg [7:0] w_addr_log [0:31];
reg [7:0] w_data_log [0:31];
integer   w_log_cnt;
reg [7:0] last_read_addr;
integer   read_count;
reg       whoami_bad;

task spi_slave_run;
    reg [7:0] cmd;
    reg       rw;
    reg [6:0] addr;
    reg [7:0] tx_byte;
    reg [7:0] rx_byte;
    integer   bn, bi;
    begin
        cmd = 8'h00;
        spi_miso = 1'b0;
        for (bi = 7; bi >= 0; bi = bi - 1) begin
            @(posedge spi_sck or posedge spi_cs_n);
            if (spi_cs_n) disable spi_slave_run;
            cmd[bi] = spi_mosi;
            if (bi > 0) begin
                @(negedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) disable spi_slave_run;
            end
        end
        rw   = cmd[7];
        addr = cmd[6:0];
        bn   = 0;
        while (!spi_cs_n) begin
            if (rw) begin
                if (whoami_bad && (addr + bn) == 7'h75)
                    tx_byte = 8'hFF;
                else
                    tx_byte = slave_regs[(addr + bn) & 7'h7F];
            end else tx_byte = 8'h00;

            rx_byte = 8'h00;
            for (bi = 7; bi >= 0; bi = bi - 1) begin
                @(negedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) begin spi_miso = 0; disable spi_slave_run; end
                spi_miso = tx_byte[bi];
                @(posedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) disable spi_slave_run;
                rx_byte[bi] = spi_mosi;
            end

            if (!rw) begin
                slave_regs[addr & 7'h7F] = rx_byte;
                w_addr_log[w_log_cnt & 5'h1F] = addr;
                w_data_log[w_log_cnt & 5'h1F] = rx_byte;
                w_log_cnt = w_log_cnt + 1;
            end
            if (rw && bn == 0) begin
                last_read_addr = addr;
                read_count = read_count + 1;
            end
            bn = bn + 1;
        end
        spi_miso = 0;
    end
endtask

initial begin : spi_slave
    integer ki;
    spi_miso      = 0;
    w_log_cnt     = 0;
    read_count    = 0;
    last_read_addr= 8'hFF;
    whoami_bad    = 0;
    for (ki = 0; ki < 128; ki = ki + 1) slave_regs[ki] = 8'h00;
    slave_regs[8'h75] = 8'h47;
    // Default IMU data
    slave_regs[8'h1D] = 8'h00; slave_regs[8'h1E] = 8'h64; // temp = 100
    slave_regs[8'h1F] = 8'h12; slave_regs[8'h20] = 8'h34; // accel_x
    slave_regs[8'h21] = 8'h56; slave_regs[8'h22] = 8'h78; // accel_y
    slave_regs[8'h23] = 8'h9A; slave_regs[8'h24] = 8'hBC; // accel_z
    slave_regs[8'h25] = 8'hDE; slave_regs[8'h26] = 8'hF0; // gyro_x
    slave_regs[8'h27] = 8'h11; slave_regs[8'h28] = 8'h22; // gyro_y
    slave_regs[8'h29] = 8'h33; slave_regs[8'h2A] = 8'h44; // gyro_z
    forever begin
        @(negedge spi_cs_n);
        spi_slave_run;
    end
end

// -----------------------------------------------------------------------
// UART receiver
// -----------------------------------------------------------------------
localparam UART_CLK_DIV = SIM_SYS_CLK / SIM_UART_BAUD;  // 10

reg [7:0] uart_buf [0:511];
integer   uart_cnt;

initial begin : uart_receiver
    integer bi;
    reg [7:0] b;
    uart_cnt = 0;
    forever begin
        @(negedge uart_tx);
        // Start bit detected - wait half baud + 1 baud for bit 0 center
        #(UART_CLK_DIV * 100000 / 2); // half baud in ns (100000ns = 1 clock)
        b = 8'h00;
        for (bi = 0; bi < 8; bi = bi + 1) begin
            #(UART_CLK_DIV * 100000); // 1 baud in ns
            b[bi] = uart_tx;
        end
        uart_buf[uart_cnt & 9'h1FF] = b;
        uart_cnt = uart_cnt + 1;
        #(UART_CLK_DIV * 100000); // stop bit
    end
end

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------
integer cnt;
reg got_it;

task wait_init;
    input [31:0] max_cycles;
    begin
        cnt = 0;
        while (!init_done && cnt < max_cycles) begin @(posedge clk); cnt++; end
        got_it = init_done;
    end
endtask

task wait_data_valid;
    input [31:0] max_cycles;
    begin
        cnt = 0;
        while (!data_valid && cnt < max_cycles) begin @(posedge clk); cnt++; end
        got_it = data_valid;
    end
endtask

task wait_uart_bytes;
    input integer num_bytes;
    input integer start_cnt;
    input [31:0] max_cycles;
    begin
        cnt = 0;
        while ((uart_cnt - start_cnt) < num_bytes && cnt < max_cycles) begin
            @(posedge clk); cnt++;
        end
        got_it = ((uart_cnt - start_cnt) >= num_bytes);
    end
endtask

task reset_dut;
    begin
        rst_n = 0;
        w_log_cnt     = 0;
        read_count    = 0;
        last_read_addr= 8'hFF;
        repeat(20) @(posedge clk);
        rst_n = 1;
    end
endtask

// -----------------------------------------------------------------------
// Test execution
// -----------------------------------------------------------------------
integer i, j;
integer pkt_base;

initial begin
`ifdef VCD
    $dumpfile("tb_top_extended.vcd");
    $dumpvars(0, tb_top_extended);
`endif

    rst_n = 0;
    repeat(20) @(posedge clk);
    rst_n = 1;

    // ==================================================================
    // TEST 1: Init visible on LED[0]
    // ==================================================================
    $display("\n--- TEST 1: LED init (led_init) asserts after boot ---");
    chk(led_init == 0, "T1a: led_init LOW before init");
    wait_init(300_000);
    chk(got_it,        "T1b: init_done asserted");
    @(posedge clk);  // LED is registered — 1-clock delay
    chk(led_init == 1, "T1c: led_init HIGH after init");

    // ==================================================================
    // TEST 2: No init with reset held
    // ==================================================================
    $display("\n--- TEST 2: No init while reset held ---");
    rst_n = 0;
    repeat(5) @(posedge clk);
    chk(!init_done, "T2a: init_done LOW during reset");
    chk(!led_init,  "T2b: led_init LOW during reset");
    rst_n = 1;

    // ==================================================================
    // TEST 3: LED error stays LOW with correct WHO_AM_I
    // ==================================================================
    $display("\n--- TEST 3: LED error stays LOW (correct WHO_AM_I) ---");
    wait_init(300_000);
    wait_data_valid(100_000);
    chk(!imu_error, "T3a: imu_error stays LOW with WHO_AM_I=0x47");
    chk(!led_error, "T3b: led_error stays LOW with correct WHO_AM_I");

    // ==================================================================
    // TEST 4: LED error goes HIGH with wrong WHO_AM_I
    // ==================================================================
    $display("\n--- TEST 4: LED error goes HIGH with wrong WHO_AM_I ---");
    whoami_bad = 1;
    reset_dut;
    cnt = 0;
    while (!imu_error && cnt < 300_000) begin @(posedge clk); cnt++; end
    chk(imu_error,  "T4a: imu_error HIGH with wrong WHO_AM_I");
    @(posedge clk);  // LED is registered — 1-clock delay
    chk(led_error == 1, "T4b: led_error HIGH with wrong WHO_AM_I");
    whoami_bad = 0;

    // ==================================================================
    // TEST 5-7: UART packets with known data
    // ==================================================================
    $display("\n--- TEST 5-7: UART data correctness ---");
    // Reset with correct WHO_AM_I
    reset_dut;
    wait_init(300_000);
    // Wait for first packet
    pkt_base = uart_cnt;
    wait_uart_bytes(18, pkt_base, 1_000_000);
    chk(got_it, "T5: Got first UART packet after init");

    // ==================================================================
    // TEST 6: Verify packet header
    // ==================================================================
    $display("\n--- TEST 6: Packet header bytes ---");
    $display("  Byte[0]=0x%02X (expect 0xAA)", uart_buf[pkt_base & 9'h1FF]);
    $display("  Byte[1]=0x%02X (expect 0x55)", uart_buf[(pkt_base+1) & 9'h1FF]);
    chk(uart_buf[pkt_base & 9'h1FF]       == 8'hAA, "T6a: Packet header[0] = 0xAA");
    chk(uart_buf[(pkt_base+1) & 9'h1FF]   == 8'h55, "T6b: Packet header[1] = 0x55");

    // ==================================================================
    // TEST 7: accel_x in correct position
    // ==================================================================
    $display("\n--- TEST 7: accel_x in packet (expect 0x1234) ---");
    $display("  accel_x = 0x%02X%02X (expect 0x1234)",
             uart_buf[(pkt_base+2) & 9'h1FF],
             uart_buf[(pkt_base+3) & 9'h1FF]);
    chk(uart_buf[(pkt_base+2) & 9'h1FF] == 8'h12, "T7a: accel_x_H = 0x12");
    chk(uart_buf[(pkt_base+3) & 9'h1FF] == 8'h34, "T7b: accel_x_L = 0x34");

    // ==================================================================
    // TEST 8: All channels in correct positions
    // ==================================================================
    $display("\n--- TEST 8: All channels correct in packet ---");
    begin : test8_block
        integer b;
        b = pkt_base;
        $display("  accel_y = 0x%02X%02X (expect 0x5678)",
                 uart_buf[(b+4)&9'h1FF], uart_buf[(b+5)&9'h1FF]);
        $display("  accel_z = 0x%02X%02X (expect 0x9ABC)",
                 uart_buf[(b+6)&9'h1FF], uart_buf[(b+7)&9'h1FF]);
        $display("  gyro_x  = 0x%02X%02X (expect 0xDEF0)",
                 uart_buf[(b+8)&9'h1FF], uart_buf[(b+9)&9'h1FF]);
        $display("  gyro_y  = 0x%02X%02X (expect 0x1122)",
                 uart_buf[(b+10)&9'h1FF], uart_buf[(b+11)&9'h1FF]);
        $display("  gyro_z  = 0x%02X%02X (expect 0x3344)",
                 uart_buf[(b+12)&9'h1FF], uart_buf[(b+13)&9'h1FF]);
        $display("  temp    = 0x%02X%02X (expect 0x0064)",
                 uart_buf[(b+14)&9'h1FF], uart_buf[(b+15)&9'h1FF]);
        chk(uart_buf[(b+4)&9'h1FF] == 8'h56, "T8a: accel_y_H = 0x56");
        chk(uart_buf[(b+5)&9'h1FF] == 8'h78, "T8b: accel_y_L = 0x78");
        chk(uart_buf[(b+6)&9'h1FF] == 8'h9A, "T8c: accel_z_H = 0x9A");
        chk(uart_buf[(b+7)&9'h1FF] == 8'hBC, "T8d: accel_z_L = 0xBC");
        chk(uart_buf[(b+8)&9'h1FF] == 8'hDE, "T8e: gyro_x_H  = 0xDE");
        chk(uart_buf[(b+9)&9'h1FF] == 8'hF0, "T8f: gyro_x_L  = 0xF0");
        chk(uart_buf[(b+10)&9'h1FF] == 8'h11,"T8g: gyro_y_H  = 0x11");
        chk(uart_buf[(b+11)&9'h1FF] == 8'h22,"T8h: gyro_y_L  = 0x22");
        chk(uart_buf[(b+12)&9'h1FF] == 8'h33,"T8i: gyro_z_H  = 0x33");
        chk(uart_buf[(b+13)&9'h1FF] == 8'h44,"T8j: gyro_z_L  = 0x44");
        chk(uart_buf[(b+14)&9'h1FF] == 8'h00,"T8k: temp_H     = 0x00");
        chk(uart_buf[(b+15)&9'h1FF] == 8'h64,"T8l: temp_L     = 0x64");
    end

    // ==================================================================
    // TEST 9: CRLF footer
    // ==================================================================
    $display("\n--- TEST 9: CRLF footer ---");
    chk(uart_buf[(pkt_base+16)&9'h1FF] == 8'h0D, "T9a: Footer CR = 0x0D");
    chk(uart_buf[(pkt_base+17)&9'h1FF] == 8'h0A, "T9b: Footer LF = 0x0A");

    // ==================================================================
    // TEST 10: Multiple packets received (at least 3)
    // ==================================================================
    $display("\n--- TEST 10: Multiple packets received ---");
    pkt_base = uart_cnt;
    wait_uart_bytes(54, pkt_base, 3_000_000);  // 3 more packets = 54 bytes
    $display("  Received %0d bytes after waiting (expect >=54)", uart_cnt - pkt_base);
    chk(got_it, "T10: At least 3 more complete packets received");

    // ==================================================================
    // TEST 11: led_data lights during transmission
    // ==================================================================
    $display("\n--- TEST 11: led_data active during TX ---");
    wait_data_valid(100_000);
    cnt = 0;
    while (!led_data && cnt < 100_000) begin @(posedge clk); cnt++; end
    chk(cnt < 100_000, "T11: led_data goes HIGH during transmission");

    // ==================================================================
    // TEST 12: led_data off after packet
    // ==================================================================
    $display("\n--- TEST 12: led_data off after packet ---");
    cnt = 0;
    while (led_data && cnt < 1_000_000) begin @(posedge clk); cnt++; end
    chk(!led_data, "T12: led_data goes LOW after packet ends");

    // ==================================================================
    // TEST 13-16: Verify SPI init writes
    // ==================================================================
    $display("\n--- TEST 13-16: SPI init write verification ---");
    // Reset to capture fresh writes
    reset_dut;
    wait_init(300_000);

    $display("  Captured %0d writes:", w_log_cnt);
    for (i = 0; i < w_log_cnt && i < 16; i = i + 1)
        $display("    [%0d] addr=0x%02X data=0x%02X", i, w_addr_log[i], w_data_log[i]);

    begin : check_writes_block
        integer found_devcfg, found_pwr, found_gyro, found_accel, found_intfcfg;
        integer ci;
        found_devcfg  = 0; found_pwr    = 0;
        found_gyro    = 0; found_accel  = 0; found_intfcfg = 0;
        for (ci = 0; ci < w_log_cnt; ci = ci + 1) begin
            if (w_addr_log[ci] == 8'h11 && w_data_log[ci] == 8'h01) found_devcfg = 1;
            if (w_addr_log[ci] == 8'h4E && w_data_log[ci] == 8'h0F) found_pwr = 1;
            if (w_addr_log[ci] == 8'h4F && w_data_log[ci] == 8'h06) found_gyro = 1;
            if (w_addr_log[ci] == 8'h50 && w_data_log[ci] == 8'h06) found_accel = 1;
            if (w_addr_log[ci] == 8'h4C) found_intfcfg = 1;
        end
        chk(found_devcfg,  "T13: DEVICE_CONFIG write: 0x11 <- 0x01 (soft reset)");
        chk(found_pwr,     "T14: PWR_MGMT0 write: 0x4E <- 0x0F (accel+gyro LN)");
        chk(found_gyro,    "T15: GYRO_CONFIG0 write: 0x4F <- 0x06 (2000dps 1kHz)");
        chk(found_accel,   "T16: ACCEL_CONFIG0 write: 0x50 <- 0x06 (16g 1kHz)");
    end

    // ==================================================================
    // TEST 17: Poll reads from correct register (0x1D TEMP_DATA1)
    // ==================================================================
    $display("\n--- TEST 17: Poll reads from TEMP_DATA1 (0x1D) ---");
    // Wait for at least one poll read
    wait_data_valid(200_000);
    $display("  last_read_addr = 0x%02X (expect 0x1D)", last_read_addr);
    chk(last_read_addr == 8'h1D, "T17: Poll reads starting at TEMP_DATA1 (0x1D)");

    // ==================================================================
    // TEST 18: Negative values propagate correctly end-to-end
    // ==================================================================
    $display("\n--- TEST 18: Negative values end-to-end ---");
    // Update slave registers with negative test values
    slave_regs[8'h1F] = 8'hFF; slave_regs[8'h20] = 8'h00; // accel_x = -256
    slave_regs[8'h25] = 8'h80; slave_regs[8'h26] = 8'h00; // gyro_x = -32768
    // Wait for at least 3 data_valid pulses to flush any in-flight stale data
    wait_data_valid(200_000); @(posedge clk);
    wait_data_valid(200_000); @(posedge clk);
    wait_data_valid(200_000); @(posedge clk);
    pkt_base = uart_cnt;
    wait_uart_bytes(18, pkt_base, 1_000_000);
    $display("  accel_x = 0x%02X%02X (expect 0xFF00)",
             uart_buf[(pkt_base+2)&9'h1FF], uart_buf[(pkt_base+3)&9'h1FF]);
    chk(uart_buf[(pkt_base+2)&9'h1FF] == 8'hFF, "T18a: Negative accel_x_H = 0xFF");
    chk(uart_buf[(pkt_base+3)&9'h1FF] == 8'h00, "T18b: Negative accel_x_L = 0x00");

    // Restore
    slave_regs[8'h1F] = 8'h12; slave_regs[8'h20] = 8'h34;
    slave_regs[8'h25] = 8'hDE; slave_regs[8'h26] = 8'hF0;

    // ==================================================================
    // TEST 19: Reset clears outputs
    // ==================================================================
    $display("\n--- TEST 19: Reset clears outputs ---");
    wait_init(300_000);
    rst_n = 0;
    repeat(5) @(posedge clk);
    chk(!init_done,  "T19a: init_done cleared by reset");
    chk(!imu_error,  "T19b: imu_error cleared by reset");
    chk(!data_valid, "T19c: data_valid cleared by reset");
    rst_n = 1;

    // ==================================================================
    // TEST 20: System re-inits after reset
    // ==================================================================
    $display("\n--- TEST 20: System re-inits after reset ---");
    wait_init(300_000);
    chk(got_it, "T20a: init_done asserts after reset release");
    wait_data_valid(200_000);
    chk(got_it, "T20b: data_valid fires after re-init");
    pkt_base = uart_cnt;
    wait_uart_bytes(18, pkt_base, 1_000_000);
    chk(got_it, "T20c: UART packet received after re-init");

    // ==================================================================
    // Summary
    // ==================================================================
    $display("\n========================================");
    $display("TOP Integration Extended TB: PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
    $display("========================================");
    if (fail_cnt == 0) $display("ALL TESTS PASSED");
    else               $display("*** %0d TESTS FAILED ***", fail_cnt);
    $finish;
end

initial begin
    #50_000_000_000_000; $display("TIMEOUT"); $finish;
end

endmodule
