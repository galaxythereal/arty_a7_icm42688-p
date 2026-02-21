// tb_icm42688_ctrl.v
`timescale 1ns/1ps

module tb_icm42688_ctrl;

reg  clk, rst_n;
wire signed [15:0] accel_x, accel_y, accel_z;
wire signed [15:0] gyro_x, gyro_y, gyro_z;
wire signed [15:0] temp_data;
wire data_valid, init_done, imu_error;
wire spi_sck, spi_mosi, spi_cs_n;
reg  spi_miso;

icm42688_ctrl #(.SYS_CLK_HZ(10_000),.SPI_CLK_HZ(1_000),.CLK_DIV(5)) dut (
    .clk_i(clk),.rst_ni(rst_n),
    .accel_x_o(accel_x),.accel_y_o(accel_y),.accel_z_o(accel_z),
    .gyro_x_o(gyro_x),.gyro_y_o(gyro_y),.gyro_z_o(gyro_z),
    .temp_o(temp_data),.data_valid_o(data_valid),.init_done_o(init_done),.error_o(imu_error),
    .spi_sck_o(spi_sck),.spi_mosi_o(spi_mosi),.spi_miso_i(spi_miso),.spi_cs_n_o(spi_cs_n)
);
initial clk=0; always #50000 clk=~clk;

integer pass_cnt=0, fail_cnt=0;
task chk; input cond; input [255:0] msg; begin
    if (cond) begin $display("[PASS] %s",msg); pass_cnt=pass_cnt+1; end
    else       begin $display("[FAIL] %s @%0t",msg,$time); fail_cnt=fail_cnt+1; end
end endtask

// ========== SPI Slave ==========
// Mode 0: CPOL=0 CPHA=0
//   - Data changes on falling SCK
//   - Data sampled on rising SCK
//   - Master changes MOSI on falling, slave changes MISO on falling
//   - MISO must be valid before the FIRST rising edge after CS assert
//
// SPI master analysis:
//   ASSERT_CS state: CS goes low, SCK=0
//   TRANSFER: clk_cnt counts to CLK_DIV-1, then toggle SCK
//     First toggle: SCK 0→1 (rising) - master samples MISO
//     Second toggle: SCK 1→0 (falling) - master shifts out next MOSI
//
// So slave must set up MISO BEFORE first rising SCK edge.
// The slave reads MOSI on rising SCK edges.
// The slave sets MISO after falling SCK edges (for the next bit).
// For the FIRST bit of a read response, slave must set MISO immediately after CS low.

reg [7:0] regs[0:127];
reg whoami_bad;
integer ki;

// Exchange: sample MOSI on posedge, drive MISO on negedge
// For the first byte (cmd): slave drives MISO=0, just samples MOSI
// For read bytes: slave drives response MSB immediately, then shifts on negedge

task do_transaction;
    reg [7:0] cmd, rx, tx;
    reg       rw;
    reg [6:0] addr;
    integer   bn, j;
    reg [7:0] the_byte;
    begin
        // Receive command byte - sample each bit on posedge SCK
        // We also need to drive MISO but for cmd byte it's don't care
        cmd = 8'h00;
        for (j=7; j>=0; j=j-1) begin
            @(posedge spi_sck or posedge spi_cs_n);
            if (spi_cs_n) disable do_transaction;
            cmd[j] = spi_mosi;
            if (j > 0) begin
                // Set up MISO for next position (before next rising edge)
                // Will be overridden after we know the full command
                @(negedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) disable do_transaction;
            end
        end
        rw   = cmd[7];
        addr = cmd[6:0];
        bn   = 0;

        while (!spi_cs_n) begin
            // Determine TX byte
            if (rw) begin
                if (whoami_bad && (addr+bn) == 7'h75) tx = 8'hFF;
                else tx = regs[(addr+bn) & 8'h7F];
            end else tx = 8'h00;

            // Drive first MISO bit BEFORE first rising SCK of this byte
            // We're currently just after the last posedge of previous byte
            // Next event is negedge SCK -> then posedge -> etc
            rx = 8'h00;
            for (j=7; j>=0; j=j-1) begin
                // Set MISO on negedge (or right after posedge for first bit)
                @(negedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) begin spi_miso=0; disable do_transaction; end
                spi_miso = tx[j];
                // Sample MOSI on posedge
                @(posedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) disable do_transaction;
                rx[j] = spi_mosi;
            end
            if (!rw) regs[addr] = rx;
            bn = bn + 1;
        end
        spi_miso = 1'b0;
    end
endtask

initial begin
    for(ki=0;ki<128;ki=ki+1) regs[ki]=8'h00;
    regs[8'h75]=8'h47;
    regs[8'h1D]=8'h00; regs[8'h1E]=8'h64;
    regs[8'h1F]=8'h12; regs[8'h20]=8'h34;
    regs[8'h21]=8'h56; regs[8'h22]=8'h78;
    regs[8'h23]=8'h9A; regs[8'h24]=8'hBC;
    regs[8'h25]=8'hDE; regs[8'h26]=8'hF0;
    regs[8'h27]=8'h11; regs[8'h28]=8'h22;
    regs[8'h29]=8'h33; regs[8'h2A]=8'h44;
    spi_miso=1'b0; whoami_bad=0;
    forever begin
        @(negedge spi_cs_n);
        do_transaction;
    end
end

// ========== Tests ==========
integer cnt, sample_cnt, i2;

initial begin
    $dumpfile("tb_icm42688_ctrl.vcd");
    $dumpvars(0, tb_icm42688_ctrl);
    rst_n=0;
    repeat(20) @(posedge clk); rst_n=1;

    $display("\n--- TEST 1: Init ---");
    cnt=0; while (!init_done && cnt<200000) begin @(posedge clk); cnt=cnt+1; end
    chk(init_done, "init_done asserted");
    chk(!imu_error, "No error during init");
    $display("    init in %0d cycles", cnt);

    $display("\n--- TEST 2: Data valid ---");
    cnt=0; while (!data_valid && cnt<50000) begin @(posedge clk); cnt=cnt+1; end
    chk(data_valid, "data_valid fires");

    $display("\n--- TEST 3: Accel X ---");
    cnt=0; while (!data_valid && cnt<50000) begin @(posedge clk); cnt=cnt+1; end
    @(posedge clk);
    $display("    accel_x=0x%04X expect 0x1234", accel_x);
    chk(accel_x == $signed(16'h1234), "Accel X = 0x1234");

    $display("\n--- TEST 4: Gyro X ---");
    cnt=0; while (!data_valid && cnt<50000) begin @(posedge clk); cnt=cnt+1; end
    @(posedge clk);
    $display("    gyro_x=0x%04X expect 0xDEF0", gyro_x);
    chk(gyro_x == $signed(16'hDEF0), "Gyro X = 0xDEF0");

    $display("\n--- TEST 5: Temp ---");
    cnt=0; while (!data_valid && cnt<50000) begin @(posedge clk); cnt=cnt+1; end
    @(posedge clk);
    $display("    temp=0x%04X expect 0x0064", temp_data);
    chk(temp_data == $signed(16'h0064), "Temp = 0x0064");

    $display("\n--- TEST 6: 3 samples ---");
    sample_cnt=0;
    for (i2=0;i2<3;i2=i2+1) begin
        cnt=0; while (!data_valid && cnt<50000) begin @(posedge clk); cnt=cnt+1; end
        if (data_valid) sample_cnt=sample_cnt+1;
        @(posedge clk);
    end
    chk(sample_cnt==3, "3 consecutive samples");

    $display("\n--- TEST 7: WHO_AM_I fail ---");
    rst_n=0; whoami_bad=1;
    repeat(10) @(posedge clk); rst_n=1;
    cnt=0; while (!imu_error && cnt<200000) begin @(posedge clk); cnt=cnt+1; end
    chk(imu_error, "Error flag on bad WHO_AM_I");
    chk(!init_done, "init_done cleared on error");

    $display("\n--- TEST 8: Recovery ---");
    whoami_bad=0;
    cnt=0; while (!init_done && cnt<1000000) begin @(posedge clk); cnt=cnt+1; end
    chk(init_done, "Recovered after WHO_AM_I fix");

    $display("\n========================================");
    $display("ICM42688 Ctrl TB: PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
    $display("========================================");
    if (fail_cnt==0) $display("ALL TESTS PASSED");
    else             $display("*** %0d FAILED ***", fail_cnt);
    $finish;
end

initial begin #10_000_000_000; $display("TIMEOUT"); $finish; end
endmodule
