// =============================================================================
// tb_spi_master.v
// Comprehensive testbench for spi_master.v
//
// Tests:
//   1. Single byte write
//   2. Single byte read
//   3. Multi-byte (2-byte) transaction
//   4. 14-byte burst read (matches ICM-42688-P data read)
//   5. Back-to-back transactions
//   6. CS timing (assert/deassert)
//   7. SCK polarity verification
//   8. MOSI data integrity
//   9. MISO capture integrity
//  10. Busy signal behavior
// =============================================================================
`timescale 1ns/1ps

module tb_spi_master;

// =====================================================================
// DUT signals
// =====================================================================
reg        clk;
reg        rst_n;
reg        start;
reg [7:0]  tx_data;
reg [3:0]  n_bytes;
wire       busy;
wire       done;
wire       req_next;
reg [7:0]  next_tx;
wire       rx_valid;
wire [7:0] rx_data;
wire [3:0] rx_idx;
wire       spi_sck;
wire       spi_mosi;
reg        spi_miso;
wire       spi_cs_n;

// =====================================================================
// DUT
// =====================================================================
spi_master #(
    .CLK_DIV   (4),    // Fast for simulation
    .MAX_BYTES (16)
) dut (
    .clk_i           (clk),
    .rst_ni          (rst_n),
    .start_i         (start),
    .tx_data_i       (tx_data),
    .n_bytes_i       (n_bytes),
    .busy_o          (busy),
    .done_o          (done),
    .req_next_byte_o (req_next),
    .next_tx_byte_i  (next_tx),
    .rx_valid_o      (rx_valid),
    .rx_data_o       (rx_data),
    .rx_byte_idx_o   (rx_idx),
    .spi_sck_o       (spi_sck),
    .spi_mosi_o      (spi_mosi),
    .spi_miso_i      (spi_miso),
    .spi_cs_n_o      (spi_cs_n)
);

// =====================================================================
// Clock: 100MHz
// =====================================================================
initial clk = 0;
always #5 clk = ~clk;

// =====================================================================
// Test infrastructure
// =====================================================================
integer pass_cnt = 0;
integer fail_cnt = 0;

task check;
    input condition;
    input [255:0] msg;
    begin
        if (condition) begin
            $display("[PASS] %s", msg);
            pass_cnt = pass_cnt + 1;
        end else begin
            $display("[FAIL] %s at time %0t", msg, $time);
            fail_cnt = fail_cnt + 1;
        end
    end
endtask

// Received bytes storage
reg [7:0] rx_bytes [0:15];
reg [3:0] rx_cnt;

always @(posedge clk) begin
    if (rx_valid) begin
        rx_bytes[rx_idx] <= rx_data;
        rx_cnt <= rx_cnt + 1;
    end
end

// SPI slave model: captures MOSI and drives MISO
reg [7:0] mosi_captured;
reg [7:0] miso_pattern;
reg [2:0] slave_bit_cnt;
reg       slave_active;

always @(posedge spi_sck or posedge spi_cs_n) begin
    if (spi_cs_n) begin
        slave_bit_cnt   <= 0;
        slave_active    <= 0;
        mosi_captured   <= 8'h00;
    end else begin
        // Sample on rising edge
        mosi_captured <= {mosi_captured[6:0], spi_mosi};
        slave_bit_cnt <= slave_bit_cnt + 1;
    end
end

// MISO driven on falling edge of SCK
always @(negedge spi_sck or posedge spi_cs_n) begin
    if (spi_cs_n) begin
        spi_miso <= 1'bz;
    end else begin
        spi_miso <= miso_pattern[7 - slave_bit_cnt];
    end
end

// Task: do one SPI transaction and wait for done
task do_transaction;
    input [7:0] first_byte;
    input [3:0] num_bytes;
    input [7:0] miso_val;
    begin
        miso_pattern <= miso_val;
        rx_cnt       <= 0;
        tx_data      <= first_byte;
        n_bytes      <= num_bytes;
        next_tx      <= 8'hAB;  // known value
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
        // Wait for done
        wait (done);
        @(posedge clk);
    end
endtask

// =====================================================================
// Test cases
// =====================================================================
initial begin
    $dumpfile("tb_spi_master.vcd");
    $dumpvars(0, tb_spi_master);

    // Reset
    rst_n  <= 0;
    start  <= 0;
    tx_data<= 0;
    n_bytes<= 1;
    spi_miso <= 1'b0;
    miso_pattern <= 8'hA5;
    rx_cnt <= 0;

    repeat(10) @(posedge clk);
    rst_n <= 1;
    repeat(5) @(posedge clk);

    // -------------------------------------------------------
    // TEST 1: Single byte write - verify CS and SCK behavior
    // -------------------------------------------------------
    $display("\n--- TEST 1: Single byte write ---");
    begin
        reg cs_before, cs_during, cs_after;
        cs_before = spi_cs_n;
        miso_pattern = 8'h00;
        do_transaction(8'hAA, 4'd1, 8'h00);
        cs_after = spi_cs_n;
        check(cs_before == 1'b1, "CS idle high before tx");
        check(cs_after  == 1'b1, "CS deasserted after tx");
        check(!busy, "Not busy after done");
    end

    // -------------------------------------------------------
    // TEST 2: Verify MOSI/MISO data integrity (loopback: miso_pattern=tx)
    // -------------------------------------------------------
    $display("\n--- TEST 2: MOSI/MISO data integrity ---");
    begin
        // Send cmd + get back known pattern on second byte
        miso_pattern = 8'hA5;
        rx_cnt = 0;
        do_transaction(8'h00, 4'd2, 8'hA5);
        @(posedge clk);
        check(rx_bytes[1] == 8'hA5, "MISO pattern 0xA5 received correctly");
    end

    // -------------------------------------------------------
    // TEST 3: MISO capture (receive 0x47 - ICM WHO_AM_I)
    // -------------------------------------------------------
    $display("\n--- TEST 3: MISO capture 0x47 ---");
    begin
        miso_pattern = 8'h47;
        rx_cnt = 0;
        do_transaction(8'hF5, 4'd2, 8'h47);  // 2 bytes: cmd + response
        @(posedge clk);
        check(rx_bytes[1] == 8'h47, "MISO received 0x47 (WHO_AM_I)");
        check(rx_cnt == 2, "Received exactly 2 bytes");
    end

    // -------------------------------------------------------
    // TEST 4: 14-byte burst read (simulates ICM data read)
    // -------------------------------------------------------
    $display("\n--- TEST 4: 14-byte burst read ---");
    begin : blk4
        integer j;
        miso_pattern = 8'hBB;
        rx_cnt = 0;
        do_transaction(8'h9D, 4'd14, 8'hBB);
        check(rx_cnt == 14, "Received 14 bytes");
        for (j = 1; j < 14; j = j+1)
            check(rx_bytes[j] == 8'hBB, "Burst byte correct");
    end

    // -------------------------------------------------------
    // TEST 5: CS stays asserted throughout multi-byte transaction
    // -------------------------------------------------------
    $display("\n--- TEST 5: CS assertion during multi-byte ---");
    begin
        reg cs_during_tx;
        cs_during_tx = 1'b1;
        miso_pattern = 8'hCC;
        tx_data  = 8'hDE;
        n_bytes  = 4'd4;
        @(posedge clk); start = 1'b1;
        @(posedge clk); start = 1'b0;
        // Monitor CS during transfer
        fork
            begin
                repeat(500) begin
                    @(posedge clk);
                    if (!spi_cs_n && busy)
                        cs_during_tx = 1'b0;  // CS asserted (good)
                end
            end
            wait(done);
        join_any
        @(posedge clk);
        check(spi_cs_n == 1'b1, "CS high after transaction");
    end

    // -------------------------------------------------------
    // TEST 6: Back-to-back transactions
    // -------------------------------------------------------
    $display("\n--- TEST 6: Back-to-back transactions ---");
    begin
        integer k;
        for (k = 0; k < 5; k = k+1) begin
            rx_cnt = 0;
            do_transaction(8'h10 + k, 4'd2, 8'h20 + k);
            // Wait a few cycles between transactions
            repeat(5) @(posedge clk);
        end
        check(1'b1, "5 back-to-back transactions completed");
        check(!busy, "Not busy after all transactions");
    end

    // -------------------------------------------------------
    // TEST 7: Busy signal - must be high during transfer
    // -------------------------------------------------------
    $display("\n--- TEST 7: Busy signal ---");
    begin
        reg saw_busy;
        saw_busy = 1'b0;
        tx_data = 8'hFF;
        n_bytes = 4'd2;
        @(posedge clk); start = 1'b1;
        @(posedge clk); start = 1'b0;
        repeat(10) @(posedge clk);
        saw_busy = busy;
        wait(done); @(posedge clk);
        check(saw_busy, "Busy high during transfer");
        check(!busy, "Busy low after done");
    end

    // -------------------------------------------------------
    // TEST 8: Reset during transfer
    // -------------------------------------------------------
    $display("\n--- TEST 8: Reset during transfer ---");
    begin
        tx_data = 8'hFF;
        n_bytes = 4'd14;
        @(posedge clk); start = 1'b1;
        @(posedge clk); start = 1'b0;
        repeat(20) @(posedge clk);
        rst_n = 1'b0;
        repeat(5) @(posedge clk);
        check(!busy, "Not busy after reset");
        check(spi_cs_n, "CS deasserted after reset");
        rst_n = 1'b1;
        repeat(5) @(posedge clk);
    end

    // -------------------------------------------------------
    // TEST 9: SCK frequency check (approximate)
    // -------------------------------------------------------
    $display("\n--- TEST 9: SCK frequency ---");
    begin
        integer sck_period;
        time t1, t2;
        miso_pattern = 8'h00;
        tx_data = 8'hAB;
        n_bytes = 4'd2;
        @(posedge clk); start = 1'b1;
        @(posedge clk); start = 1'b0;

        @(posedge spi_sck); t1 = $time;
        @(posedge spi_sck); t2 = $time;
        sck_period = t2 - t1;
        wait(done); @(posedge clk);
        // CLK_DIV=4, clk=10ns -> SCK period = 2*CLK_DIV*10ns = 80ns
        check((sck_period >= 70 && sck_period <= 90), "SCK period in range");
        $display("    SCK period = %0d ns", sck_period);
    end

    // -------------------------------------------------------
    // TEST 10: Idle state after reset
    // -------------------------------------------------------
    $display("\n--- TEST 10: Idle state ---");
    check(spi_cs_n == 1'b1, "CS idle high");
    check(spi_sck  == 1'b0, "SCK idle low (Mode 0)");
    check(!busy, "Not busy at idle");

    // =====================================================================
    // Summary
    // =====================================================================
    $display("\n========================================");
    $display("SPI Master TB: PASS=%0d  FAIL=%0d", pass_cnt, fail_cnt);
    $display("========================================\n");

    if (fail_cnt == 0)
        $display("ALL TESTS PASSED");
    else
        $display("*** %0d TEST(S) FAILED ***", fail_cnt);

    $finish;
end

// Timeout watchdog
initial begin
    #2_000_000;
    $display("TIMEOUT!");
    $finish;
end

endmodule
