// =============================================================================
// tb_uart_tx.v
// Testbench for uart_tx.v
//
// Tests:
//  1. Single byte transmission (start/stop bit, data)
//  2. Multiple byte back-to-back
//  3. Busy signal timing
//  4. 0x00 and 0xFF corner cases
//  5. UART data integrity (receiver model)
// =============================================================================
`timescale 1ns/1ps

module tb_uart_tx;

reg        clk;
reg        rst_n;
reg        start;
reg  [7:0] data;
wire       tx;
wire       busy;

// 100MHz clock, 115200 baud
// CLK_DIV = 100e6/115200 ≈ 868
// Use faster params for sim: 10MHz clock, 1Mbaud -> div=10
uart_tx #(
    .CLK_HZ    (10_000_000),
    .BAUD_RATE (1_000_000),
    .CLK_DIV   (10)
) dut (
    .clk_i   (clk),
    .rst_ni  (rst_n),
    .start_i (start),
    .data_i  (data),
    .tx_o    (tx),
    .busy_o  (busy)
);

initial clk = 0;
always #50 clk = ~clk;  // 10MHz

integer pass_cnt = 0;
integer fail_cnt = 0;

task check_t;
    input cond;
    input [127:0] msg;
    begin
        if (cond) begin $display("[PASS] %s", msg); pass_cnt++; end
        else       begin $display("[FAIL] %s @%0t", msg, $time); fail_cnt++; end
    end
endtask

// UART receiver model
// Receives one byte after seeing start bit
task uart_receive;
    output [7:0] received;
    input        expected_good;
    begin
        reg [7:0] rx_byte;
        integer   bit_idx;
        // Wait for start bit (TX goes low)
        @(negedge tx);
        // Sample in middle of start bit
        #500;  // half bit period = 10 * 50ns / 2 = 250ns... use 500ns
        // Wait full start bit
        #1000;
        // Sample 8 data bits
        for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx+1) begin
            rx_byte[bit_idx] = tx;
            #1000;  // 1 bit period (clk_div=10 cycles * 100ns)
        end
        // Verify stop bit
        check_t(tx == 1'b1, "Stop bit = 1");
        received = rx_byte;
    end
endtask

reg [7:0] rx_byte;

initial begin
    $dumpfile("tb_uart_tx.vcd");
    $dumpvars(0, tb_uart_tx);

    rst_n = 0;
    start = 0;
    data  = 0;
    repeat(10) @(posedge clk);
    rst_n = 1;
    repeat(5) @(posedge clk);

    // Test 1: Idle state
    $display("\n--- TEST 1: Idle state ---");
    check_t(tx == 1'b1, "TX idle high");
    check_t(!busy, "Not busy at idle");

    // Test 2: Send 0x55 (0101_0101)
    $display("\n--- TEST 2: Send 0x55 ---");
    data  = 8'h55;
    start = 1'b1;
    @(posedge clk);
    start = 1'b0;
    check_t(busy, "Busy after start");
    uart_receive(rx_byte, 1);
    check_t(rx_byte == 8'h55, "Received 0x55 correct");

    // Wait for done
    wait(!busy);
    check_t(tx == 1'b1, "TX returns to idle");

    // Test 3: Send 0xAA
    $display("\n--- TEST 3: Send 0xAA ---");
    data = 8'hAA;
    @(posedge clk); start = 1'b1;
    @(posedge clk); start = 1'b0;
    uart_receive(rx_byte, 1);
    check_t(rx_byte == 8'hAA, "Received 0xAA correct");
    wait(!busy);

    // Test 4: Corner case 0x00
    $display("\n--- TEST 4: Send 0x00 ---");
    data = 8'h00;
    @(posedge clk); start = 1'b1;
    @(posedge clk); start = 1'b0;
    uart_receive(rx_byte, 1);
    check_t(rx_byte == 8'h00, "Received 0x00 correct");
    wait(!busy);

    // Test 5: Corner case 0xFF
    $display("\n--- TEST 5: Send 0xFF ---");
    data = 8'hFF;
    @(posedge clk); start = 1'b1;
    @(posedge clk); start = 1'b0;
    uart_receive(rx_byte, 1);
    check_t(rx_byte == 8'hFF, "Received 0xFF correct");
    wait(!busy);

    // Test 6: Back-to-back bytes
    $display("\n--- TEST 6: Back-to-back bytes (5 bytes) ---");
    begin : blk6
        integer k;
        reg [7:0] expected_val;
        reg [7:0] recv;
        for (k = 0; k < 5; k = k + 1) begin
            expected_val = 8'h30 + k;
            data = expected_val;
            wait(!busy);
            @(posedge clk); start = 1'b1;
            @(posedge clk); start = 1'b0;
            uart_receive(recv, 1);
            check_t(recv == expected_val, "Back-to-back byte correct");
        end
    end
    wait(!busy);

    // Test 7: Busy timing
    $display("\n--- TEST 7: Busy timing ---");
    begin
        integer busy_cycles;
        data = 8'h42;
        @(posedge clk); start = 1'b1;
        @(posedge clk); start = 1'b0;
        busy_cycles = 0;
        while (busy) begin
            @(posedge clk);
            busy_cycles = busy_cycles + 1;
        end
        // 10 bit periods (start + 8 data + stop) * CLK_DIV cycles
        // = 10 * 10 = 100 cycles (approx)
        $display("    Busy for %0d cycles", busy_cycles);
        check_t(busy_cycles >= 90 && busy_cycles <= 115, "Busy duration reasonable");
    end

    $display("\n========================================");
    $display("UART TX TB: PASS=%0d  FAIL=%0d", pass_cnt, fail_cnt);
    $display("========================================\n");

    if (fail_cnt == 0) $display("ALL TESTS PASSED");
    else               $display("*** %0d FAILED ***", fail_cnt);

    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
