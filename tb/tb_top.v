// tb_top.v - Integration testbench
`timescale 1ns/1ps

module tb_top;

reg  clk, resetn;
wire spi_sck, spi_mosi, spi_cs_n;
reg  spi_miso;
wire uart_txd;
wire [1:0] led;

// Instantiate modules directly with fast params
wire signed [15:0] accel_x, accel_y, accel_z;
wire signed [15:0] gyro_x, gyro_y, gyro_z;
wire signed [15:0] temp_data;
wire data_valid, init_done, imu_error;

icm42688_ctrl #(.SYS_CLK_HZ(10_000),.SPI_CLK_HZ(1_000),.CLK_DIV(5)) u_imu (
    .clk_i(clk),.rst_ni(resetn),
    .accel_x_o(accel_x),.accel_y_o(accel_y),.accel_z_o(accel_z),
    .gyro_x_o(gyro_x),.gyro_y_o(gyro_y),.gyro_z_o(gyro_z),
    .temp_o(temp_data),.data_valid_o(data_valid),.init_done_o(init_done),.error_o(imu_error),
    .spi_sck_o(spi_sck),.spi_mosi_o(spi_mosi),.spi_miso_i(spi_miso),.spi_cs_n_o(spi_cs_n)
);

reg led_init, led_data, led_error;

imu_data_stream #(.CLK_HZ(10_000),.BAUD_RATE(1_000)) u_stream (
    .clk_i(clk),.rst_ni(resetn),
    .accel_x_i(accel_x),.accel_y_i(accel_y),.accel_z_i(accel_z),
    .gyro_x_i(gyro_x),.gyro_y_i(gyro_y),.gyro_z_i(gyro_z),
    .temp_i(temp_data),.data_valid_i(data_valid),.init_done_i(init_done),.error_i(imu_error),
    .uart_tx_o(uart_txd),
    .led_init_o(led_init),.led_data_o(led_data),.led_error_o(led_error)
);

initial clk=0; always #50000 clk=~clk;

integer pass_cnt=0, fail_cnt=0;
task chk; input cond; input [255:0] msg; begin
    if (cond) begin $display("[PASS] %s",msg); pass_cnt=pass_cnt+1; end
    else       begin $display("[FAIL] %s @%0t",msg,$time); fail_cnt=fail_cnt+1; end
end endtask

// ===== SPI Slave (same as ctrl TB) =====
reg [7:0] regs[0:127];
reg whoami_bad;
integer ki;

task do_transaction;
    reg [7:0] cmd, rx, tx;
    reg rw; reg [6:0] addr;
    integer bn, j;
    begin
        cmd=8'h00;
        for (j=7;j>=0;j=j-1) begin
            @(posedge spi_sck or posedge spi_cs_n);
            if (spi_cs_n) disable do_transaction;
            cmd[j]=spi_mosi;
            if (j>0) begin
                @(negedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) disable do_transaction;
            end
        end
        rw=cmd[7]; addr=cmd[6:0]; bn=0;
        while (!spi_cs_n) begin
            if (rw) begin
                if (whoami_bad && (addr+bn)==7'h75) tx=8'hFF;
                else tx=regs[(addr+bn) & 8'h7F];
            end else tx=8'h00;
            rx=8'h00;
            for (j=7;j>=0;j=j-1) begin
                @(negedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) begin spi_miso=0; disable do_transaction; end
                spi_miso=tx[j];
                @(posedge spi_sck or posedge spi_cs_n);
                if (spi_cs_n) disable do_transaction;
                rx[j]=spi_mosi;
            end
            if (!rw) regs[addr]=rx;
            bn=bn+1;
        end
        spi_miso=1'b0;
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
    forever begin @(negedge spi_cs_n); do_transaction; end
end

// ===== UART Receiver (1000 baud, 10kHz clk -> 10 cycles/bit) =====
// Bit period = 10 cycles * 100us = 1ms = 1_000_000 ns
localparam UART_BIT_NS = 1_000_000;

reg [7:0] uart_pkt [0:17];
integer   total_pkts;

task uart_recv_byte; output [7:0] b; integer j; begin
    b=8'h00;
    @(negedge uart_txd);                    // start bit
    #(UART_BIT_NS + UART_BIT_NS/2);        // 1.5 bit periods into data
    for (j=0;j<8;j=j+1) begin
        b[j]=uart_txd;
        if (j<7) #UART_BIT_NS;
    end
    #(UART_BIT_NS/2); // to stop bit
end endtask

// ===== Tests =====
integer cnt, i3;
reg [7:0] rb;

initial begin
    $dumpfile("tb_top.vcd");
    $dumpvars(0, tb_top);
    resetn=0; total_pkts=0;
    repeat(20) @(posedge clk); resetn=1;

    // TEST 1: Init
    $display("\n--- TEST 1: System init ---");
    cnt=0; while (!init_done && cnt<200000) begin @(posedge clk); cnt=cnt+1; end
    repeat(2) @(posedge clk);  // wait for LED register to update
    chk(init_done, "init_done asserted");
    chk(!imu_error, "No errors");
    chk(led_init==1'b1, "LED_init lit");

    // TEST 2: Receive one UART packet
    $display("\n--- TEST 2: UART packet ---");
    begin
        reg [7:0] rcv;
        uart_recv_byte(rcv); uart_pkt[0]=rcv;
        uart_recv_byte(rcv); uart_pkt[1]=rcv;
        chk(uart_pkt[0]==8'hAA, "Packet header[0]=0xAA");
        chk(uart_pkt[1]==8'h55, "Packet header[1]=0x55");
        for (i3=2;i3<18;i3=i3+1) begin
            uart_recv_byte(rcv); uart_pkt[i3]=rcv;
        end
        chk(uart_pkt[16]==8'h0D, "CRLF: CR");
        chk(uart_pkt[17]==8'h0A, "CRLF: LF");
        total_pkts=total_pkts+1;
    end

    // TEST 3: Accel data in packet
    $display("\n--- TEST 3: Accel X in packet ---");
    chk({uart_pkt[2],uart_pkt[3]}==16'h1234, "Accel X in UART packet");

    // TEST 4: 3 more packets
    $display("\n--- TEST 4: 3 more packets ---");
    begin
        integer pi;
        reg [7:0] ph0, ph1;
        for (pi=0;pi<3;pi=pi+1) begin
            uart_recv_byte(ph0); uart_recv_byte(ph1);
            chk(ph0==8'hAA && ph1==8'h55, "Packet has valid header");
            for (i3=2;i3<18;i3=i3+1) begin
                uart_recv_byte(rb); uart_pkt[i3]=rb;
            end
            total_pkts=total_pkts+1;
        end
        $display("    Total packets: %0d", total_pkts);
        chk(total_pkts>=4, "Received 4+ packets");
    end

    // TEST 5: Reset and reinit
    $display("\n--- TEST 5: Reset & re-init ---");
    resetn=0; repeat(10) @(posedge clk); resetn=1;
    cnt=0; while (!init_done && cnt<200000) begin @(posedge clk); cnt=cnt+1; end
    repeat(2) @(posedge clk);
    chk(init_done, "init_done after reset");

    // TEST 6: LED status
    $display("\n--- TEST 6: LED status ---");
    chk(led_init==1'b1, "LED_init=1 after init");
    chk(led_error==1'b0, "LED_error=0 no error");

    $display("\n========================================");
    $display("TOP Integration TB: PASS=%0d FAIL=%0d", pass_cnt, fail_cnt);
    $display("========================================");
    if (fail_cnt==0) $display("ALL TESTS PASSED");
    else             $display("*** %0d FAILED ***", fail_cnt);
    $finish;
end

initial begin #20_000_000_000; $display("WATCHDOG"); $finish; end
endmodule
