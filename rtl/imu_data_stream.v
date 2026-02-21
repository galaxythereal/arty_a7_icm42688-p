// =============================================================================
// imu_data_stream.v
// Formats IMU data as ASCII and streams over UART
//
// Output format per line:
//   "AX:+DDDDD AY:+DDDDD AZ:+DDDDD GX:+DDDDD GY:+DDDDD GZ:+DDDDD T:+DDDDD\r\n"
// Where DDDDD is decimal signed 16-bit value.
//
// Simpler binary packet format (62 bytes):
//   [0xAA][0x55][AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L]
//   [GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L][T_H][T_L][0x0D][0x0A]
// =============================================================================
module imu_data_stream #(
    parameter CLK_HZ    = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire        clk_i,
    input  wire        rst_ni,

    // From IMU controller
    input  wire signed [15:0] accel_x_i,
    input  wire signed [15:0] accel_y_i,
    input  wire signed [15:0] accel_z_i,
    input  wire signed [15:0] gyro_x_i,
    input  wire signed [15:0] gyro_y_i,
    input  wire signed [15:0] gyro_z_i,
    input  wire signed [15:0] temp_i,
    input  wire        data_valid_i,  // pulse triggers transmission
    input  wire        init_done_i,
    input  wire        error_i,

    // UART
    output wire        uart_tx_o,

    // Status LEDs
    output reg         led_init_o,
    output reg         led_data_o,
    output reg         led_error_o
);

// =====================================================================
// Binary packet: 18 bytes
//   [0xAA][0x55] = header
//   [AX_H][AX_L][AY_H][AY_L][AZ_H][AZ_L] = accel (6 bytes)
//   [GX_H][GX_L][GY_H][GY_L][GZ_H][GZ_L] = gyro  (6 bytes)
//   [T_H][T_L]  = temp  (2 bytes)
//   [0x0D][0x0A] = CRLF
// =====================================================================
localparam PKT_LEN = 18;

reg [7:0]  pkt_buf [0:PKT_LEN-1];
reg [4:0]  pkt_idx_r;
reg        sending_r;
reg        uart_start_r;
reg [7:0]  uart_data_r;

// UART
wire uart_busy_w;

uart_tx #(
    .CLK_HZ    (CLK_HZ),
    .BAUD_RATE (BAUD_RATE)
) u_uart (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .start_i (uart_start_r),
    .data_i  (uart_data_r),
    .tx_o    (uart_tx_o),
    .busy_o  (uart_busy_w)
);

// Packet assembly
always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        sending_r    <= 1'b0;
        pkt_idx_r    <= 0;
        uart_start_r <= 1'b0;
        uart_data_r  <= 8'h00;
        led_init_o   <= 1'b0;
        led_data_o   <= 1'b0;
        led_error_o  <= 1'b0;
    end else begin
        uart_start_r <= 1'b0;
        led_init_o   <= init_done_i;
        led_error_o  <= error_i;

        if (data_valid_i && !sending_r) begin
            // Assemble packet
            pkt_buf[0]  <= 8'hAA;
            pkt_buf[1]  <= 8'h55;
            pkt_buf[2]  <= accel_x_i[15:8];
            pkt_buf[3]  <= accel_x_i[7:0];
            pkt_buf[4]  <= accel_y_i[15:8];
            pkt_buf[5]  <= accel_y_i[7:0];
            pkt_buf[6]  <= accel_z_i[15:8];
            pkt_buf[7]  <= accel_z_i[7:0];
            pkt_buf[8]  <= gyro_x_i[15:8];
            pkt_buf[9]  <= gyro_x_i[7:0];
            pkt_buf[10] <= gyro_y_i[15:8];
            pkt_buf[11] <= gyro_y_i[7:0];
            pkt_buf[12] <= gyro_z_i[15:8];
            pkt_buf[13] <= gyro_z_i[7:0];
            pkt_buf[14] <= temp_i[15:8];
            pkt_buf[15] <= temp_i[7:0];
            pkt_buf[16] <= 8'h0D;  // CR
            pkt_buf[17] <= 8'h0A;  // LF

            pkt_idx_r  <= 0;
            sending_r  <= 1'b1;
            led_data_o <= 1'b1;
        end

        if (sending_r) begin
            if (!uart_busy_w && !uart_start_r) begin
                uart_data_r  <= pkt_buf[pkt_idx_r];
                uart_start_r <= 1'b1;
                pkt_idx_r    <= pkt_idx_r + 1;
                if (pkt_idx_r == PKT_LEN - 1) begin
                    sending_r  <= 1'b0;
                    led_data_o <= 1'b0;
                end
            end
        end
    end
end

endmodule
