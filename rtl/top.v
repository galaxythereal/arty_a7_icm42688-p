// =============================================================================
// top.v
// Top-level module: ICM-42688-P IMU Streaming on Arty A7-100T
//
// Hardware connections (using Pmod JA - standard pmod, no 200Ω resistors issue
// since SPI is 1MHz):
//   JA[0] (G13) -> IMU MOSI (SDI)
//   JA[1] (B11) -> IMU MISO (SDO)
//   JA[2] (A11) -> IMU SCK  (SCLK)
//   JA[3] (D12) -> IMU CS_N (nCS)
//   JA[4] (D13) -> IMU INT1 (interrupt, optional input)
//
// UART via onboard USB-UART bridge (U5, FTDI):
//   IO4 (D10) = UART TX -> goes to USB UART RX on Arty
//
// LEDs:
//   LD0 (H17) = init done
//   LD1 (K15) = data streaming (blinks)
//   LD2 (J13) = error
// =============================================================================

module top (
    input  wire        clk,          // 100MHz system clock (E3)

    // Pmod JA (SPI to IMU)
    output wire        ja_pin1,      // MOSI  G13
    input  wire        ja_pin2,      // MISO  B11
    output wire        ja_pin3,      // SCK   A11
    output wire        ja_pin4,      // CS_N  D12

    // UART to host
    output wire        uart_rxd_out, // IO4 = D10

    // LEDs
    output wire [3:0]  led,          // H17 K15 J13 J14

    // Reset button (active-low)
    input  wire        resetn         // C2
);

// =====================================================================
// Parameters
// =====================================================================
localparam SYS_CLK_HZ   = 100_000_000;
localparam SPI_CLK_HZ   = 1_000_000;
localparam UART_BAUD    = 115_200;

// =====================================================================
// IMU signals
// =====================================================================
wire signed [15:0] accel_x, accel_y, accel_z;
wire signed [15:0] gyro_x,  gyro_y,  gyro_z;
wire signed [15:0] temp_data;
wire               data_valid;
wire               init_done;
wire               imu_error;
wire               spi_sck, spi_mosi, spi_miso, spi_cs_n;

assign spi_miso = ja_pin2;
assign ja_pin1  = spi_mosi;
assign ja_pin3  = spi_sck;
assign ja_pin4  = spi_cs_n;

// =====================================================================
// ICM-42688-P Controller
// =====================================================================
icm42688_ctrl #(
    .SYS_CLK_HZ (SYS_CLK_HZ),
    .SPI_CLK_HZ (SPI_CLK_HZ)
) u_imu (
    .clk_i       (clk),
    .rst_ni      (resetn),
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

// =====================================================================
// Data streamer (UART)
// =====================================================================
wire led_init, led_data, led_error;

imu_data_stream #(
    .CLK_HZ    (SYS_CLK_HZ),
    .BAUD_RATE (UART_BAUD)
) u_stream (
    .clk_i       (clk),
    .rst_ni      (resetn),
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
    .uart_tx_o   (uart_rxd_out),
    .led_init_o  (led_init),
    .led_data_o  (led_data),
    .led_error_o (led_error)
);

assign led[0] = led_init;
assign led[1] = led_data;
assign led[2] = led_error;
assign led[3] = 1'b0;

endmodule
