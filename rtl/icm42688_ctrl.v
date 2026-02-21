// icm42688_ctrl.v - ICM-42688-P IMU Controller
`include "icm42688_regs.vh"

module icm42688_ctrl #(
    parameter SYS_CLK_HZ = 100_000_000,
    parameter SPI_CLK_HZ = 1_000_000,
    parameter CLK_DIV    = SYS_CLK_HZ / (SPI_CLK_HZ * 2)
)(
    input  wire        clk_i,
    input  wire        rst_ni,
    output reg  signed [15:0] accel_x_o,
    output reg  signed [15:0] accel_y_o,
    output reg  signed [15:0] accel_z_o,
    output reg  signed [15:0] gyro_x_o,
    output reg  signed [15:0] gyro_y_o,
    output reg  signed [15:0] gyro_z_o,
    output reg  signed [15:0] temp_o,
    output reg         data_valid_o,
    output reg         init_done_o,
    output reg         error_o,
    output wire        spi_sck_o,
    output wire        spi_mosi_o,
    input  wire        spi_miso_i,
    output wire        spi_cs_n_o
);

// States
localparam ST_RESET         = 4'd0;
localparam ST_BOOT_WAIT     = 4'd1;
localparam ST_WRITE_DEVCFG  = 4'd2;
localparam ST_WAIT_SRESET   = 4'd3;
localparam ST_READ_WHOAMI   = 4'd4;
localparam ST_WRITE_INTFCFG = 4'd5;
localparam ST_WRITE_PWRMGMT = 4'd6;
localparam ST_WAIT_ENABLE   = 4'd7;
localparam ST_WRITE_GYROCFG = 4'd8;
localparam ST_WRITE_ACCECFG = 4'd9;
localparam ST_INIT_DONE     = 4'd10;
localparam ST_POLL_START    = 4'd11;
localparam ST_POLL_WAIT     = 4'd12;
localparam ST_POLL_DELAY    = 4'd13;
localparam ST_ERROR         = 4'd15;

// Timing
localparam RESET_CYCLES   = (SYS_CLK_HZ / 1000);
localparam BOOT_CYCLES    = (SYS_CLK_HZ / 1000) * 10;  // 10ms post-reset (ICM-42688-P req.)
localparam WAIT_EN_CYCLES = (SYS_CLK_HZ / 1000) * 45;
localparam POLL_CYCLES    = (SYS_CLK_HZ / 1000);
localparam ERROR_CYCLES   = SYS_CLK_HZ;

// Configuration bytes (avoid macro concatenation issues)
// GYRO_CONFIG0: FS=±2000dps (000), ODR=1kHz (0110) -> 8'b000_0_0110 = 8'h06
localparam GYRO_CFG0_VAL  = 8'h06;
// ACCEL_CONFIG0: FS=±16g (000), ODR=1kHz (0110) -> 8'b000_0_0110 = 8'h06
localparam ACCEL_CFG0_VAL = 8'h06;

reg [3:0]  state_r;
reg [31:0] timer_r;
reg        spi_start_r;
reg [7:0]  spi_tx_r;
reg [4:0]  spi_nbytes_r;
wire       spi_busy_w;
wire       spi_done_w;
wire       spi_rx_valid_w;
wire [7:0] spi_rx_data_w;
wire [4:0] spi_rx_idx_w;
wire       spi_req_next_w;
reg [7:0]  spi_next_tx_r;
reg [7:0]  write_data_r;
reg [7:0]  whoami_r;

spi_master #(.CLK_DIV(CLK_DIV), .MAX_BYTES(16)) u_spi (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .start_i(spi_start_r), .tx_data_i(spi_tx_r), .n_bytes_i(spi_nbytes_r),
    .busy_o(spi_busy_w), .done_o(spi_done_w),
    .req_next_byte_o(spi_req_next_w), .next_tx_byte_i(spi_next_tx_r),
    .rx_valid_o(spi_rx_valid_w), .rx_data_o(spi_rx_data_w), .rx_byte_idx_o(spi_rx_idx_w),
    .spi_sck_o(spi_sck_o), .spi_mosi_o(spi_mosi_o),
    .spi_miso_i(spi_miso_i), .spi_cs_n_o(spi_cs_n_o)
);

// RX buffer - individual registers for Verilog-2001 compat
// 15 bytes total: 1 dummy + 2 temp + 6 accel + 6 gyro
reg [7:0] rx0,rx1,rx2,rx3,rx4,rx5,rx6,rx7,rx8,rx9,rx10,rx11,rx12,rx13,rx14;

// Next TX byte mux
always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) spi_next_tx_r <= 8'h00;
    else if (spi_req_next_w)
        spi_next_tx_r <= write_data_r;
end

// RX capture
always @(posedge clk_i) begin
    if (spi_rx_valid_w) begin
        case (spi_rx_idx_w)
            5'd0:  rx0  <= spi_rx_data_w;
            5'd1:  rx1  <= spi_rx_data_w;
            5'd2:  rx2  <= spi_rx_data_w;
            5'd3:  rx3  <= spi_rx_data_w;
            5'd4:  rx4  <= spi_rx_data_w;
            5'd5:  rx5  <= spi_rx_data_w;
            5'd6:  rx6  <= spi_rx_data_w;
            5'd7:  rx7  <= spi_rx_data_w;
            5'd8:  rx8  <= spi_rx_data_w;
            5'd9:  rx9  <= spi_rx_data_w;
            5'd10: rx10 <= spi_rx_data_w;
            5'd11: rx11 <= spi_rx_data_w;
            5'd12: rx12 <= spi_rx_data_w;
            5'd13: rx13 <= spi_rx_data_w;
            5'd14: rx14 <= spi_rx_data_w;
            default: ;
        endcase
        if ((state_r == ST_READ_WHOAMI) && (spi_rx_idx_w == 5'd1))
            whoami_r <= spi_rx_data_w;
    end
end

// Main FSM
always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        state_r <= ST_RESET; timer_r <= 0;
        spi_start_r <= 0; spi_tx_r <= 0; spi_nbytes_r <= 1; write_data_r <= 0;
        init_done_o <= 0; error_o <= 0; data_valid_o <= 0;
        accel_x_o <= 0; accel_y_o <= 0; accel_z_o <= 0;
        gyro_x_o <= 0; gyro_y_o <= 0; gyro_z_o <= 0; temp_o <= 0;
    end else begin
        spi_start_r  <= 0;
        data_valid_o <= 0;

        case (state_r)
        ST_RESET: begin
            if (timer_r >= RESET_CYCLES-1) begin timer_r<=0; state_r<=ST_BOOT_WAIT; end
            else timer_r <= timer_r+1;
        end
        ST_BOOT_WAIT: begin
            if (timer_r >= BOOT_CYCLES-1) begin timer_r<=0; state_r<=ST_WRITE_DEVCFG; end
            else timer_r <= timer_r+1;
        end
        ST_WRITE_DEVCFG: begin
            if (!spi_busy_w && !spi_start_r) begin
                write_data_r <= 8'h01;
                spi_tx_r     <= {1'b0, 7'h11};  // REG_DEVICE_CONFIG = 0x11
                spi_nbytes_r <= 5'd2;
                spi_start_r  <= 1'b1;
                timer_r      <= 0;
                state_r      <= ST_WAIT_SRESET;
            end
        end
        ST_WAIT_SRESET: begin
            if (timer_r >= BOOT_CYCLES-1) begin
                timer_r <= 0;
                if (!spi_busy_w) state_r <= ST_READ_WHOAMI;
            end else timer_r <= timer_r+1;
        end
        ST_READ_WHOAMI: begin
            if (!spi_busy_w && !spi_start_r && !spi_done_w) begin
                spi_tx_r     <= 8'hF5;  // READ | 0x75 WHO_AM_I
                spi_nbytes_r <= 5'd2;
                spi_start_r  <= 1'b1;
            end
            if (spi_done_w) begin
                if (whoami_r == 8'h47) state_r <= ST_WRITE_INTFCFG;
                else begin state_r <= ST_ERROR; error_o <= 1'b1; end
            end
        end
        ST_WRITE_INTFCFG: begin
            if (!spi_busy_w && !spi_start_r && !spi_done_w) begin
                write_data_r <= 8'h13;  // bit4=1: Big Endian sensor data
                spi_tx_r     <= {1'b0, 7'h4C};  // REG_INTF_CONFIG0 = 0x4C
                spi_nbytes_r <= 5'd2; spi_start_r <= 1'b1;
            end
            if (spi_done_w) state_r <= ST_WRITE_PWRMGMT;
        end
        ST_WRITE_PWRMGMT: begin
            if (!spi_busy_w && !spi_start_r && !spi_done_w) begin
                write_data_r <= 8'h0F;
                spi_tx_r     <= {1'b0, 7'h4E};  // REG_PWR_MGMT0 = 0x4E
                spi_nbytes_r <= 5'd2; spi_start_r <= 1'b1;
            end
            if (spi_done_w) begin timer_r<=0; state_r<=ST_WAIT_ENABLE; end
        end
        ST_WAIT_ENABLE: begin
            if (timer_r >= WAIT_EN_CYCLES-1) begin timer_r<=0; state_r<=ST_WRITE_GYROCFG; end
            else timer_r <= timer_r+1;
        end
        ST_WRITE_GYROCFG: begin
            if (!spi_busy_w && !spi_start_r && !spi_done_w) begin
                write_data_r <= GYRO_CFG0_VAL;
                spi_tx_r     <= {1'b0, 7'h4F};  // REG_GYRO_CONFIG0 = 0x4F
                spi_nbytes_r <= 5'd2; spi_start_r <= 1'b1;
            end
            if (spi_done_w) state_r <= ST_WRITE_ACCECFG;
        end
        ST_WRITE_ACCECFG: begin
            if (!spi_busy_w && !spi_start_r && !spi_done_w) begin
                write_data_r <= ACCEL_CFG0_VAL;
                spi_tx_r     <= {1'b0, 7'h50};  // REG_ACCEL_CONFIG0 = 0x50
                spi_nbytes_r <= 5'd2; spi_start_r <= 1'b1;
            end
            if (spi_done_w) begin init_done_o <= 1'b1; state_r <= ST_INIT_DONE; end
        end
        ST_INIT_DONE: begin
            timer_r <= 0; state_r <= ST_POLL_START;
        end
        ST_POLL_START: begin
            if (!spi_busy_w && !spi_start_r && !spi_done_w) begin
                spi_tx_r     <= 8'h9D;  // READ | 0x1D TEMP_DATA1
                spi_nbytes_r <= 5'd15;
                spi_start_r  <= 1'b1;
                state_r      <= ST_POLL_WAIT;
            end
        end
        ST_POLL_WAIT: begin
            if (spi_done_w) begin
                temp_o    <= $signed({rx1,  rx2});
                accel_x_o <= $signed({rx3,  rx4});
                accel_y_o <= $signed({rx5,  rx6});
                accel_z_o <= $signed({rx7,  rx8});
                gyro_x_o  <= $signed({rx9,  rx10});
                gyro_y_o  <= $signed({rx11, rx12});
                gyro_z_o  <= $signed({rx13, rx14});
                data_valid_o <= 1'b1;
                timer_r <= 0;
                state_r <= ST_POLL_DELAY;
            end
        end
        ST_POLL_DELAY: begin
            if (timer_r >= POLL_CYCLES-1) begin timer_r<=0; state_r<=ST_POLL_START; end
            else timer_r <= timer_r+1;
        end
        ST_ERROR: begin
            error_o <= 1'b1;
            if (timer_r >= ERROR_CYCLES-1) begin
                timer_r <= 0; error_o <= 0; init_done_o <= 0; state_r <= ST_RESET;
            end else timer_r <= timer_r+1;
        end
        default: state_r <= ST_RESET;
        endcase
    end
end

endmodule