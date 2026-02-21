// =============================================================================
// spi_master.v
// Generic SPI Master - Mode 0 (CPOL=0, CPHA=0)
// 
// Supports configurable clock divider and 8-bit transfers.
// CS is asserted low for the entire multi-byte transaction.
//
// Interface:
//   start_i      - pulse high for 1 clk to begin transaction
//   tx_data_i    - byte to send (latched at start)
//   n_bytes_i    - total bytes in transaction (1..255)
//   busy_o       - high while transaction in progress
//   done_o       - 1-cycle pulse when transaction complete
//   rx_data_o    - received byte (valid for 1 cycle after done_o)
//   rx_valid_o   - byte received (pulses each time a byte is received)
//   rx_byte_idx_o- index of which byte was just received
// =============================================================================

module spi_master #(
    parameter CLK_DIV   = 10,   // SPI_CLK = SYS_CLK / (2 * CLK_DIV)
    parameter MAX_BYTES = 16    // max bytes per transaction
)(
    input  wire        clk_i,
    input  wire        rst_ni,

    // Control interface
    input  wire        start_i,
    input  wire [7:0]  tx_data_i,          // first byte; further bytes via next_byte
    input  wire [$clog2(MAX_BYTES):0] n_bytes_i,
    output wire        busy_o,
    output reg         done_o,

    // Byte-level streaming (for multi-byte transfers)
    output reg         req_next_byte_o,    // request next TX byte
    input  wire [7:0]  next_tx_byte_i,     // next TX byte (must be valid when req_next_byte_o=1)
    output reg         rx_valid_o,         // byte received
    output reg  [7:0]  rx_data_o,          // received byte
    output reg  [$clog2(MAX_BYTES):0] rx_byte_idx_o,  // which byte index received

    // SPI pins
    output reg         spi_sck_o,
    output wire        spi_mosi_o,
    input  wire        spi_miso_i,
    output reg         spi_cs_n_o
);

// ---------- State machine ----------
localparam IDLE     = 3'd0;
localparam ASSERT_CS= 3'd1;
localparam TRANSFER = 3'd2;
localparam HOLD_CS  = 3'd3;
localparam DEASSERT = 3'd4;

reg [2:0]  state_r;
reg [$clog2(CLK_DIV+1)-1:0] clk_cnt_r;
reg [3:0]  bit_cnt_r;        // 0..7
reg [$clog2(MAX_BYTES):0] byte_cnt_r;
reg [$clog2(MAX_BYTES):0] n_bytes_r;
reg [7:0]  tx_shift_r;
reg [7:0]  rx_shift_r;
reg        load_next_r;  // flag: next TX byte ready to load on falling edge

assign busy_o     = (state_r != IDLE);
assign spi_mosi_o = tx_shift_r[7];

always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        state_r        <= IDLE;
        spi_sck_o      <= 1'b0;
        spi_cs_n_o     <= 1'b1;
        clk_cnt_r      <= 0;
        bit_cnt_r      <= 0;
        byte_cnt_r     <= 0;
        n_bytes_r      <= 0;
        tx_shift_r     <= 8'h0;
        rx_shift_r     <= 8'h0;
        load_next_r    <= 1'b0;
        done_o         <= 1'b0;
        rx_valid_o     <= 1'b0;
        rx_data_o      <= 8'h0;
        rx_byte_idx_o  <= 0;
        req_next_byte_o<= 1'b0;
    end else begin
        done_o          <= 1'b0;
        rx_valid_o      <= 1'b0;
        req_next_byte_o <= 1'b0;

        case (state_r)
        IDLE: begin
            spi_sck_o  <= 1'b0;
            spi_cs_n_o <= 1'b1;
            clk_cnt_r  <= 0;
            bit_cnt_r  <= 0;
            byte_cnt_r <= 0;
            load_next_r<= 1'b0;
            if (start_i) begin
                tx_shift_r <= tx_data_i;
                n_bytes_r  <= n_bytes_i;
                state_r    <= ASSERT_CS;
            end
        end

        ASSERT_CS: begin
            spi_cs_n_o <= 1'b0;  // assert CS
            spi_sck_o  <= 1'b0;
            clk_cnt_r  <= 0;
            state_r    <= TRANSFER;
        end

        TRANSFER: begin
            if (clk_cnt_r == CLK_DIV - 1) begin
                clk_cnt_r <= 0;
                spi_sck_o <= ~spi_sck_o;  // toggle clock

                if (!spi_sck_o) begin
                    // Rising edge: sample MISO
                    rx_shift_r <= {rx_shift_r[6:0], spi_miso_i};

                    if (bit_cnt_r == 7) begin
                        // Byte complete
                        bit_cnt_r  <= 0;
                        byte_cnt_r <= byte_cnt_r + 1;

                        // Output received byte
                        rx_valid_o    <= 1'b1;
                        rx_data_o     <= {rx_shift_r[6:0], spi_miso_i};
                        rx_byte_idx_o <= byte_cnt_r;

                        if (byte_cnt_r == n_bytes_r - 1) begin
                            // Last byte done
                            state_r <= DEASSERT;
                        end else begin
                            // Request next TX byte and flag for falling edge
                            req_next_byte_o <= 1'b1;
                            load_next_r     <= 1'b1;
                        end
                    end else begin
                        bit_cnt_r <= bit_cnt_r + 1;
                    end
                end else begin
                    // Falling edge: shift out MOSI
                    if (bit_cnt_r == 0 && load_next_r) begin
                        tx_shift_r  <= next_tx_byte_i;
                        load_next_r <= 1'b0;
                    end else begin
                        tx_shift_r <= {tx_shift_r[6:0], 1'b0};
                    end
                end
            end else begin
                clk_cnt_r <= clk_cnt_r + 1;
            end
        end

        DEASSERT: begin
            spi_cs_n_o <= 1'b1;
            spi_sck_o  <= 1'b0;
            done_o     <= 1'b1;
            state_r    <= IDLE;
        end

        default: state_r <= IDLE;
        endcase
    end
end

endmodule
