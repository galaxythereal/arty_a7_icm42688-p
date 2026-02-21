// =============================================================================
// uart_tx.v
// Simple 8N1 UART Transmitter
// =============================================================================
module uart_tx #(
    parameter CLK_HZ    = 100_000_000,
    parameter BAUD_RATE = 115_200,
    parameter CLK_DIV   = CLK_HZ / BAUD_RATE
)(
    input  wire       clk_i,
    input  wire       rst_ni,
    input  wire       start_i,    // pulse to begin transmission
    input  wire [7:0] data_i,     // byte to send
    output reg        tx_o,       // UART TX pin
    output reg        busy_o      // high while transmitting
);

localparam IDLE  = 2'd0;
localparam START = 2'd1;
localparam DATA  = 2'd2;
localparam STOP  = 2'd3;

reg [1:0]  state_r;
reg [15:0] baud_cnt_r;
reg [3:0]  bit_cnt_r;
reg [7:0]  shift_r;

always @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
        state_r    <= IDLE;
        tx_o       <= 1'b1;
        busy_o     <= 1'b0;
        baud_cnt_r <= 0;
        bit_cnt_r  <= 0;
        shift_r    <= 8'h0;
    end else begin
        case (state_r)
        IDLE: begin
            tx_o   <= 1'b1;
            busy_o <= 1'b0;
            if (start_i) begin
                shift_r    <= data_i;
                baud_cnt_r <= 0;
                state_r    <= START;
                busy_o     <= 1'b1;
            end
        end
        START: begin
            tx_o <= 1'b0;  // start bit
            if (baud_cnt_r >= CLK_DIV - 1) begin
                baud_cnt_r <= 0;
                bit_cnt_r  <= 0;
                state_r    <= DATA;
            end else begin
                baud_cnt_r <= baud_cnt_r + 1;
            end
        end
        DATA: begin
            tx_o <= shift_r[0];
            if (baud_cnt_r >= CLK_DIV - 1) begin
                baud_cnt_r <= 0;
                shift_r    <= {1'b0, shift_r[7:1]};
                if (bit_cnt_r == 7) begin
                    state_r <= STOP;
                end else begin
                    bit_cnt_r <= bit_cnt_r + 1;
                end
            end else begin
                baud_cnt_r <= baud_cnt_r + 1;
            end
        end
        STOP: begin
            tx_o <= 1'b1;  // stop bit
            if (baud_cnt_r >= CLK_DIV - 1) begin
                baud_cnt_r <= 0;
                state_r    <= IDLE;
                busy_o     <= 1'b0;
            end else begin
                baud_cnt_r <= baud_cnt_r + 1;
            end
        end
        endcase
    end
end

endmodule
