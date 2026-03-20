module bias_loader #(
    parameter int REAL_K    = 6,
    parameter int BIAS_BASE = 16'd0
)(
    input  logic clk,
    input  logic rst,

    // Handshake
    input  logic start_i,
    output logic done_o,
    output logic busy_o,

    // Bias DRAM (32-bit wide)
    output logic [15:0] ext_b_addr_o,
    output logic        ext_b_rd_en_o,
    input  logic [31:0] ext_b_data_i,

    // Bias register file output
    output logic signed [31:0] bias_rf_o [REAL_K]
);

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        BS_IDLE = 2'd0,
        BS_LOAD = 2'd1,
        BS_DONE = 2'd2
    } bs_t;
    bs_t bs_state;

    logic [$clog2(REAL_K+1):0] ld_cnt;
    logic signed [31:0] bias_rf [REAL_K];

    // -------------------------------------------------------------------------
    // Sequential
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            bs_state <= BS_IDLE;
            ld_cnt   <= '0;
            done_o   <= 1'b0;
            busy_o   <= 1'b0;
            for (int i = 0; i < REAL_K; i++) bias_rf[i] <= '0;
        end else begin
            done_o <= 1'b0;

            case (bs_state)

                BS_IDLE: begin
                    busy_o <= 1'b0;
                    if (start_i) begin
                        ld_cnt   <= '0;
                        busy_o   <= 1'b1;
                        bs_state <= BS_LOAD;
                    end
                end

                // ld_cnt=0        : address for bias[0] issued (comb)
                // ld_cnt=1..REAL_K: data for bias[ld_cnt-1] arrives, store it
                // ld_cnt=REAL_K   : last word stored, transition to DONE
                BS_LOAD: begin
                    if (ld_cnt >= 1 && ld_cnt <= REAL_K)
                        bias_rf[ld_cnt - 1] <= $signed(ext_b_data_i);

                    if (ld_cnt == REAL_K) begin
                        done_o   <= 1'b1;
                        busy_o   <= 1'b0;
                        bs_state <= BS_DONE;
                    end else begin
                        ld_cnt <= ld_cnt + 1'b1;
                    end
                end

                BS_DONE: begin
                    done_o   <= 1'b0;
                    bs_state <= BS_IDLE;
                end

                default: bs_state <= BS_IDLE;

            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Combinational outputs
    // -------------------------------------------------------------------------
    always_comb begin
        ext_b_rd_en_o = 1'b0;
        ext_b_addr_o  = '0;

        if (bs_state == BS_LOAD && ld_cnt < REAL_K) begin
            ext_b_rd_en_o = 1'b1;
            ext_b_addr_o  = 16'(BIAS_BASE + ld_cnt);
        end
    end

    // Expose register file
    always_comb begin
        for (int i = 0; i < REAL_K; i++)
            bias_rf_o[i] = bias_rf[i];
    end

endmodule

