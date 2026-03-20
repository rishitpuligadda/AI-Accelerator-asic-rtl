module sram_shadow_reader #(
    parameter int TILE_MAX = 4,
    parameter int REAL_K   = 6,
    parameter int REAL_M   = 6,
    parameter int REAL_N   = 6,
    parameter int AW       = 8
)(
    input  logic clk,
    input  logic rst,

    // Handshake
    input  logic start_i,
    output logic done_o,
    output logic busy_o,

    // Tile geometry (stable from start_i to done_o)
    input  logic        act_sel_i,      // 0=activations in SRAM A, 1=SRAM C
    input  logic [AW-1:0] sram_a_base_i,
    input  logic [5:0]  words_a_i,
    input  logic [5:0]  words_b_i,
    input  logic [$clog2(TILE_MAX+1)-1:0] eff_cols_i,

    // SRAM A port 1 (read activations when act_sel=0)
    output logic          sram_a_csb1,
    output logic [AW-1:0] sram_a_addr1,
    input  logic [31:0]   sram_a_dout1,

    // SRAM B port 1 (read weights)
    output logic          sram_b_csb1,
    output logic [AW-1:0] sram_b_addr1,
    input  logic [31:0]   sram_b_dout1,

    // SRAM C port 1 (read activations when act_sel=1)
    output logic          sram_c_csb1,
    output logic [AW-1:0] sram_c_addr1,
    input  logic [31:0]   sram_c_dout1,

    // Register file outputs
    output logic signed [7:0] x_rf_o [TILE_MAX][REAL_M],
    output logic signed [7:0] y_rf_o [REAL_M][TILE_MAX]
);

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        SR_IDLE    = 2'd0,
        SR_READ_A  = 2'd1,
        SR_READ_B  = 2'd2,
        SR_DONE    = 2'd3
    } sr_t;
    sr_t sr_state;

    logic [5:0] rd_cnt;

    // Latched geometry
    logic        lat_act_sel;
    logic [AW-1:0] lat_sram_a_base;
    logic [5:0]  lat_words_a, lat_words_b;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_cols;

    // Register files
    logic signed [7:0] x_rf [TILE_MAX][REAL_M];
    logic signed [7:0] y_rf [REAL_M][TILE_MAX];

    // -------------------------------------------------------------------------
    // Sequential
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin : sr_seq
        logic [31:0] word;
        int flat, rr, cc;

        if (rst) begin
            sr_state       <= SR_IDLE;
            rd_cnt         <= '0;
            done_o         <= 1'b0;
            busy_o         <= 1'b0;
            lat_act_sel    <= 1'b0;
            lat_sram_a_base <= '0;
            lat_words_a    <= '0;
            lat_words_b    <= '0;
            lat_eff_cols   <= '0;
            for (int i=0;i<TILE_MAX;i++)
                for (int j=0;j<REAL_M;j++) x_rf[i][j] <= '0;
            for (int i=0;i<REAL_M;i++)
                for (int j=0;j<TILE_MAX;j++) y_rf[i][j] <= '0;
        end else begin
            done_o <= 1'b0;

            case (sr_state)

                SR_IDLE: begin
                    busy_o <= 1'b0;
                    if (start_i) begin
                        lat_act_sel     <= act_sel_i;
                        lat_sram_a_base <= sram_a_base_i;
                        lat_words_a     <= words_a_i;
                        lat_words_b     <= words_b_i;
                        lat_eff_cols    <= eff_cols_i;
                        rd_cnt          <= '0;
                        busy_o          <= 1'b1;
                        sr_state        <= SR_READ_A;
                    end
                end

                // -------------------------------------------------------------
                // SR_READ_A: read words_a words from SRAM A (or C) into x_rf
                // rd_cnt=0      : address for word 0 issued (comb)
                // rd_cnt=1..N   : data for word rd_cnt-1 arrives, unpack into x_rf
                // rd_cnt=words_a: last word unpacked, move to SR_READ_B
                // -------------------------------------------------------------
                SR_READ_A: begin
                    if (rd_cnt >= 1) begin
                        word = lat_act_sel ? sram_c_dout1 : sram_a_dout1;
                        for (int lane = 0; lane < 4; lane++) begin
                            flat = (int'(rd_cnt) - 1) * 4 + lane;
                            rr   = flat / REAL_M;
                            cc   = flat % REAL_M;
                            if (rr < TILE_MAX && cc < REAL_M)
                                x_rf[rr][cc] <= $signed(word[lane*8 +: 8]);
                        end
                    end
                    if (rd_cnt == lat_words_a) begin
                        rd_cnt   <= '0;
                        sr_state <= SR_READ_B;
                    end else begin
                        rd_cnt <= rd_cnt + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // SR_READ_B: same structure, SRAM B into y_rf
                // y_rf layout: y_rf[row][local_col]
                //   flat byte index = (rd_cnt-1)*4 + lane
                //   row      = flat / eff_cols
                //   local_col = flat % eff_cols
                // -------------------------------------------------------------
                SR_READ_B: begin
                    if (rd_cnt >= 1) begin
                        word = sram_b_dout1;
                        for (int lane = 0; lane < 4; lane++) begin
                            flat = (int'(rd_cnt) - 1) * 4 + lane;
                            rr   = flat / int'(lat_eff_cols);
                            cc   = flat % int'(lat_eff_cols);
                            if (rr < REAL_M && cc < TILE_MAX)
                                y_rf[rr][cc] <= $signed(word[lane*8 +: 8]);
                        end
                    end
                    if (rd_cnt == lat_words_b) begin
                        done_o   <= 1'b1;
                        busy_o   <= 1'b0;
                        sr_state <= SR_DONE;
                    end else begin
                        rd_cnt <= rd_cnt + 1'b1;
                    end
                end

                SR_DONE: begin
                    done_o   <= 1'b0;
                    sr_state <= SR_IDLE;
                end

                default: sr_state <= SR_IDLE;

            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Combinational: SRAM port 1 address generation
    // -------------------------------------------------------------------------
    always_comb begin
        sram_a_csb1 = 1'b1; sram_a_addr1 = '0;
        sram_b_csb1 = 1'b1; sram_b_addr1 = '0;
        sram_c_csb1 = 1'b1; sram_c_addr1 = '0;

        case (sr_state)
            SR_READ_A: begin
                if (rd_cnt < lat_words_a) begin
                    if (!lat_act_sel) begin
                        sram_a_csb1  = 1'b0;
                        sram_a_addr1 = AW'(lat_sram_a_base + rd_cnt);
                    end else begin
                        sram_c_csb1  = 1'b0;
                        sram_c_addr1 = AW'(lat_sram_a_base + rd_cnt);
                    end
                end
            end
            SR_READ_B: begin
                if (rd_cnt < lat_words_b) begin
                    sram_b_csb1  = 1'b0;
                    sram_b_addr1 = AW'(rd_cnt);
                end
            end
            default: ;
        endcase
    end

    // Expose register files
    always_comb begin
        for (int i=0;i<TILE_MAX;i++)
            for (int j=0;j<REAL_M;j++)
                x_rf_o[i][j] = x_rf[i][j];
        for (int i=0;i<REAL_M;i++)
            for (int j=0;j<TILE_MAX;j++)
                y_rf_o[i][j] = y_rf[i][j];
    end

endmodule

