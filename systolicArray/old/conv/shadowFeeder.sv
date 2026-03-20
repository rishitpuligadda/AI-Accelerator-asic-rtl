module sram_shadow_reader #(
    parameter int TILE_MAX = 4,
    parameter int REAL_K   = 4,
    parameter int REAL_M   = 9,
    parameter int REAL_N   = 1,
    parameter int AW       = 8
)(
    input  logic clk,
    input  logic rst,

    input  logic start_i,
    output logic done_o,
    output logic busy_o,

    input  logic        act_sel_i,
    input  logic [AW-1:0] sram_a_base_i,
    input  logic [5:0]  words_a_i,
    input  logic [5:0]  words_b_i,
    input  logic [$clog2(TILE_MAX+1)-1:0] eff_cols_i,

    output logic          sram_a_csb1,
    output logic [AW-1:0] sram_a_addr1,
    input  logic [31:0]   sram_a_dout1,

    output logic          sram_b_csb1,
    output logic [AW-1:0] sram_b_addr1,
    input  logic [31:0]   sram_b_dout1,

    output logic          sram_c_csb1,
    output logic [AW-1:0] sram_c_addr1,
    input  logic [31:0]   sram_c_dout1,

    output logic signed [7:0] x_rf_o [TILE_MAX][REAL_M],
    output logic signed [7:0] y_rf_o [REAL_M][TILE_MAX]
);

    typedef enum logic [1:0] {
        SR_IDLE   = 2'd0,
        SR_READ_A = 2'd1,
        SR_READ_B = 2'd2,
        SR_DONE   = 2'd3
    } sr_t;
    sr_t sr_state;

    logic [5:0] rd_cnt;

    logic        lat_act_sel;
    logic [AW-1:0] lat_sram_a_base;
    logic [5:0]  lat_words_a, lat_words_b;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_cols;

    logic signed [7:0] x_rf [TILE_MAX][REAL_M];
    logic signed [7:0] y_rf [REAL_M][TILE_MAX];

    always_ff @(posedge clk or posedge rst) begin : sr_seq
        logic [31:0] word;
        int flat, rr, cc;

        if (rst) begin
            sr_state        <= SR_IDLE;
            rd_cnt          <= '0;
            done_o          <= 1'b0;
            busy_o          <= 1'b0;
            lat_act_sel     <= 1'b0;
            lat_sram_a_base <= '0;
            lat_words_a     <= '0;
            lat_words_b     <= '0;
            lat_eff_cols    <= '0;
            for (int i=0; i<TILE_MAX; i++)
                for (int j=0; j<REAL_M; j++) x_rf[i][j] <= '0;
            for (int i=0; i<REAL_M; i++)
                for (int j=0; j<TILE_MAX; j++) y_rf[i][j] <= '0;
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
                    // Debug: print y_rf (kernel) first 3 rows
                    $display("[SR @%0t] y_rf[0][0]=%0d y_rf[0][1]=%0d y_rf[1][0]=%0d y_rf[1][1]=%0d y_rf[2][0]=%0d y_rf[2][1]=%0d",
                        $time, $signed(y_rf[0][0]), $signed(y_rf[0][1]),
                        $signed(y_rf[1][0]), $signed(y_rf[1][1]),
                        $signed(y_rf[2][0]), $signed(y_rf[2][1]));
                    $display("[SR @%0t] x_rf[0][0]=%0d x_rf[0][1]=%0d x_rf[0][2]=%0d x_rf[1][0]=%0d",
                        $time, $signed(x_rf[0][0]), $signed(x_rf[0][1]),
                        $signed(x_rf[0][2]), $signed(x_rf[1][0]));
                end

                default: sr_state <= SR_IDLE;
            endcase
        end
    end

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

    always_comb begin
        for (int i=0; i<TILE_MAX; i++)
            for (int j=0; j<REAL_M; j++)
                x_rf_o[i][j] = x_rf[i][j];
        for (int i=0; i<REAL_M; i++)
            for (int j=0; j<TILE_MAX; j++)
                y_rf_o[i][j] = y_rf[i][j];
    end

endmodule
