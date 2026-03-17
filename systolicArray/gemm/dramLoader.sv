module dram_loader #(
    parameter int TILE_MAX  = 4,
    parameter int REAL_K    = 6,
    parameter int REAL_M    = 6,
    parameter int REAL_N    = 6,
    parameter int AW        = 8
)(
    input  logic clk,
    input  logic rst,

    // Handshake
    input  logic        start_i,
    input  logic        skip_x_i,
    output logic        load_ready_o,
    output logic        busy_o,

    // Tile geometry (stable from start_i to load_ready_o)
    input  logic [$clog2(REAL_K):0]        row_tile_i,
    input  logic [$clog2(REAL_N):0]        col_tile_i,
    input  logic [$clog2(TILE_MAX+1)-1:0]  eff_rows_i,
    input  logic [$clog2(TILE_MAX+1)-1:0]  eff_cols_i,
    input  logic [5:0]   words_a_i,
    input  logic [5:0]   words_b_i,
    input  logic [AW-1:0] sram_a_base_i,

    // X DRAM (32-bit wide)
    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    // Y DRAM (32-bit wide)
    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    // SRAM A port 0
    output logic          sram_a_csb0,
    output logic          sram_a_web0,
    output logic [3:0]    sram_a_wmask0,
    output logic [AW-1:0] sram_a_addr0,
    output logic [31:0]   sram_a_din0,

    // SRAM B port 0
    output logic          sram_b_csb0,
    output logic          sram_b_web0,
    output logic [3:0]    sram_b_wmask0,
    output logic [AW-1:0] sram_b_addr0,
    output logic [31:0]   sram_b_din0
);

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        LS_IDLE   = 3'd0,
        LS_FILL_A = 3'd1,
        LS_FILL_B = 3'd2,
        LS_READY  = 3'd3
    } ls_t;
    ls_t ls_state;

    logic [5:0] ld_cnt;
    logic [$clog2(REAL_K):0]       lat_row_tile;
    logic [$clog2(REAL_N):0]       lat_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_rows;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_cols;
    logic [5:0]    lat_words_a, lat_words_b;
    logic [AW-1:0] lat_sram_a_base;

    // -------------------------------------------------------------------------
    // Sequential FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ls_state        <= LS_IDLE;
            ld_cnt          <= '0;
            lat_row_tile    <= '0;
            lat_col_tile    <= '0;
            lat_eff_rows    <= '0;
            lat_eff_cols    <= '0;
            lat_words_a     <= '0;
            lat_words_b     <= '0;
            lat_sram_a_base <= '0;
            load_ready_o    <= 1'b0;
            busy_o          <= 1'b0;
        end else begin
            load_ready_o <= 1'b0;

            case (ls_state)

                LS_IDLE: begin
                    busy_o <= 1'b0;
                    if (start_i) begin
                        lat_row_tile    <= row_tile_i;
                        lat_col_tile    <= col_tile_i;
                        lat_eff_rows    <= eff_rows_i;
                        lat_eff_cols    <= eff_cols_i;
                        lat_words_a     <= words_a_i;
                        lat_words_b     <= words_b_i;
                        lat_sram_a_base <= sram_a_base_i;
                        ld_cnt          <= '0;
                        busy_o          <= 1'b1;
                        ls_state        <= skip_x_i ? LS_FILL_B : LS_FILL_A;
                    end
                end

                LS_FILL_A: begin
                    if (ld_cnt == lat_words_a) begin
                        ld_cnt   <= '0;
                        ls_state <= LS_FILL_B;
                    end else begin
                        ld_cnt <= ld_cnt + 1'b1;
                    end
                end

                LS_FILL_B: begin
                    if (ld_cnt == lat_words_b) begin
                        ld_cnt       <= '0;
                        load_ready_o <= 1'b1;
                        ls_state     <= LS_READY;
                    end else begin
                        ld_cnt <= ld_cnt + 1'b1;
                    end
                end

                LS_READY: begin
                    busy_o   <= 1'b0;
                    ls_state <= LS_IDLE;
                end

                default: ls_state <= LS_IDLE;

            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Combinational outputs
    // -------------------------------------------------------------------------
    always_comb begin
        ext_x_rd_en_o = 1'b0; ext_x_addr_o = '0;
        ext_y_rd_en_o = 1'b0; ext_y_addr_o = '0;
        sram_a_csb0 = 1'b1; sram_a_web0 = 1'b1;
        sram_a_wmask0 = 4'hF; sram_a_addr0 = '0; sram_a_din0 = '0;
        sram_b_csb0 = 1'b1; sram_b_web0 = 1'b1;
        sram_b_wmask0 = 4'hF; sram_b_addr0 = '0; sram_b_din0 = '0;

        case (ls_state)
            LS_FILL_A: begin
                // Issue read address for the next word
                if (ld_cnt < lat_words_a) begin
                    automatic int flat_byte  = int'(ld_cnt) * 4;
                    automatic int local_row  = flat_byte / REAL_M;
                    automatic int local_col  = flat_byte % REAL_M;
                    automatic int global_row = int'(lat_row_tile) + local_row;
                    ext_x_rd_en_o = 1'b1;
                    ext_x_addr_o  = 16'((global_row * REAL_M + local_col) / 4);
                end
                // Write the word that arrived last cycle
                if (ld_cnt >= 1) begin
                    sram_a_csb0   = 1'b0;
                    sram_a_web0   = 1'b0;
                    sram_a_wmask0 = 4'hF;
                    sram_a_addr0  = AW'(lat_sram_a_base + ld_cnt - 1);
                    sram_a_din0   = ext_x_data_i;
                end
            end

            LS_FILL_B: begin
                // Y is stored in DRAM in tile-packed format:
                // for each col-tile, REAL_M rows x eff_cols bytes packed sequentially.
                // y_col_base = col_tile * REAL_M / 4  (words before this col-tile's strip)
                // Fetch word: y_col_base + ld_cnt
                if (ld_cnt < lat_words_b) begin
                    automatic int y_col_base = (int'(lat_col_tile) * REAL_M) / 4;
                    ext_y_rd_en_o = 1'b1;
                    ext_y_addr_o  = 16'(y_col_base + int'(ld_cnt));
                end
                if (ld_cnt >= 1) begin
                    sram_b_csb0   = 1'b0;
                    sram_b_web0   = 1'b0;
                    sram_b_wmask0 = 4'hF;
                    sram_b_addr0  = AW'(ld_cnt - 1);
                    sram_b_din0   = ext_y_data_i;
                end
            end

            default: ;
        endcase
    end

endmodule
