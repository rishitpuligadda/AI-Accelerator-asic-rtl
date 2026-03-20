// =============================================================================
// compute_engine.sv  (v2 - optimised)
//
// Optimisations vs v1:
//
// O1. Parallel SRAM read: MS_READ_A and MS_READ_B merged into MS_READ_AB.
//     SRAM A (activations) and SRAM B (weights) have independent port-1 buses,
//     so both can be drained simultaneously. rd_a_cnt and rd_b_cnt advance
//     independently; state exits when both reach their word counts.
//     Saves min(words_a, words_b) + 1 cycles per tile.
//     Example (words_a=9, words_b=9): 20 cycles -> 10 cycles.
//
// O2. dl_start_o fired immediately on LOAD_DRAM entry.
//     Old code used a state_entry_d1 register to generate a delayed pulse,
//     costing 1 extra cycle per tile. Fix: dl_start_o is asserted in the same
//     cycle as the transition INTO MS_LOAD_DRAM (from MS_LOAD_BIAS or
//     MS_ADVANCE_CHK). state_entry and state_entry_d1 registers removed.
//     Saves 1 cycle per tile pass.
//
// O3. MS_ADVANCE_CHK2 removed.
//     The tile iterator updates at posedge of MS_ADVANCE. By MS_ADVANCE_CHK
//     (the next cycle) ti_all_done is already stable - no second wait needed.
//     Saves 1 cycle per tile pass.
//
// O4. x_rf / y_rf reset removed from always_ff.
//     These arrays are fully overwritten in MS_READ_AB before any read in
//     MS_RUN. Resetting them on rst wastes synthesis area for no benefit.
//
// Combined saving: ~12 cycles per tile (O1=10, O2=1, O3=1).
//
// Master FSM (12 states, was 14):
//   MS_RESET       - fire bias_loader start
//   MS_LOAD_BIAS   - wait for bias_loader done, fire dl_start_o on exit
//   MS_LOAD_DRAM   - wait for dram_loader load_ready
//   MS_READ_AB     - drain SRAM A->x_rf AND SRAM B->y_rf in parallel
//   MS_RESET_ARRAY - pulse local reset to systolic array (1 cycle)
//   MS_DEASSERT    - drop local reset (1 cycle)
//   MS_RUN         - skew-feed data, accumulate, count to capture_t
//   MS_WRITE       - pack and write output bytes row by row
//   MS_OVERFLOW    - write overflow bytes into next word
//   MS_ADVANCE     - advance tile iterator
//   MS_ADVANCE_CHK - wait 1 cycle, check ti_all_done -> loop or finish
//   MS_DONE        - assert done output
// =============================================================================

module compute_engine #(
    parameter int TILE_MAX  = 4,
    parameter int REAL_K    = 4,
    parameter int REAL_M    = 9,
    parameter int REAL_N    = 1,
    parameter int AW        = 8,
    parameter int BIAS_BASE = 0
)(
    input  logic clk,
    input  logic rst,

    output logic done,

    input  logic is_last_layer,
    input  logic act_sel,
    input  logic out_sel,
    input  logic skip_x,

    // Bias loader interface
    output logic        bl_start_o,
    input  logic        bl_done_i,
    input  logic signed [31:0] bias_rf_i [REAL_K],

    // DRAM loader interface
    output logic        dl_start_o,
    input  logic        dl_load_ready_i,

    output logic [$clog2(REAL_K):0]        dl_row_tile_o,
    output logic [$clog2(REAL_N):0]        dl_col_tile_o,
    output logic [$clog2(TILE_MAX+1)-1:0]  dl_eff_rows_o,
    output logic [$clog2(TILE_MAX+1)-1:0]  dl_eff_cols_o,
    output logic [5:0]                     dl_words_a_o,
    output logic [5:0]                     dl_words_b_o,
    output logic [AW-1:0]                  dl_sram_a_base_o,

    // SRAM port 0 - shared with dram_loader (muxed here)
    input  logic          dl_sram_a_csb0, dl_sram_a_web0,
    input  logic [3:0]    dl_sram_a_wmask0,
    input  logic [AW-1:0] dl_sram_a_addr0,
    input  logic [31:0]   dl_sram_a_din0,

    input  logic          dl_sram_b_csb0, dl_sram_b_web0,
    input  logic [3:0]    dl_sram_b_wmask0,
    input  logic [AW-1:0] dl_sram_b_addr0,
    input  logic [31:0]   dl_sram_b_din0,

    output logic          sram_a_csb0, sram_a_web0,
    output logic [3:0]    sram_a_wmask0,
    output logic [AW-1:0] sram_a_addr0,
    output logic [31:0]   sram_a_din0,

    output logic          sram_b_csb0, sram_b_web0,
    output logic [3:0]    sram_b_wmask0,
    output logic [AW-1:0] sram_b_addr0,
    output logic [31:0]   sram_b_din0,

    output logic          sram_c_csb0, sram_c_web0,
    output logic [3:0]    sram_c_wmask0,
    output logic [AW-1:0] sram_c_addr0,
    output logic [31:0]   sram_c_din0,

    // SRAM port 1 - read only
    output logic          sram_a_csb1,
    output logic [AW-1:0] sram_a_addr1,
    input  logic [31:0]   sram_a_dout1,

    output logic          sram_b_csb1,
    output logic [AW-1:0] sram_b_addr1,
    input  logic [31:0]   sram_b_dout1,

    output logic          sram_c_csb1,
    output logic [AW-1:0] sram_c_addr1,
    input  logic [31:0]   sram_c_dout1,

    // DRAM Z output
    output logic [15:0] ext_z_addr_o,
    output logic        ext_z_wr_en_o,
    output logic [31:0] ext_z_data_o,
    output logic [3:0]  ext_z_wmask_o
);

    // =========================================================================
    // Master FSM states
    // =========================================================================
    typedef enum logic [3:0] {
        MS_RESET       = 4'd0,
        MS_LOAD_BIAS   = 4'd1,
        MS_LOAD_DRAM   = 4'd2,
        MS_READ_AB     = 4'd3,
        MS_RESET_ARRAY = 4'd4,
        MS_DEASSERT    = 4'd5,
        MS_RUN         = 4'd6,
        MS_WRITE       = 4'd7,
        MS_OVERFLOW    = 4'd8,
        MS_ADVANCE     = 4'd9,
        MS_ADVANCE_CHK = 4'd10,
        MS_DONE        = 4'd11
    } ms_t;
    ms_t ms_state;

    // =========================================================================
    // Tile iterator
    // =========================================================================
    logic [$clog2(REAL_K):0]       ti_row_tile;
    logic [$clog2(REAL_N):0]       ti_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0] ti_eff_rows;
    logic [$clog2(TILE_MAX+1)-1:0] ti_eff_cols;
    logic                          ti_all_done;

    function automatic int imin(input int a, input int b);
        return (a < b) ? a : b;
    endfunction

    function automatic int words_of(input int elements);
        return (elements + 3) / 4;
    endfunction

    logic [$clog2(TILE_MAX+1)-1:0] ti_eff_rows_c, ti_eff_cols_c;
    always_comb begin
        ti_eff_rows_c = $bits(ti_eff_rows)'(imin(REAL_K - int'(ti_row_tile), TILE_MAX));
        ti_eff_cols_c = $bits(ti_eff_cols)'(imin(REAL_N - int'(ti_col_tile), TILE_MAX));
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ti_row_tile <= '0;
            ti_col_tile <= '0;
            ti_eff_rows <= '0;
            ti_eff_cols <= '0;
            ti_all_done <= 1'b0;
        end else begin
            if (ms_state == MS_RESET) begin
                ti_row_tile <= '0;
                ti_col_tile <= '0;
                ti_eff_rows <= $bits(ti_eff_rows)'(imin(REAL_K, TILE_MAX));
                ti_eff_cols <= $bits(ti_eff_cols)'(imin(REAL_N, TILE_MAX));
                ti_all_done <= 1'b0;
            end else if (ms_state == MS_ADVANCE) begin
                if (int'(ti_col_tile) + int'(ti_eff_cols_c) < REAL_N) begin
                    ti_col_tile <= $bits(ti_col_tile)'(int'(ti_col_tile) + int'(ti_eff_cols_c));
                    ti_eff_cols <= $bits(ti_eff_cols)'(imin(REAL_N - int'(ti_col_tile) - int'(ti_eff_cols_c), TILE_MAX));
                end else if (int'(ti_row_tile) + int'(ti_eff_rows_c) < REAL_K) begin
                    ti_row_tile <= $bits(ti_row_tile)'(int'(ti_row_tile) + int'(ti_eff_rows_c));
                    ti_col_tile <= '0;
                    ti_eff_rows <= $bits(ti_eff_rows)'(imin(REAL_K - int'(ti_row_tile) - int'(ti_eff_rows_c), TILE_MAX));
                    ti_eff_cols <= $bits(ti_eff_cols)'(imin(REAL_N, TILE_MAX));
                end else begin
                    ti_all_done <= 1'b1;
                end
            end
        end
    end

    // Wire tile geometry to dram_loader
    assign dl_row_tile_o    = ti_row_tile;
    assign dl_col_tile_o    = ti_col_tile;
    assign dl_eff_rows_o    = ti_eff_rows;
    assign dl_eff_cols_o    = ti_eff_cols;
    assign dl_words_a_o     = 6'(words_of(int'(ti_eff_rows) * REAL_M));
    assign dl_words_b_o     = 6'(words_of(REAL_M * int'(ti_eff_cols)));
    assign dl_sram_a_base_o = AW'((int'(ti_row_tile) * REAL_M + 3) / 4);  // ceiling division

    // =========================================================================
    // Latched tile geometry for current compute pass
    // =========================================================================
    logic [$clog2(REAL_K):0]        lat_row_tile;
    logic [$clog2(REAL_N):0]        lat_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0]  lat_eff_rows;
    logic [$clog2(TILE_MAX+1)-1:0]  lat_eff_cols;
    logic [5:0]                     lat_words_a;
    logic [5:0]                     lat_words_b;
    logic [AW-1:0]                  lat_sram_a_base;
    logic                           lat_act_sel;

    // =========================================================================
    // Register files (no reset - fully overwritten in MS_READ_AB before use)
    // =========================================================================
    logic signed [7:0] x_rf [TILE_MAX][REAL_M];
    logic signed [7:0] y_rf [REAL_M][TILE_MAX];

    // =========================================================================
    // Systolic array feeds and results
    // =========================================================================
    logic signed  [7:0] a_feed [TILE_MAX];
    logic signed  [7:0] b_feed [TILE_MAX];
    logic signed [31:0] tile_z [TILE_MAX][TILE_MAX];
    logic               local_rst;

    // =========================================================================
    // Counters
    // =========================================================================
    logic [5:0] rd_a_cnt;
    logic [5:0] rd_b_cnt;
    logic       rd_a_done;   // set when rd_a_cnt reaches lat_words_a
    logic       rd_b_done;   // set when rd_b_cnt reaches lat_words_b
    logic [7:0] run_cnt;
    logic [7:0] capture_t;

    // =========================================================================
    // Write phase state
    // =========================================================================
    logic [$clog2(TILE_MAX):0] cur_row;
    logic [15:0]               row_start;
    logic [AW-1:0]             word_addr;
    logic [31:0]               overflow_word;
    logic [3:0]                overflow_mask;
    logic [AW-1:0]             overflow_addr;
    logic                      has_overflow;

    // =========================================================================
    // Output bytes - raw 8-bit truncation of accumulator
    // =========================================================================
    logic [7:0] out_byte [TILE_MAX];
    always_comb begin
        for (int j = 0; j < TILE_MAX; j++)
            out_byte[j] = (j < int'(lat_eff_cols))
                          ? tile_z[cur_row][j][7:0]
                          : 8'h00;
    end

    // =========================================================================
    // Master sequential FSM
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin : ms_seq
        logic [31:0] word;
        int flat, rr, cc;

        if (rst) begin
            ms_state        <= MS_RESET;
            done            <= 1'b0;
            bl_start_o      <= 1'b0;
            dl_start_o      <= 1'b0;
            rd_a_cnt        <= '0;
            rd_b_cnt        <= '0;
            rd_a_done       <= 1'b0;
            rd_b_done       <= 1'b0;
            run_cnt         <= '0;
            capture_t       <= '0;
            cur_row         <= '0;
            row_start       <= '0;
            word_addr       <= '0;
            overflow_word   <= '0;
            overflow_mask   <= '0;
            overflow_addr   <= '0;
            has_overflow    <= 1'b0;
            local_rst       <= 1'b1;
            lat_row_tile    <= '0;
            lat_col_tile    <= '0;
            lat_eff_rows    <= '0;
            lat_eff_cols    <= '0;
            lat_words_a     <= '0;
            lat_words_b     <= '0;
            lat_sram_a_base <= '0;
            lat_act_sel     <= 1'b0;
        end else begin
            bl_start_o <= 1'b0;
            dl_start_o <= 1'b0;

            case (ms_state)

                // -------------------------------------------------------------
                MS_RESET: begin
                    bl_start_o <= 1'b1;
                    ms_state   <= MS_LOAD_BIAS;
                end

                // -------------------------------------------------------------
                // Wait for bias done; fire dram_loader in same cycle (O2)
                // -------------------------------------------------------------
                MS_LOAD_BIAS: begin
                    if (bl_done_i) begin
                        dl_start_o <= 1'b1;
                        ms_state   <= MS_LOAD_DRAM;
                    end
                end

                // -------------------------------------------------------------
                MS_LOAD_DRAM: begin
                    if (dl_load_ready_i) begin
                        lat_row_tile    <= ti_row_tile;
                        lat_col_tile    <= ti_col_tile;
                        lat_eff_rows    <= ti_eff_rows;
                        lat_eff_cols    <= ti_eff_cols;
                        lat_words_a     <= dl_words_a_o;
                        lat_words_b     <= dl_words_b_o;
                        lat_sram_a_base <= dl_sram_a_base_o;
                        lat_act_sel     <= act_sel;
                        capture_t       <= 8'(REAL_M + int'(ti_eff_rows)
                                              + int'(ti_eff_cols) - 2);
                        rd_a_cnt        <= '0;
                        rd_b_cnt        <= '0;
                        rd_a_done       <= 1'b0;
                        rd_b_done       <= 1'b0;
                        ms_state        <= MS_READ_AB;
                    end
                end

                // -------------------------------------------------------------
                // Drain SRAM A->x_rf AND SRAM B->y_rf simultaneously (O1)
                // Both counters advance independently; exit when both done.
                // -------------------------------------------------------------
                MS_READ_AB: begin
                    // ---- Pipeline A: receive + advance, guarded by rd_a_done ----
                    // While not done: receive word[cnt-1] if cnt>=1, then
                    // either mark done (cnt==words_a) or advance counter.
                    // Once done: hold silently. Exit when both done.
                    if (!rd_a_done) begin
                        if (rd_a_cnt >= 1) begin
                            word = lat_act_sel ? sram_c_dout1 : sram_a_dout1;
                            for (int lane = 0; lane < 4; lane++) begin
                                flat = (int'(rd_a_cnt) - 1) * 4 + lane;
                                rr   = flat / REAL_M;
                                cc   = flat % REAL_M;
                                if (rr < TILE_MAX && cc < REAL_M)
                                    x_rf[rr][cc] <= $signed(word[lane*8 +: 8]);
                            end
                        end
                        if (rd_a_cnt == lat_words_a)
                            rd_a_done <= 1'b1;
                        else
                            rd_a_cnt <= rd_a_cnt + 1'b1;
                    end

                    // ---- Pipeline B: receive + advance, guarded by rd_b_done ----
                    if (!rd_b_done) begin
                        if (rd_b_cnt >= 1) begin
                            word = sram_b_dout1;
                            for (int lane = 0; lane < 4; lane++) begin
                                flat = (int'(rd_b_cnt) - 1) * 4 + lane;
                                rr   = flat / int'(lat_eff_cols);
                                cc   = flat % int'(lat_eff_cols);
                                if (rr < REAL_M && cc < TILE_MAX)
                                    y_rf[rr][cc] <= $signed(word[lane*8 +: 8]);
                            end
                        end
                        if (rd_b_cnt == lat_words_b)
                            rd_b_done <= 1'b1;
                        else
                            rd_b_cnt <= rd_b_cnt + 1'b1;
                    end

                    // ---- Exit when both pipelines have finished ----
                    if (rd_a_done && rd_b_done) begin
                        rd_a_cnt  <= '0;
                        rd_b_cnt  <= '0;
                        rd_a_done <= 1'b0;
                        rd_b_done <= 1'b0;
                        ms_state  <= MS_RESET_ARRAY;
                    end
                end

                // -------------------------------------------------------------
                MS_RESET_ARRAY: begin
                    local_rst <= 1'b1;
                    run_cnt   <= '0;
                    ms_state  <= MS_DEASSERT;
                end

                // -------------------------------------------------------------
                MS_DEASSERT: begin
                    local_rst <= 1'b0;
                    run_cnt   <= '0;
                    ms_state  <= MS_RUN;
                end

                // -------------------------------------------------------------
                MS_RUN: begin
                    local_rst <= 1'b0;
                    if (run_cnt == capture_t) begin
                        cur_row   <= '0;
                        row_start <= 16'(int'(lat_row_tile) * REAL_N
                                         + int'(lat_col_tile));
                        word_addr <= AW'((int'(lat_row_tile) * REAL_N
                                         + int'(lat_col_tile)) / 4);
                        ms_state  <= MS_WRITE;
                    end else begin
                        run_cnt <= run_cnt + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                MS_WRITE: begin
                    begin
                        int sl;
                        sl = int'(row_start) % 4;
                        overflow_word <= '0;
                        overflow_mask <= 4'h0;
                        has_overflow  <= 1'b0;
                        for (int j = 0; j < 4; j++) begin
                            if (j < int'(lat_eff_cols) && (sl + j) >= 4) begin
                                overflow_word[(sl+j-4)*8 +: 8] <= out_byte[j];
                                overflow_mask[sl+j-4]          <= 1'b1;
                                has_overflow                   <= 1'b1;
                            end
                        end
                        overflow_addr <= AW'(int'(word_addr) + 1);
                    end
                    begin
                        int sl2;
                        sl2 = int'(row_start) % 4;
                        if (sl2 + int'(lat_eff_cols) > 4) begin
                            ms_state <= MS_OVERFLOW;
                        end else if (int'(cur_row) == int'(lat_eff_rows) - 1) begin
                            ms_state <= MS_ADVANCE;
                        end else begin
                            cur_row   <= $bits(cur_row)'(int'(cur_row) + 1);
                            row_start <= 16'(int'(row_start) + REAL_N);
                            word_addr <= AW'((int'(row_start) + REAL_N) / 4);
                            ms_state  <= MS_WRITE;
                        end
                    end
                end

                // -------------------------------------------------------------
                MS_OVERFLOW: begin
                    if (int'(cur_row) == int'(lat_eff_rows) - 1) begin
                        ms_state <= MS_ADVANCE;
                    end else begin
                        cur_row   <= $bits(cur_row)'(int'(cur_row) + 1);
                        row_start <= 16'(int'(row_start) + REAL_N);
                        word_addr <= AW'((int'(row_start) + REAL_N) / 4);
                        ms_state  <= MS_WRITE;
                    end
                end

                // -------------------------------------------------------------
                // Advance tile iterator; check completion 1 cycle later (O3)
                // -------------------------------------------------------------
                MS_ADVANCE: begin
                    ms_state <= MS_ADVANCE_CHK;
                end

                MS_ADVANCE_CHK: begin
                    // ti_all_done is stable by now (1 cycle after MS_ADVANCE)
                    if (ti_all_done) begin
                        ms_state <= MS_DONE;
                    end else begin
                        dl_start_o <= 1'b1;    // fire dram_loader immediately (O2)
                        ms_state   <= MS_LOAD_DRAM;
                    end
                end

                // -------------------------------------------------------------
                MS_DONE: begin
                    done <= 1'b1;
                end

                default: ms_state <= MS_RESET;

            endcase
        end
    end

    // =========================================================================
    // Systolic array
    // =========================================================================
    systolic_tiled #(.TILE_MAX(TILE_MAX)) u_systolic (
        .clk(clk), .rst(local_rst),
        .a(a_feed), .b(b_feed),
        .c(tile_z)
    );

    // =========================================================================
    // Feed sequencer - skews data into systolic array
    //
    // Lane k of a_feed gets y_rf[run_cnt - k][k]:
    //   weight tap (run_cnt - k) for filter column k, skewed by k cycles.
    // Lane k of b_feed gets x_rf[k][run_cnt - k]:
    //   activation tap (run_cnt - k) for output row k, skewed by k cycles.
    // =========================================================================
    always_comb begin
        for (int k = 0; k < TILE_MAX; k++) begin
            automatic int  t_minus_k = int'(run_cnt) - k;
            automatic logic active   = (ms_state == MS_RUN ||
                                        ms_state == MS_DEASSERT);
            automatic logic in_range = (t_minus_k >= 0 &&
                                        t_minus_k < REAL_M);

            a_feed[k] = (active && k < int'(lat_eff_cols) && in_range)
                        ? y_rf[t_minus_k][k] : 8'sd0;

            b_feed[k] = (active && k < int'(lat_eff_rows) && in_range)
                        ? x_rf[k][t_minus_k] : 8'sd0;
        end
    end

    // =========================================================================
    // SRAM port 1 read address generation (combinational)
    // MS_READ_AB: both A and B addresses issued simultaneously each cycle
    // =========================================================================
    always_comb begin
        sram_a_csb1 = 1'b1; sram_a_addr1 = '0;
        sram_b_csb1 = 1'b1; sram_b_addr1 = '0;
        sram_c_csb1 = 1'b1; sram_c_addr1 = '0;

        if (ms_state == MS_READ_AB) begin
            if (rd_a_cnt < lat_words_a) begin
                if (!lat_act_sel) begin
                    sram_a_csb1  = 1'b0;
                    sram_a_addr1 = AW'(lat_sram_a_base + rd_a_cnt);
                end else begin
                    sram_c_csb1  = 1'b0;
                    sram_c_addr1 = AW'(lat_sram_a_base + rd_a_cnt);
                end
            end
            if (rd_b_cnt < lat_words_b) begin
                sram_b_csb1  = 1'b0;
                sram_b_addr1 = AW'(rd_b_cnt);
            end
        end
    end

    // =========================================================================
    // SRAM port 0 mux + output write path (combinational)
    // =========================================================================
    logic [31:0]   ow_word;
    logic [3:0]    ow_mask;
    logic [AW-1:0] ow_waddr;

    always_comb begin
        sram_a_csb0   = 1'b1; sram_a_web0 = 1'b1;
        sram_a_wmask0 = 4'hF; sram_a_addr0 = '0; sram_a_din0 = '0;
        sram_b_csb0   = 1'b1; sram_b_web0 = 1'b1;
        sram_b_wmask0 = 4'hF; sram_b_addr0 = '0; sram_b_din0 = '0;
        sram_c_csb0   = 1'b1; sram_c_web0 = 1'b1;
        sram_c_wmask0 = 4'hF; sram_c_addr0 = '0; sram_c_din0 = '0;
        ext_z_wr_en_o = 1'b0; ext_z_addr_o = '0;
        ext_z_data_o  = '0;   ext_z_wmask_o = 4'h0;
        ow_word       = '0;
        ow_mask       = 4'h0;
        ow_waddr      = word_addr;

        if (ms_state == MS_LOAD_DRAM) begin
            sram_a_csb0   = dl_sram_a_csb0;
            sram_a_web0   = dl_sram_a_web0;
            sram_a_wmask0 = dl_sram_a_wmask0;
            sram_a_addr0  = dl_sram_a_addr0;
            sram_a_din0   = dl_sram_a_din0;
            sram_b_csb0   = dl_sram_b_csb0;
            sram_b_web0   = dl_sram_b_web0;
            sram_b_wmask0 = dl_sram_b_wmask0;
            sram_b_addr0  = dl_sram_b_addr0;
            sram_b_din0   = dl_sram_b_din0;

        end else if (ms_state == MS_WRITE || ms_state == MS_OVERFLOW) begin
            automatic int start_lane;
            start_lane = int'(row_start) % 4;

            if (ms_state == MS_WRITE) begin
                ow_word = '0;
                ow_mask = 4'h0;
                for (int j = 0; j < 4; j++) begin
                    automatic int lane = start_lane + j;
                    if (j < int'(lat_eff_cols) && lane < 4) begin
                        ow_word[lane*8 +: 8] = out_byte[j];
                        ow_mask[lane]        = 1'b1;
                    end
                end
                ow_waddr = word_addr;
            end else begin
                ow_word  = overflow_word;
                ow_mask  = overflow_mask;
                ow_waddr = overflow_addr;
            end

            if (is_last_layer) begin
                ext_z_wr_en_o = 1'b1;
                ext_z_addr_o  = {8'b0, ow_waddr};
                ext_z_data_o  = ow_word;
                ext_z_wmask_o = ow_mask;
            end else if (!out_sel) begin
                sram_c_csb0   = 1'b0;
                sram_c_web0   = 1'b0;
                sram_c_wmask0 = ow_mask;
                sram_c_addr0  = ow_waddr;
                sram_c_din0   = ow_word;
            end else begin
                sram_a_csb0   = 1'b0;
                sram_a_web0   = 1'b0;
                sram_a_wmask0 = ow_mask;
                sram_a_addr0  = ow_waddr;
                sram_a_din0   = ow_word;
            end
        end
    end

endmodule
