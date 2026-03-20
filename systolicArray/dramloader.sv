// =============================================================================
// dram_loader_conv.sv  (v4 - byte-by-byte weight fetch)
//
// Im2col fetch (activations): unchanged 2-state pump REQ ? RSP.
//
// Weight fetch (weights): now also a 2-state byte pump REQ_B ? RSP_B.
//   For a tile at col_tile with eff_cols filters, the weight byte for
//   tap t, tile-local column c is stored at DRAM Y byte address:
//       t * REAL_N + col_tile + c
//   These bytes are NOT contiguous when REAL_N > eff_cols (i.e. when
//   the filter dimension is tiled), so a word-level sequential fetch
//   picks up the wrong bytes.  The byte pump reads each weight byte
//   individually and packs them into SRAM B in tap-major, col-minor
//   order, exactly matching what MS_READ_B in compute_engine expects.
// =============================================================================

module dram_loader #(
    parameter int TILE_MAX = 4,
    parameter int REAL_K   = 4,
    parameter int REAL_M   = 9,
    parameter int REAL_N   = 1,
    parameter int IN_H     = 4,
    parameter int IN_W     = 4,
    parameter int IN_C     = 1,
    parameter int K_H      = 3,
    parameter int K_W      = 3,
    parameter int STRIDE   = 1,
    parameter int PAD      = 0,
    parameter int AW       = 8
)(
    input  logic clk,
    input  logic rst,

    input  logic        start_i,
    input  logic        skip_x_i,
    output logic        load_ready_o,
    output logic        busy_o,

    input  logic [$clog2(REAL_K):0]        row_tile_i,
    input  logic [$clog2(REAL_N):0]        col_tile_i,
    input  logic [$clog2(TILE_MAX+1)-1:0]  eff_rows_i,
    input  logic [$clog2(TILE_MAX+1)-1:0]  eff_cols_i,
    input  logic [5:0]   words_a_i,
    input  logic [5:0]   words_b_i,
    input  logic [AW-1:0] sram_a_base_i,

    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    output logic          sram_a_csb0,
    output logic          sram_a_web0,
    output logic [3:0]    sram_a_wmask0,
    output logic [AW-1:0] sram_a_addr0,
    output logic [31:0]   sram_a_din0,

    output logic          sram_b_csb0,
    output logic          sram_b_web0,
    output logic [3:0]    sram_b_wmask0,
    output logic [AW-1:0] sram_b_addr0,
    output logic [31:0]   sram_b_din0
);

    // -------------------------------------------------------------------------
    // Derived
    // -------------------------------------------------------------------------
    localparam int OUT_W = (IN_W + 2*PAD - K_W) / STRIDE + 1;

    // -------------------------------------------------------------------------
    // FSM states
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        LS_IDLE      = 3'd0,
        LS_FETCH_REQ = 3'd1,   // activation fetch: compute address
        LS_FETCH_RSP = 3'd2,   // activation fetch: receive byte, pack, flush
        LS_FLUSH     = 3'd3,   // flush partial activation word
        LS_FETCH_B_REQ = 3'd4, // weight fetch:     compute address
        LS_FETCH_B_RSP = 3'd5, // weight fetch:     receive byte, pack, flush
        LS_FLUSH_B   = 3'd6,   // flush partial weight word
        LS_READY     = 3'd7
    } ls_t;
    ls_t ls_state;

    // -------------------------------------------------------------------------
    // Latched tile geometry
    // -------------------------------------------------------------------------
    logic [$clog2(REAL_K):0]       lat_row_tile;
    logic [$clog2(REAL_N):0]       lat_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_rows;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_cols;
    logic [5:0]    lat_words_a, lat_words_b;
    logic [AW-1:0] lat_sram_a_base;

    // -------------------------------------------------------------------------
    // Im2col position (activation fetch)
    // -------------------------------------------------------------------------
    logic [5:0] ic_row;
    logic [5:0] ic_col;

    // Latched from REQ state for use in RSP state
    logic        is_pad;
    logic [1:0]  src_byte_lane;

    // -------------------------------------------------------------------------
    // Weight fetch position
    // -------------------------------------------------------------------------
    logic [5:0] wt_tap;   // current tap index (0..REAL_M-1)
    logic [2:0] wt_col;   // current tile-local column (0..eff_cols-1)
    logic [1:0] wt_byte_lane;  // byte lane within the fetched DRAM word

    // -------------------------------------------------------------------------
    // Shared pack buffer ? used for both activation and weight packing
    // -------------------------------------------------------------------------
    logic [7:0]  pack_b0, pack_b1, pack_b2, pack_b3;
    logic [1:0]  pack_lane;

    // SRAM A write address (activation packing)
    logic [AW-1:0] sram_a_wr;

    // SRAM B write address (weight packing)
    logic [AW-1:0] sram_b_wr;

    // -------------------------------------------------------------------------
    // SRAM A write control (combinational ? driven from RSP / FLUSH states)
    // -------------------------------------------------------------------------
    logic        do_write_a;
    logic [31:0] write_word_a;
    logic [3:0]  write_mask_a;
    logic [AW-1:0] write_addr_a;

    // -------------------------------------------------------------------------
    // SRAM B write control (combinational ? driven from RSP_B / FLUSH_B states)
    // -------------------------------------------------------------------------
    logic        do_write_b;
    logic [31:0] write_word_b;
    logic [3:0]  write_mask_b;
    logic [AW-1:0] write_addr_b;

    // -------------------------------------------------------------------------
    // Im2col address helper
    // -------------------------------------------------------------------------
    function automatic void get_im2col(
        input  int row_tile_in, ic_r, ic_c,
        output logic        is_pad_out,
        output logic [15:0] word_addr_out,
        output logic [1:0]  byte_lane_out
    );
        int glob_row, oh, ow, ic_ch, kh_i, kw_i;
        int src_h, src_w, byte_addr;
        glob_row = row_tile_in + ic_r;
        oh       = glob_row / OUT_W;
        ow       = glob_row % OUT_W;
        ic_ch    = ic_c / (K_H * K_W);
        kh_i     = (ic_c % (K_H * K_W)) / K_W;
        kw_i     = ic_c % K_W;
        src_h    = oh * STRIDE + kh_i - PAD;
        src_w    = ow * STRIDE + kw_i - PAD;
        if (src_h < 0 || src_h >= IN_H || src_w < 0 || src_w >= IN_W) begin
            is_pad_out    = 1'b1;
            word_addr_out = '0;
            byte_lane_out = '0;
        end else begin
            byte_addr     = ic_ch * IN_H * IN_W + src_h * IN_W + src_w;
            is_pad_out    = 1'b0;
            word_addr_out = 16'(byte_addr >> 2);
            byte_lane_out = 2'(byte_addr & 2'h3);
        end
    endfunction

    // -------------------------------------------------------------------------
    // Weight byte address helper
    //   byte_addr = tap * REAL_N + col_tile + tile_local_col
    //   word_addr = byte_addr >> 2
    //   lane      = byte_addr  & 3
    // -------------------------------------------------------------------------
    function automatic void get_weight_addr(
        input  int tap_in, col_tile_in, tile_col_in,
        output logic [15:0] word_addr_out,
        output logic [1:0]  byte_lane_out
    );
        int byte_addr;
        byte_addr     = tap_in * REAL_N + col_tile_in + tile_col_in;
        word_addr_out = 16'(byte_addr >> 2);
        byte_lane_out = 2'(byte_addr & 2'h3);
    endfunction

    // =========================================================================
    // Sequential FSM
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ls_state        <= LS_IDLE;
            ic_row          <= '0;
            ic_col          <= '0;
            wt_tap          <= '0;
            wt_col          <= '0;
            wt_byte_lane    <= '0;
            is_pad          <= 1'b0;
            src_byte_lane   <= '0;
            pack_b0         <= '0;
            pack_b1         <= '0;
            pack_b2         <= '0;
            pack_b3         <= '0;
            pack_lane       <= '0;
            sram_a_wr       <= '0;
            sram_b_wr       <= '0;
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

                // -------------------------------------------------------------
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
                        ic_row          <= '0;
                        ic_col          <= '0;
                        wt_tap          <= '0;
                        wt_col          <= '0;
                        pack_b0         <= '0;
                        pack_b1         <= '0;
                        pack_b2         <= '0;
                        pack_b3         <= '0;
                        pack_lane       <= '0;
                        sram_a_wr       <= sram_a_base_i;
                        sram_b_wr       <= '0;
                        busy_o          <= 1'b1;
                        ls_state        <= skip_x_i ? LS_FETCH_B_REQ : LS_FETCH_REQ;
                    end
                end

                // =============================================================
                // ACTIVATION FETCH  (unchanged from v3)
                // =============================================================

                LS_FETCH_REQ: begin
                    begin
                        logic        p;
                        logic [15:0] wa;
                        logic [1:0]  bl;
                        get_im2col(int'(lat_row_tile), int'(ic_row),
                                   int'(ic_col), p, wa, bl);
                        is_pad        <= p;
                        src_byte_lane <= bl;
                    end
                    ls_state <= LS_FETCH_RSP;
                end

                LS_FETCH_RSP: begin
                    begin
                        logic [7:0] bval;
                        bval = is_pad ? 8'h00
                                      : ext_x_data_i[int'(src_byte_lane)*8 +: 8];

                        case (pack_lane)
                            2'd0: pack_b0 <= bval;
                            2'd1: pack_b1 <= bval;
                            2'd2: pack_b2 <= bval;
                            2'd3: pack_b3 <= bval;
                        endcase

                        if (pack_lane == 2'd3) begin
                            sram_a_wr <= AW'(int'(sram_a_wr) + 1);
                            pack_b0   <= '0;
                            pack_b1   <= '0;
                            pack_b2   <= '0;
                            pack_b3   <= '0;
                            pack_lane <= '0;
                        end else begin
                            pack_lane <= pack_lane + 2'd1;
                        end

                        if (int'(ic_col) == REAL_M - 1) begin
                            ic_col <= '0;
                            if (int'(ic_row) == int'(lat_eff_rows) - 1) begin
                                ic_row <= '0;
                                if (pack_lane != 2'd3)
                                    ls_state <= LS_FLUSH;
                                else begin
                                    // Start weight fetch
                                    pack_b0   <= '0;
                                    pack_b1   <= '0;
                                    pack_b2   <= '0;
                                    pack_b3   <= '0;
                                    pack_lane <= '0;
                                    ls_state  <= LS_FETCH_B_REQ;
                                end
                            end else begin
                                ic_row   <= 6'(int'(ic_row) + 1);
                                ls_state <= LS_FETCH_REQ;
                            end
                        end else begin
                            ic_col   <= 6'(int'(ic_col) + 1);
                            ls_state <= LS_FETCH_REQ;
                        end
                    end
                end

                LS_FLUSH: begin
                    // Flush partial activation word
                    sram_a_wr <= AW'(int'(sram_a_wr) + 1);
                    pack_b0   <= '0;
                    pack_b1   <= '0;
                    pack_b2   <= '0;
                    pack_b3   <= '0;
                    pack_lane <= '0;
                    ls_state  <= LS_FETCH_B_REQ;
                end

                // =============================================================
                // WEIGHT FETCH  (new byte-by-byte pump)
                // =============================================================

                LS_FETCH_B_REQ: begin
                    // Compute DRAM Y address for (wt_tap, wt_col) and latch lane
                    begin
                        logic [15:0] wa;
                        logic [1:0]  bl;
                        get_weight_addr(int'(wt_tap),
                                        int'(lat_col_tile),
                                        int'(wt_col),
                                        wa, bl);
                        wt_byte_lane <= bl;
                        // ext_y_rd_en and ext_y_addr driven combinationally below
                    end
                    ls_state <= LS_FETCH_B_RSP;
                end

                LS_FETCH_B_RSP: begin
                    begin
                        logic [7:0] bval;
                        bval = ext_y_data_i[int'(wt_byte_lane)*8 +: 8];

                        case (pack_lane)
                            2'd0: pack_b0 <= bval;
                            2'd1: pack_b1 <= bval;
                            2'd2: pack_b2 <= bval;
                            2'd3: pack_b3 <= bval;
                        endcase

                        if (pack_lane == 2'd3) begin
                            sram_b_wr <= AW'(int'(sram_b_wr) + 1);
                            pack_b0   <= '0;
                            pack_b1   <= '0;
                            pack_b2   <= '0;
                            pack_b3   <= '0;
                            pack_lane <= '0;
                        end else begin
                            pack_lane <= pack_lane + 2'd1;
                        end

                        // Advance weight position counters
                        if (int'(wt_col) == int'(lat_eff_cols) - 1) begin
                            wt_col <= '0;
                            if (int'(wt_tap) == REAL_M - 1) begin
                                // All weight bytes fetched
                                wt_tap <= '0;
                                if (pack_lane != 2'd3)
                                    ls_state <= LS_FLUSH_B;
                                else begin
                                    pack_b0      <= '0;
                                    pack_b1      <= '0;
                                    pack_b2      <= '0;
                                    pack_b3      <= '0;
                                    pack_lane    <= '0;
                                    load_ready_o <= 1'b1;
                                    ls_state     <= LS_READY;
                                end
                            end else begin
                                wt_tap   <= 6'(int'(wt_tap) + 1);
                                ls_state <= LS_FETCH_B_REQ;
                            end
                        end else begin
                            wt_col   <= 3'(int'(wt_col) + 1);
                            ls_state <= LS_FETCH_B_REQ;
                        end
                    end
                end

                LS_FLUSH_B: begin
                    // Flush partial weight word
                    sram_b_wr    <= AW'(int'(sram_b_wr) + 1);
                    pack_b0      <= '0;
                    pack_b1      <= '0;
                    pack_b2      <= '0;
                    pack_b3      <= '0;
                    pack_lane    <= '0;
                    load_ready_o <= 1'b1;
                    ls_state     <= LS_READY;
                end

                LS_READY: begin
                    busy_o   <= 1'b0;
                    ls_state <= LS_IDLE;
                end

                default: ls_state <= LS_IDLE;
            endcase
        end
    end


    // =========================================================================
    // Combinational: SRAM A write
    //
    // Written on LS_FETCH_RSP when pack_lane==3 (full word ready),
    // and on LS_FLUSH for the final partial word.
    // Current byte is NOT yet in pack_bX (updates at next posedge), so
    // we compute it inline here.
    // =========================================================================
    always_comb begin
        do_write_a   = 1'b0;
        write_word_a = '0;
        write_mask_a = '0;
        write_addr_a = sram_a_wr;

        sram_a_csb0   = 1'b1;
        sram_a_web0   = 1'b1;
        sram_a_wmask0 = 4'hF;
        sram_a_addr0  = '0;
        sram_a_din0   = '0;

        if (ls_state == LS_FLUSH) begin
            automatic logic [31:0] flush_word;
            flush_word    = {pack_b3, pack_b2, pack_b1, pack_b0};
            sram_a_csb0   = 1'b0;
            sram_a_web0   = 1'b0;
            sram_a_wmask0 = 4'hF;
            sram_a_addr0  = sram_a_wr;
            sram_a_din0   = flush_word;
        end else if (ls_state == LS_FETCH_RSP) begin
            automatic logic [7:0] bval_c;
            bval_c = is_pad ? 8'h00
                            : ext_x_data_i[int'(src_byte_lane)*8 +: 8];

            if (pack_lane == 2'd3) begin
                write_word_a = {pack_b3, pack_b2, pack_b1, pack_b0};
                case (pack_lane)
                    2'd0: write_word_a[ 7: 0] = bval_c;
                    2'd1: write_word_a[15: 8] = bval_c;
                    2'd2: write_word_a[23:16] = bval_c;
                    2'd3: write_word_a[31:24] = bval_c;
                endcase
                write_mask_a = 4'hF;
                write_addr_a = sram_a_wr;
                do_write_a   = 1'b1;

                sram_a_csb0   = 1'b0;
                sram_a_web0   = 1'b0;
                sram_a_wmask0 = 4'hF;
                sram_a_addr0  = write_addr_a;
                sram_a_din0   = write_word_a;
            end
        end
    end

    // =========================================================================
    // Combinational: DRAM X address (LS_FETCH_REQ)
    // =========================================================================
    always_comb begin
        ext_x_rd_en_o = 1'b0;
        ext_x_addr_o  = '0;
        if (ls_state == LS_FETCH_REQ) begin
            logic        p;
            logic [15:0] wa;
            logic [1:0]  bl;
            get_im2col(int'(lat_row_tile), int'(ic_row),
                       int'(ic_col), p, wa, bl);
            if (!p) begin
                ext_x_rd_en_o = 1'b1;
                ext_x_addr_o  = wa;
            end
        end
    end

    // =========================================================================
    // Combinational: DRAM Y address (LS_FETCH_B_REQ) + SRAM B write (LS_FETCH_B_RSP / LS_FLUSH_B)
    // =========================================================================
    always_comb begin
        ext_y_rd_en_o = 1'b0;
        ext_y_addr_o  = '0;
        sram_b_csb0   = 1'b1;
        sram_b_web0   = 1'b1;
        sram_b_wmask0 = 4'hF;
        sram_b_addr0  = '0;
        sram_b_din0   = '0;

        if (ls_state == LS_FETCH_B_REQ) begin
            logic [15:0] wa;
            logic [1:0]  bl;
            get_weight_addr(int'(wt_tap), int'(lat_col_tile), int'(wt_col), wa, bl);
            ext_y_rd_en_o = 1'b1;
            ext_y_addr_o  = wa;

        end else if (ls_state == LS_FLUSH_B) begin
            automatic logic [31:0] flush_word;
            flush_word    = {pack_b3, pack_b2, pack_b1, pack_b0};
            sram_b_csb0   = 1'b0;
            sram_b_web0   = 1'b0;
            sram_b_wmask0 = 4'hF;
            sram_b_addr0  = sram_b_wr;
            sram_b_din0   = flush_word;

        end else if (ls_state == LS_FETCH_B_RSP) begin
            automatic logic [7:0] bval_c;
            bval_c = ext_y_data_i[int'(wt_byte_lane)*8 +: 8];

            if (pack_lane == 2'd3) begin
                automatic logic [31:0] word_c;
                word_c = {pack_b3, pack_b2, pack_b1, pack_b0};
                case (pack_lane)
                    2'd0: word_c[ 7: 0] = bval_c;
                    2'd1: word_c[15: 8] = bval_c;
                    2'd2: word_c[23:16] = bval_c;
                    2'd3: word_c[31:24] = bval_c;
                endcase
                sram_b_csb0   = 1'b0;
                sram_b_web0   = 1'b0;
                sram_b_wmask0 = 4'hF;
                sram_b_addr0  = sram_b_wr;
                sram_b_din0   = word_c;
            end
        end
    end

endmodule
