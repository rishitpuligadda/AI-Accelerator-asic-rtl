// =============================================================================
// dram_loader_conv.sv  (v3 - clean rewrite, no local variable aliasing)
//
// Im2col fetch: simple 2-state pump per byte.
//   LS_FETCH_REQ: compute src address, issue DRAM read, latch is_pad/byte_lane
//   LS_FETCH_RSP: receive byte, pack into word buffer, flush word to SRAM A
//                 when pack_lane==3 or last column of im2col row
//
// Key fix: byte is written directly into pack_buf in RSP state.
//          SRAM write uses pack_buf directly (no intermediate local copy).
//          SRAM write is issued combinationally on the SAME cycle as RSP
//          so the sky130 SRAM sees csb0=0 at the posedge that follows.
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
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        LS_IDLE      = 3'd0,
        LS_FETCH_REQ = 3'd1,
        LS_FETCH_RSP = 3'd2,
        LS_FILL_B    = 3'd3,
        LS_FLUSH     = 3'd4,
        LS_READY     = 3'd5
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
    // Im2col position
    // -------------------------------------------------------------------------
    logic [5:0] ic_row;
    logic [5:0] ic_col;

    // Latched from REQ state for use in RSP state
    logic        is_pad;
    logic [1:0]  src_byte_lane;   // which byte lane within DRAM word

    // -------------------------------------------------------------------------
    // Pack buffer (accumulates up to 4 bytes before SRAM write)
    // -------------------------------------------------------------------------
    logic [7:0]  pack_b0, pack_b1, pack_b2, pack_b3;
    logic [1:0]  pack_lane;       // next lane to fill
    logic [AW-1:0] sram_a_wr;    // next SRAM A word address

    // -------------------------------------------------------------------------
    // SRAM A write control (combinational from RSP state)
    // -------------------------------------------------------------------------
    logic        do_write_a;
    logic [31:0] write_word_a;
    logic [3:0]  write_mask_a;
    logic [AW-1:0] write_addr_a;

    // -------------------------------------------------------------------------
    // Weight fetch counter
    // -------------------------------------------------------------------------
    logic [5:0] ld_cnt_b;

    // -------------------------------------------------------------------------
    // Im2col address helper (function - purely combinational)
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
    // Sequential FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ls_state        <= LS_IDLE;
            ic_row          <= '0;
            ic_col          <= '0;
            is_pad          <= 1'b0;
            src_byte_lane   <= '0;
            pack_b0         <= '0;
            pack_b1         <= '0;
            pack_b2         <= '0;
            pack_b3         <= '0;
            pack_lane       <= '0;
            sram_a_wr       <= '0;
            ld_cnt_b        <= '0;
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
                        pack_b0         <= '0;
                        pack_b1         <= '0;
                        pack_b2         <= '0;
                        pack_b3         <= '0;
                        pack_lane       <= '0;
                        sram_a_wr       <= sram_a_base_i;
                        ld_cnt_b        <= '0;
                        busy_o          <= 1'b1;
                        ls_state        <= skip_x_i ? LS_FILL_B : LS_FETCH_REQ;
                    end
                end

                // -------------------------------------------------------------
                // Latch address info, issue DRAM read combinationally
                // -------------------------------------------------------------
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

                // -------------------------------------------------------------
                // Receive byte, pack, flush if needed
                // -------------------------------------------------------------
                LS_FETCH_RSP: begin
                    begin
                        logic [7:0] bval;
                        bval = is_pad ? 8'h00
                                      : ext_x_data_i[int'(src_byte_lane)*8 +: 8];

                        // Write byte into correct pack register
                        case (pack_lane)
                            2'd0: pack_b0 <= bval;
                            2'd1: pack_b1 <= bval;
                            2'd2: pack_b2 <= bval;
                            2'd3: pack_b3 <= bval;
                        endcase

                        // Flush only when word is full (pack_lane==3)
                        // Do NOT flush at end of im2col row - bytes flow
                        // continuously across row boundaries so shadow_reader
                        // flat-index unpacking works correctly
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

                        // Advance im2col counters
                        if (int'(ic_col) == REAL_M - 1) begin
                            ic_col <= '0;
                            if (int'(ic_row) == int'(lat_eff_rows) - 1) begin
                                // Flush any remaining partial word before moving to weights
                                // (handled by final_flush state - for now just move on,
                                // shadow_reader handles partial last word via eff_rows*REAL_M)
                                ic_row    <= '0;
                                ld_cnt_b  <= '0;
                                // If partial word remains, flush it first
                                if (pack_lane != 2'd0)
                                    ls_state <= LS_FLUSH;
                                else
                                    ls_state <= LS_FILL_B;
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

                // -------------------------------------------------------------
                LS_FILL_B: begin
                    if (ld_cnt_b == lat_words_b) begin
                        ld_cnt_b     <= '0;
                        load_ready_o <= 1'b1;
                        ls_state     <= LS_READY;
                    end else begin
                        ld_cnt_b <= ld_cnt_b + 1'b1;
                    end
                end

                LS_FLUSH: begin
                    // Write partial word (pack_lane > 0, remaining bytes are 0)
                    // Combinational block handles the actual SRAM write
                    sram_a_wr <= AW'(int'(sram_a_wr) + 1);
                    pack_b0   <= '0;
                    pack_b1   <= '0;
                    pack_b2   <= '0;
                    pack_b3   <= '0;
                    pack_lane <= '0;
                    ld_cnt_b  <= '0;
                    ls_state  <= LS_FILL_B;
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
    // DEBUG monitor
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (ls_state == LS_FETCH_RSP) begin
            $display("[DL @%0t] RSP ic_row=%0d ic_col=%0d is_pad=%0d src_lane=%0d pack_lane=%0d",
                $time, ic_row, ic_col, is_pad, src_byte_lane, pack_lane);
        end
        if (sram_b_csb0 == 1'b0 && sram_b_web0 == 1'b0) begin
            $display("[DL @%0t] SRAM_B WR addr=%0d data=0x%08x mask=%04b",
                $time, sram_b_addr0, sram_b_din0, sram_b_wmask0);
        end
        if (sram_a_csb0 == 1'b0 && sram_a_web0 == 1'b0) begin
            $display("[DL @%0t] SRAM_A WR addr=%0d data=0x%08x mask=%04b",
                $time, sram_a_addr0, sram_a_din0, sram_a_wmask0);
        end
    end

    // -------------------------------------------------------------------------
    // Combinational: SRAM A write
    //
    // We write on the SAME cycle as LS_FETCH_RSP when a flush is needed.
    // The sky130 SRAM latches csb0/web0/addr0/din0 at posedge clk, so
    // driving them combinationally from the current state is correct ?
    // the NEXT posedge will latch and write the data.
    //
    // Current byte (bval) is not yet in pack_bX registers (they update at
    // the next posedge), so we compute it inline here too.
    // -------------------------------------------------------------------------
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
            // Flush remaining partial word - unused bytes already 0
            automatic logic [31:0] flush_word;
            flush_word = {pack_b3, pack_b2, pack_b1, pack_b0};
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
                // Build the word including the current byte
                write_word_a = {pack_b3, pack_b2, pack_b1, pack_b0};
                case (pack_lane)
                    2'd0: write_word_a[ 7: 0] = bval_c;
                    2'd1: write_word_a[15: 8] = bval_c;
                    2'd2: write_word_a[23:16] = bval_c;
                    2'd3: write_word_a[31:24] = bval_c;
                endcase
                // Always write full word - unused byte lanes are 0 from pack_buf reset
                // This prevents x propagation through SRAM into shadow_reader
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

    // -------------------------------------------------------------------------
    // Combinational: DRAM X address (LS_FETCH_REQ)
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Combinational: DRAM Y + SRAM B
    // -------------------------------------------------------------------------
    always_comb begin
        ext_y_rd_en_o = 1'b0;
        ext_y_addr_o  = '0;
        sram_b_csb0   = 1'b1;
        sram_b_web0   = 1'b1;
        sram_b_wmask0 = 4'hF;
        sram_b_addr0  = '0;
        sram_b_din0   = '0;

        if (ls_state == LS_FILL_B) begin
            if (ld_cnt_b < lat_words_b) begin
                automatic int y_col_base;
                y_col_base    = (int'(lat_col_tile) * REAL_M) / 4;
                ext_y_rd_en_o = 1'b1;
                ext_y_addr_o  = 16'(y_col_base + int'(ld_cnt_b));
            end
            if (ld_cnt_b >= 1) begin
                sram_b_csb0   = 1'b0;
                sram_b_web0   = 1'b0;
                sram_b_wmask0 = 4'hF;
                sram_b_addr0  = AW'(ld_cnt_b - 1);
                sram_b_din0   = ext_y_data_i;
            end
        end
    end

endmodule
