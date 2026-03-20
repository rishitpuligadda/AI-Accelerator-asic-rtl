// =============================================================================
// dram_loader.sv  (v5 - dual parallel pipelined fetch)
//
// Optimisation 1: Pipelined byte fetch (1 cycle per byte instead of 2)
//   Each pipeline uses a 2-stage structure:
//     Cycle N  : compute address, issue DRAM read  (stage 0 = issue)
//     Cycle N+1: receive data from DRAM            (stage 1 = receive)
//   While stage 1 is receiving byte[k], stage 0 is already issuing addr[k+1].
//   Net throughput: 1 cycle per byte after a 1-cycle fill penalty.
//
// Optimisation 2: Parallel activation + weight fetch
//   Activations read from DRAM X into SRAM A.
//   Weights    read from DRAM Y into SRAM B.
//   These use completely separate DRAM buses and separate SRAM ports, so they
//   run simultaneously from the moment start_i fires.
//   load_ready_o is asserted when BOTH pipelines have finished.
//
// Combined speedup (typical tile):
//   Before: 2*(act_bytes + wt_bytes) cycles (sequential, 2 per byte)
//   After:  max(act_bytes, wt_bytes) + 1 cycles (parallel, 1 per byte)
//   Example: 36-byte act + 36-byte wt -> 144 cycles -> 37 cycles (~3.9x)
//
// Pipeline states (two independent FSMs):
//   Activation: ACT_IDLE -> ACT_PIPE -> ACT_DRAIN -> [ACT_FLUSH ->] ACT_DONE
//   Weight:     WT_IDLE  -> WT_PIPE  -> WT_DRAIN  -> [WT_FLUSH  ->] WT_DONE
//
// ACT_PIPE / WT_PIPE (steady-state, one byte per cycle):
//   Each cycle:
//     1. If stage-1 pipeline register is valid: extract byte from DRAM data,
//        pack it, flush to SRAM when 4 bytes accumulate.
//     2. If there are remaining bytes to issue: compute next address,
//        drive DRAM read enable, update stage-1 pipeline register.
//     3. When the last address has been issued, move to DRAIN on the next cycle.
//
// ACT_DRAIN / WT_DRAIN (1 cycle, receive the last in-flight byte):
//   Receive and pack the final byte (stage-1 is still valid).
//   Move to FLUSH if a partial word is sitting in the pack buffer, else DONE.
//
// ACT_FLUSH / WT_FLUSH (1 cycle, write the partial last word):
//   Drive the SRAM write with the remaining pack bytes, advance wr pointer.
//   Move to DONE.
//
// ACT_DONE / WT_DONE:
//   Pipeline is finished. When both reach DONE, load_ready_o pulses for 1 cycle
//   and both pipelines return to IDLE.
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
    // Derived constants
    // -------------------------------------------------------------------------
    localparam int OUT_W = (IN_W + 2*PAD - K_W) / STRIDE + 1;

    // -------------------------------------------------------------------------
    // Activation pipeline FSM states
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ACT_IDLE  = 3'd0,
        ACT_PIPE  = 3'd1,
        ACT_DRAIN = 3'd2,
        ACT_FLUSH = 3'd3,
        ACT_DONE  = 3'd4
    } act_t;
    act_t act_state;

    // -------------------------------------------------------------------------
    // Weight pipeline FSM states
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        WT_IDLE  = 3'd0,
        WT_PIPE  = 3'd1,
        WT_DRAIN = 3'd2,
        WT_FLUSH = 3'd3,
        WT_DONE  = 3'd4
    } wt_t;
    wt_t wt_state;

    // -------------------------------------------------------------------------
    // Latched tile geometry (captured on start_i)
    // -------------------------------------------------------------------------
    logic [$clog2(REAL_K):0]       lat_row_tile;
    logic [$clog2(REAL_N):0]       lat_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_rows;
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_cols;
    logic [AW-1:0]                 lat_sram_a_base;

    // Total bytes each pipeline must transfer for this tile
    // Computed combinationally from latched geometry, used during ACT_PIPE/WT_PIPE
    logic [7:0] act_total;   // eff_rows * REAL_M
    logic [7:0] wt_total;    // REAL_M * eff_cols
    always_comb begin
        act_total = 8'(int'(lat_eff_rows) * REAL_M);
        wt_total  = 8'(REAL_M * int'(lat_eff_cols));
    end

    // -------------------------------------------------------------------------
    // Activation pipeline: issue-side counters and stage-1 register
    // -------------------------------------------------------------------------
    logic [7:0] act_issue_cnt;  // counts bytes issued so far (0..act_total-1)
    logic [5:0] act_ic_row;     // im2col row being issued (0..eff_rows-1)
    logic [5:0] act_ic_col;     // im2col col being issued (0..REAL_M-1)

    logic        act_s1_valid;      // stage-1 pipeline register: was a read issued?
    logic        act_s1_is_pad;     // was the issued address a pad position?
    logic [1:0]  act_s1_byte_lane;  // byte lane of the issued read

    // Activation pack buffer (independent of weight pack)
    logic [7:0]  act_pb0, act_pb1, act_pb2, act_pb3;
    logic [1:0]  act_pack_lane;
    logic [AW-1:0] act_sram_wr;     // next SRAM A word address to write

    // -------------------------------------------------------------------------
    // Weight pipeline: issue-side counters and stage-1 register
    // -------------------------------------------------------------------------
    logic [7:0] wt_issue_cnt;  // counts bytes issued so far (0..wt_total-1)
    logic [5:0] wt_tap;        // tap being issued (0..REAL_M-1)
    logic [2:0] wt_col;        // tile-local column being issued (0..eff_cols-1)

    logic        wt_s1_valid;
    logic [1:0]  wt_s1_byte_lane;

    // Weight pack buffer
    logic [7:0]  wt_pb0, wt_pb1, wt_pb2, wt_pb3;
    logic [1:0]  wt_pack_lane;
    logic [AW-1:0] wt_sram_wr;     // next SRAM B word address to write

    // -------------------------------------------------------------------------
    // Address helper functions (purely combinational, same as v4)
    // -------------------------------------------------------------------------
    function automatic void get_im2col(
        input  int row_tile_in, ic_r, ic_c,
        output logic        is_pad_out,
        output logic [15:0] word_addr_out,
        output logic [1:0]  byte_lane_out
    );
        int glob_row, oh, ow, ic_ch, kh_i, kw_i, src_h, src_w, byte_addr;
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
    // ACTIVATION PIPELINE ? sequential
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            act_state      <= ACT_IDLE;
            act_issue_cnt  <= '0;
            act_ic_row     <= '0;
            act_ic_col     <= '0;
            act_s1_valid   <= 1'b0;
            act_s1_is_pad  <= 1'b0;
            act_s1_byte_lane <= '0;
            act_pb0        <= '0; act_pb1 <= '0;
            act_pb2        <= '0; act_pb3 <= '0;
            act_pack_lane  <= '0;
            act_sram_wr    <= '0;
        end else begin
            case (act_state)

                // ---------------------------------------------------------
                ACT_IDLE: begin
                    act_s1_valid <= 1'b0;
                    if (start_i) begin
                        act_ic_row    <= '0;
                        act_ic_col    <= '0;
                        act_issue_cnt <= '0;
                        act_pb0       <= '0; act_pb1 <= '0;
                        act_pb2       <= '0; act_pb3 <= '0;
                        act_pack_lane <= '0;
                        act_sram_wr   <= sram_a_base_i;
                        if (skip_x_i) begin
                            act_s1_valid <= 1'b0;
                            act_state    <= ACT_DONE;
                        end else begin
                            act_state <= ACT_PIPE;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Steady-state: one byte received, one address issued per cycle
                // ---------------------------------------------------------
                ACT_PIPE: begin
                    // ---- Stage 1: receive the byte issued last cycle ----
                    if (act_s1_valid) begin
                        logic [7:0] bval;
                        bval = act_s1_is_pad ? 8'h00
                                             : ext_x_data_i[int'(act_s1_byte_lane)*8 +: 8];
                        case (act_pack_lane)
                            2'd0: act_pb0 <= bval;
                            2'd1: act_pb1 <= bval;
                            2'd2: act_pb2 <= bval;
                            2'd3: act_pb3 <= bval;
                        endcase
                        if (act_pack_lane == 2'd3) begin
                            act_sram_wr   <= AW'(int'(act_sram_wr) + 1);
                            act_pb0       <= '0; act_pb1 <= '0;
                            act_pb2       <= '0; act_pb3 <= '0;
                            act_pack_lane <= '0;
                        end else begin
                            act_pack_lane <= act_pack_lane + 2'd1;
                        end
                    end

                    // ---- Stage 0: issue next address ----
                    if (int'(act_issue_cnt) < int'(act_total)) begin
                        // Issue address for current (ic_row, ic_col)
                        // (ext_x_rd_en_o / ext_x_addr_o driven combinationally below)
                        begin
                            logic        p;
                            logic [15:0] wa;
                            logic [1:0]  bl;
                            get_im2col(int'(lat_row_tile), int'(act_ic_row),
                                       int'(act_ic_col), p, wa, bl);
                            act_s1_valid     <= 1'b1;
                            act_s1_is_pad    <= p;
                            act_s1_byte_lane <= bl;
                        end
                        // Advance im2col counters
                        if (int'(act_ic_col) == REAL_M - 1) begin
                            act_ic_col <= '0;
                            act_ic_row <= 6'(int'(act_ic_row) + 1);
                        end else begin
                            act_ic_col <= 6'(int'(act_ic_col) + 1);
                        end
                        act_issue_cnt <= 8'(int'(act_issue_cnt) + 1);

                        // If this was the last byte to issue, next cycle is DRAIN
                        if (int'(act_issue_cnt) == int'(act_total) - 1)
                            act_state <= ACT_DRAIN;
                    end else begin
                        act_s1_valid <= 1'b0;
                    end
                end

                // ---------------------------------------------------------
                // Drain: receive the final in-flight byte, no new issue
                // ---------------------------------------------------------
                ACT_DRAIN: begin
                    act_s1_valid <= 1'b0;
                    if (act_s1_valid) begin
                        logic [7:0] bval;
                        bval = act_s1_is_pad ? 8'h00
                                             : ext_x_data_i[int'(act_s1_byte_lane)*8 +: 8];
                        case (act_pack_lane)
                            2'd0: act_pb0 <= bval;
                            2'd1: act_pb1 <= bval;
                            2'd2: act_pb2 <= bval;
                            2'd3: act_pb3 <= bval;
                        endcase
                        if (act_pack_lane == 2'd3) begin
                            act_sram_wr   <= AW'(int'(act_sram_wr) + 1);
                            act_pb0       <= '0; act_pb1 <= '0;
                            act_pb2       <= '0; act_pb3 <= '0;
                            act_pack_lane <= '0;
                            act_state     <= ACT_DONE;
                        end else begin
                            act_pack_lane <= act_pack_lane + 2'd1;
                            act_state     <= ACT_FLUSH;
                        end
                    end else begin
                        // s1 was not valid (act_total == 0 edge case)
                        act_state <= (act_pack_lane != 2'd0) ? ACT_FLUSH : ACT_DONE;
                    end
                end

                // ---------------------------------------------------------
                // Flush: write the remaining partial word to SRAM A
                // (combinational block drives the SRAM write this cycle)
                // ---------------------------------------------------------
                ACT_FLUSH: begin
                    act_sram_wr   <= AW'(int'(act_sram_wr) + 1);
                    act_pb0       <= '0; act_pb1 <= '0;
                    act_pb2       <= '0; act_pb3 <= '0;
                    act_pack_lane <= '0;
                    act_state     <= ACT_DONE;
                end

                // ---------------------------------------------------------
                ACT_DONE: begin
                    // Wait here until both pipelines done (handled in output logic)
                    // Return to IDLE is triggered by the load_ready pulse
                    if (load_ready_o)
                        act_state <= ACT_IDLE;
                end

                default: act_state <= ACT_IDLE;
            endcase
        end
    end

    // =========================================================================
    // WEIGHT PIPELINE ? sequential
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wt_state       <= WT_IDLE;
            wt_issue_cnt   <= '0;
            wt_tap         <= '0;
            wt_col         <= '0;
            wt_s1_valid    <= 1'b0;
            wt_s1_byte_lane <= '0;
            wt_pb0         <= '0; wt_pb1 <= '0;
            wt_pb2         <= '0; wt_pb3 <= '0;
            wt_pack_lane   <= '0;
            wt_sram_wr     <= '0;
        end else begin
            case (wt_state)

                // ---------------------------------------------------------
                WT_IDLE: begin
                    wt_s1_valid <= 1'b0;
                    if (start_i) begin
                        wt_tap        <= '0;
                        wt_col        <= '0;
                        wt_issue_cnt  <= '0;
                        wt_pb0        <= '0; wt_pb1 <= '0;
                        wt_pb2        <= '0; wt_pb3 <= '0;
                        wt_pack_lane  <= '0;
                        wt_sram_wr    <= '0;
                        wt_state      <= WT_PIPE;
                    end
                end

                // ---------------------------------------------------------
                WT_PIPE: begin
                    // ---- Stage 1: receive the byte issued last cycle ----
                    if (wt_s1_valid) begin
                        logic [7:0] bval;
                        bval = ext_y_data_i[int'(wt_s1_byte_lane)*8 +: 8];
                        case (wt_pack_lane)
                            2'd0: wt_pb0 <= bval;
                            2'd1: wt_pb1 <= bval;
                            2'd2: wt_pb2 <= bval;
                            2'd3: wt_pb3 <= bval;
                        endcase
                        if (wt_pack_lane == 2'd3) begin
                            wt_sram_wr   <= AW'(int'(wt_sram_wr) + 1);
                            wt_pb0       <= '0; wt_pb1 <= '0;
                            wt_pb2       <= '0; wt_pb3 <= '0;
                            wt_pack_lane <= '0;
                        end else begin
                            wt_pack_lane <= wt_pack_lane + 2'd1;
                        end
                    end

                    // ---- Stage 0: issue next weight address ----
                    if (int'(wt_issue_cnt) < int'(wt_total)) begin
                        begin
                            logic [15:0] wa;
                            logic [1:0]  bl;
                            get_weight_addr(int'(wt_tap), int'(lat_col_tile),
                                            int'(wt_col), wa, bl);
                            wt_s1_valid     <= 1'b1;
                            wt_s1_byte_lane <= bl;
                        end
                        // Advance weight counters (col-major within tap)
                        if (int'(wt_col) == int'(lat_eff_cols) - 1) begin
                            wt_col <= '0;
                            wt_tap <= 6'(int'(wt_tap) + 1);
                        end else begin
                            wt_col <= 3'(int'(wt_col) + 1);
                        end
                        wt_issue_cnt <= 8'(int'(wt_issue_cnt) + 1);

                        if (int'(wt_issue_cnt) == int'(wt_total) - 1)
                            wt_state <= WT_DRAIN;
                    end else begin
                        wt_s1_valid <= 1'b0;
                    end
                end

                // ---------------------------------------------------------
                WT_DRAIN: begin
                    wt_s1_valid <= 1'b0;
                    if (wt_s1_valid) begin
                        logic [7:0] bval;
                        bval = ext_y_data_i[int'(wt_s1_byte_lane)*8 +: 8];
                        case (wt_pack_lane)
                            2'd0: wt_pb0 <= bval;
                            2'd1: wt_pb1 <= bval;
                            2'd2: wt_pb2 <= bval;
                            2'd3: wt_pb3 <= bval;
                        endcase
                        if (wt_pack_lane == 2'd3) begin
                            wt_sram_wr   <= AW'(int'(wt_sram_wr) + 1);
                            wt_pb0       <= '0; wt_pb1 <= '0;
                            wt_pb2       <= '0; wt_pb3 <= '0;
                            wt_pack_lane <= '0;
                            wt_state     <= WT_DONE;
                        end else begin
                            wt_pack_lane <= wt_pack_lane + 2'd1;
                            wt_state     <= WT_FLUSH;
                        end
                    end else begin
                        wt_state <= (wt_pack_lane != 2'd0) ? WT_FLUSH : WT_DONE;
                    end
                end

                // ---------------------------------------------------------
                WT_FLUSH: begin
                    wt_sram_wr   <= AW'(int'(wt_sram_wr) + 1);
                    wt_pb0       <= '0; wt_pb1 <= '0;
                    wt_pb2       <= '0; wt_pb3 <= '0;
                    wt_pack_lane <= '0;
                    wt_state     <= WT_DONE;
                end

                // ---------------------------------------------------------
                WT_DONE: begin
                    if (load_ready_o)
                        wt_state <= WT_IDLE;
                end

                default: wt_state <= WT_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Latch tile geometry on start_i
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            lat_row_tile    <= '0;
            lat_col_tile    <= '0;
            lat_eff_rows    <= '0;
            lat_eff_cols    <= '0;
            lat_sram_a_base <= '0;
            busy_o          <= 1'b0;
        end else begin
            if (start_i) begin
                lat_row_tile    <= row_tile_i;
                lat_col_tile    <= col_tile_i;
                lat_eff_rows    <= eff_rows_i;
                lat_eff_cols    <= eff_cols_i;
                lat_sram_a_base <= sram_a_base_i;
                busy_o          <= 1'b1;
            end
            if (load_ready_o)
                busy_o <= 1'b0;
        end
    end

    // =========================================================================
    // load_ready_o: pulse for 1 cycle when both pipelines reach DONE
    // =========================================================================
    assign load_ready_o = (act_state == ACT_DONE) && (wt_state == WT_DONE);

    // =========================================================================
    // Combinational: DRAM X read (activation issue stage)
    // Driven when ACT_PIPE and there are bytes left to issue.
    // =========================================================================
    always_comb begin
        ext_x_rd_en_o = 1'b0;
        ext_x_addr_o  = '0;
        if (act_state == ACT_PIPE && int'(act_issue_cnt) < int'(act_total)) begin
            logic        p;
            logic [15:0] wa;
            logic [1:0]  bl;
            get_im2col(int'(lat_row_tile), int'(act_ic_row), int'(act_ic_col),
                       p, wa, bl);
            if (!p) begin
                ext_x_rd_en_o = 1'b1;
                ext_x_addr_o  = wa;
            end
        end
    end

    // =========================================================================
    // Combinational: DRAM Y read (weight issue stage)
    // Driven when WT_PIPE and there are bytes left to issue.
    // =========================================================================
    always_comb begin
        ext_y_rd_en_o = 1'b0;
        ext_y_addr_o  = '0;
        if (wt_state == WT_PIPE && int'(wt_issue_cnt) < int'(wt_total)) begin
            logic [15:0] wa;
            logic [1:0]  bl;
            get_weight_addr(int'(wt_tap), int'(lat_col_tile), int'(wt_col), wa, bl);
            ext_y_rd_en_o = 1'b1;
            ext_y_addr_o  = wa;
        end
    end

    // =========================================================================
    // Combinational: SRAM A write
    //
    // Fires in three situations:
    //   ACT_PIPE  when act_pack_lane==3 and act_s1_valid: the 4th byte completes
    //             a full word. The current byte comes directly from ext_x_data_i
    //             (it is not in act_pb3 yet ? that updates at posedge).
    //   ACT_DRAIN same condition (receiving final in-flight byte).
    //   ACT_FLUSH the partial word held in act_pb0..pb3 is written as-is.
    // =========================================================================
    always_comb begin
        sram_a_csb0   = 1'b1;
        sram_a_web0   = 1'b1;
        sram_a_wmask0 = 4'hF;
        sram_a_addr0  = '0;
        sram_a_din0   = '0;

        if (act_state == ACT_FLUSH) begin
            sram_a_csb0   = 1'b0;
            sram_a_web0   = 1'b0;
            sram_a_wmask0 = 4'hF;
            sram_a_addr0  = act_sram_wr;
            sram_a_din0   = {act_pb3, act_pb2, act_pb1, act_pb0};

        end else if ((act_state == ACT_PIPE || act_state == ACT_DRAIN)
                     && act_s1_valid && act_pack_lane == 2'd3) begin
            automatic logic [7:0] bval_c;
            bval_c = act_s1_is_pad ? 8'h00
                                   : ext_x_data_i[int'(act_s1_byte_lane)*8 +: 8];
            sram_a_csb0   = 1'b0;
            sram_a_web0   = 1'b0;
            sram_a_wmask0 = 4'hF;
            sram_a_addr0  = act_sram_wr;
            sram_a_din0   = {bval_c, act_pb2, act_pb1, act_pb0};
        end
    end

    // =========================================================================
    // Combinational: SRAM B write
    // Mirrors the SRAM A write logic but for weights.
    // =========================================================================
    always_comb begin
        sram_b_csb0   = 1'b1;
        sram_b_web0   = 1'b1;
        sram_b_wmask0 = 4'hF;
        sram_b_addr0  = '0;
        sram_b_din0   = '0;

        if (wt_state == WT_FLUSH) begin
            sram_b_csb0   = 1'b0;
            sram_b_web0   = 1'b0;
            sram_b_wmask0 = 4'hF;
            sram_b_addr0  = wt_sram_wr;
            sram_b_din0   = {wt_pb3, wt_pb2, wt_pb1, wt_pb0};

        end else if ((wt_state == WT_PIPE || wt_state == WT_DRAIN)
                     && wt_s1_valid && wt_pack_lane == 2'd3) begin
            automatic logic [7:0] bval_c;
            bval_c = ext_y_data_i[int'(wt_s1_byte_lane)*8 +: 8];
            sram_b_csb0   = 1'b0;
            sram_b_web0   = 1'b0;
            sram_b_wmask0 = 4'hF;
            sram_b_addr0  = wt_sram_wr;
            sram_b_din0   = {bval_c, wt_pb2, wt_pb1, wt_pb0};
        end
    end

endmodule
