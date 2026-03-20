module output_writer #(
    parameter int TILE_MAX  = 4,
    parameter int REAL_K    = 6,
    parameter int REAL_N    = 6,
    parameter int AW        = 8
)(
    input  logic clk,
    input  logic rst,

    // Handshake
    input  logic start_i,
    output logic done_o,
    output logic busy_o,

    // Layer control
    input  logic is_last_layer,
    input  logic out_sel,        // 0=write to SRAM C, 1=write to SRAM A

    // Requantisation parameters
    input  logic [15:0] M_int,
    input  logic  [4:0] shift,

    // Tile geometry
    input  logic [$clog2(REAL_K):0]        row_tile_i,
    input  logic [$clog2(REAL_N):0]        col_tile_i,
    input  logic [$clog2(TILE_MAX+1)-1:0]  eff_rows_i,
    input  logic [$clog2(TILE_MAX+1)-1:0]  eff_cols_i,

    // Accumulator results from systolic array
    input  logic signed [31:0] tile_z_i [TILE_MAX][TILE_MAX],

    // Bias register file
    input  logic signed [31:0] bias_rf_i [REAL_K],

    // DRAM Z (32-bit wide, word-addressed) + byte write mask
    output logic [15:0] ext_z_addr_o,
    output logic        ext_z_wr_en_o,
    output logic [31:0] ext_z_data_o,
    output logic [3:0]  ext_z_wmask_o,   // NEW: byte write mask for DRAM Z

    // SRAM C port 0
    output logic          sram_c_csb0,
    output logic          sram_c_web0,
    output logic [3:0]    sram_c_wmask0,
    output logic [AW-1:0] sram_c_addr0,
    output logic [31:0]   sram_c_din0,

    // SRAM A port 0
    output logic          sram_a_csb0,
    output logic          sram_a_web0,
    output logic [3:0]    sram_a_wmask0,
    output logic [AW-1:0] sram_a_addr0,
    output logic [31:0]   sram_a_din0
);

    // -------------------------------------------------------------------------
    // simd_requant interface
    // -------------------------------------------------------------------------
    logic [3:0][31:0] rq_acc;
    logic [3:0][31:0] rq_bias;
    logic [3:0] [7:0] rq_out;

    simd_requant rq_unit (
        .clk(clk), .rst_n(~rst),
        .acc_in(rq_acc), .bias_in(rq_bias),
        .M_int(M_int), .shift(shift),
        .data_out(rq_out)
    );

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        OW_IDLE     = 3'd0,
        OW_LATCH    = 3'd1,
        OW_WRITE    = 3'd2,
        OW_OVERFLOW = 3'd3,
        OW_DONE     = 3'd4
    } ow_t;
    ow_t ow_state;

    logic [$clog2(TILE_MAX):0] cur_row;

    // Overflow word ? holds spilled bytes when a row crosses a word boundary
    logic [31:0]   overflow_word;
    logic [3:0]    overflow_mask;   // FIX: byte enables for overflow word
    logic [AW-1:0] overflow_addr;
    logic          has_overflow;

    // Latched geometry
    logic [$clog2(REAL_K):0]        lat_row_tile;
    logic [$clog2(REAL_N):0]        lat_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0]  lat_eff_rows;
    logic [$clog2(TILE_MAX+1)-1:0]  lat_eff_cols;

    logic [15:0]   row_start;
    logic [AW-1:0] word_addr;

    // -------------------------------------------------------------------------
    // Sequential
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ow_state      <= OW_IDLE;
            cur_row       <= '0;
            lat_row_tile  <= '0;
            lat_col_tile  <= '0;
            lat_eff_rows  <= '0;
            lat_eff_cols  <= '0;
            row_start     <= '0;
            word_addr     <= '0;
            overflow_word <= '0;
            overflow_mask <= '0;
            overflow_addr <= '0;
            has_overflow  <= 1'b0;
            done_o        <= 1'b0;
            busy_o        <= 1'b0;
        end else begin
            done_o <= 1'b0;

            case (ow_state)

                OW_IDLE: begin
                    busy_o <= 1'b0;
                    if (start_i) begin
                        lat_row_tile <= row_tile_i;
                        lat_col_tile <= col_tile_i;
                        lat_eff_rows <= eff_rows_i;
                        lat_eff_cols <= eff_cols_i;
                        cur_row      <= '0;
                        row_start    <= 16'(int'(row_tile_i) * REAL_N + int'(col_tile_i));
                        word_addr    <= AW'((int'(row_tile_i) * REAL_N + int'(col_tile_i)) / 4);
                        busy_o       <= 1'b1;
                        ow_state     <= OW_LATCH;
                    end
                end

                // Present cur_row to requant; after one cycle rq_out is valid
                OW_LATCH: begin
                    ow_state <= OW_WRITE;
                end

                OW_WRITE: begin
                    // -----------------------------------------------------------
                    // Compute overflow: bytes that spill past lane 3
                    // Also compute overflow_mask for byte-enable writes
                    // -----------------------------------------------------------
                    begin
                        int sl;
                        sl = int'(row_start) % 4;
                        overflow_word <= '0;
                        overflow_mask <= 4'h0;
                        has_overflow  <= 1'b0;
                        for (int j = 0; j < 4; j++) begin
                            if (j < int'(lat_eff_cols) && (sl + j) >= 4) begin
                                overflow_word[(sl+j-4)*8 +: 8] <= rq_out[j];
                                overflow_mask[sl+j-4]          <= 1'b1;  // FIX: track mask
                                has_overflow                   <= 1'b1;
                            end
                        end
                        overflow_addr <= AW'(int'(word_addr) + 1);
                    end

                    begin
                        int sl2;
                        sl2 = int'(row_start) % 4;
                        if (sl2 + int'(lat_eff_cols) > 4) begin
                            // Overflow bytes exist: go to OW_OVERFLOW.
                            // Do NOT advance cur_row/row_start/word_addr here ?
                            // OW_OVERFLOW is responsible for that advancement.
                            ow_state <= OW_OVERFLOW;
                        end else if (int'(cur_row) == int'(lat_eff_rows) - 1) begin
                            // Last row, no overflow ? done
                            done_o   <= 1'b1;
                            busy_o   <= 1'b0;
                            ow_state <= OW_DONE;
                        end else begin
                            // Non-overflow: advance to next row now
                            cur_row   <= $bits(cur_row)'(int'(cur_row) + 1);
                            row_start <= 16'(int'(row_start) + REAL_N);
                            word_addr <= AW'((int'(row_start) + REAL_N) / 4);
                            ow_state  <= OW_LATCH;
                        end
                    end
                end

                OW_OVERFLOW: begin
                    // FIX: advance cur_row, row_start, word_addr here
                    // (previously missing ? caused same row to be re-processed)
                    if (int'(cur_row) == int'(lat_eff_rows) - 1) begin
                        done_o   <= 1'b1;
                        busy_o   <= 1'b0;
                        ow_state <= OW_DONE;
                    end else begin
                        cur_row   <= $bits(cur_row)'(int'(cur_row) + 1);
                        row_start <= 16'(int'(row_start) + REAL_N);
                        word_addr <= AW'((int'(row_start) + REAL_N) / 4);
                        ow_state  <= OW_LATCH;
                    end
                end

                OW_DONE: begin
                    done_o   <= 1'b0;
                    ow_state <= OW_IDLE;
                end

                default: ow_state <= OW_IDLE;

            endcase
        end
    end

    logic [31:0]   ow_word;
    logic [3:0]    ow_mask;    // FIX: per-lane write mask
    logic [AW-1:0] ow_waddr;

    // -------------------------------------------------------------------------
    // Combinational outputs
    // -------------------------------------------------------------------------
    always_comb begin
        // Drive requant with current row
        for (int j = 0; j < 4; j++) begin
            rq_acc[j]  = (j < int'(lat_eff_cols))
                         ? tile_z_i[cur_row][j] : 32'd0;
            rq_bias[j] = (j < int'(lat_eff_cols))
                         ? bias_rf_i[int'(lat_row_tile) + int'(cur_row)] : 32'd0;
        end

        // Default outputs
        ext_z_wr_en_o = 1'b0;
        ext_z_addr_o  = '0;
        ext_z_data_o  = '0;
        ext_z_wmask_o = 4'h0;
        sram_c_csb0   = 1'b1; sram_c_web0 = 1'b1;
        sram_c_wmask0 = 4'hF; sram_c_addr0 = '0; sram_c_din0 = '0;
        sram_a_csb0   = 1'b1; sram_a_web0 = 1'b1;
        sram_a_wmask0 = 4'hF; sram_a_addr0 = '0; sram_a_din0 = '0;

        ow_word  = '0;
        ow_mask  = 4'h0;
        ow_waddr = word_addr;

        if (ow_state == OW_WRITE || ow_state == OW_OVERFLOW) begin
            automatic int start_lane;
            start_lane = int'(row_start) % 4;

            if (ow_state == OW_WRITE) begin
                // FIX: pack only the active lanes and build matching byte mask
                ow_word  = '0;
                ow_mask  = 4'h0;
                for (int j = 0; j < 4; j++) begin
                    automatic int lane = start_lane + j;
                    if (j < int'(lat_eff_cols) && lane < 4) begin
                        ow_word[lane*8 +: 8] = rq_out[j];
                        ow_mask[lane]        = 1'b1;
                    end
                end
                ow_waddr = word_addr;
            end else begin
                // OW_OVERFLOW: use registered overflow word and its mask
                ow_word  = overflow_word;
                ow_mask  = overflow_mask;   // FIX: use exact byte mask
                ow_waddr = overflow_addr;
            end

            if (is_last_layer) begin
                ext_z_wr_en_o = 1'b1;
                ext_z_addr_o  = {8'b0, ow_waddr};
                ext_z_data_o  = ow_word;
                ext_z_wmask_o = ow_mask;    // FIX: expose mask to testbench
            end else if (!out_sel) begin
                sram_c_csb0   = 1'b0;
                sram_c_web0   = 1'b0;
                sram_c_wmask0 = ow_mask;    // FIX: byte-masked write
                sram_c_addr0  = ow_waddr;
                sram_c_din0   = ow_word;
            end else begin
                sram_a_csb0   = 1'b0;
                sram_a_web0   = 1'b0;
                sram_a_wmask0 = ow_mask;    // FIX: byte-masked write
                sram_a_addr0  = ow_waddr;
                sram_a_din0   = ow_word;
            end
        end
    end

endmodule
