// =============================================================================
// sysController.sv  -  top-level integration shell (conv version)
//
// New parameters vs FC version:
//   IN_H, IN_W   - input feature map spatial dimensions
//   IN_C         - input channels
//   OUT_C        - output channels (= REAL_N)
//   K_H, K_W     - kernel spatial dimensions
//   STRIDE       - convolution stride
//   PAD          - zero padding
//
// Derived automatically:
//   OUT_H = (IN_H + 2*PAD - K_H)/STRIDE + 1
//   OUT_W = (IN_W + 2*PAD - K_W)/STRIDE + 1
//   REAL_K = OUT_H * OUT_W   (im2col rows)
//   REAL_M = K_H * K_W * IN_C (im2col cols / inner dim)
//   REAL_N = OUT_C
// =============================================================================

module systolic_pipe_conv #(
    parameter  TILE_MAX  = 4,
    parameter  IN_H      = 4,
    parameter  IN_W      = 4,
    parameter  IN_C      = 1,
    parameter  OUT_C     = 1,
    parameter  K_H       = 3,
    parameter  K_W       = 3,
    parameter  STRIDE    = 1,
    parameter  PAD       = 0,
    parameter  AW        = 8,
    parameter  BIAS_BASE = 16'd0,

    // Derived - do not override
    parameter  OUT_H  = (IN_H + 2*PAD - K_H) / STRIDE + 1,
    parameter  OUT_W  = (IN_W + 2*PAD - K_W) / STRIDE + 1,
    parameter  REAL_K = OUT_H * OUT_W,
    parameter  REAL_M = K_H * K_W * IN_C,
    parameter  REAL_N = OUT_C
)(
    input  logic clk, rst,
    output logic done,

    input  logic        is_last_layer,
    input  logic        act_sel,
    input  logic        out_sel,
    input  logic        skip_x,

    input  logic [15:0] M_int,
    input  logic  [4:0] shift,

    // X DRAM (image)
    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    // Y DRAM (weights)
    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    // Z DRAM (output, last layer only)
    output logic [15:0] ext_z_addr_o,
    output logic        ext_z_wr_en_o,
    output logic [31:0] ext_z_data_o,
    output logic [3:0]  ext_z_wmask_o,

    // Bias DRAM
    output logic [15:0] ext_b_addr_o,
    output logic        ext_b_rd_en_o,
    input  logic [31:0] ext_b_data_i,

    // SRAM A
    output logic          sram_a_csb0, sram_a_web0,
    output logic [3:0]    sram_a_wmask0,
    output logic [AW-1:0] sram_a_addr0,
    output logic [31:0]   sram_a_din0,
    input  logic [31:0]   sram_a_dout0,
    output logic          sram_a_csb1,
    output logic [AW-1:0] sram_a_addr1,
    input  logic [31:0]   sram_a_dout1,

    // SRAM B
    output logic          sram_b_csb0, sram_b_web0,
    output logic [3:0]    sram_b_wmask0,
    output logic [AW-1:0] sram_b_addr0,
    output logic [31:0]   sram_b_din0,
    input  logic [31:0]   sram_b_dout0,
    output logic          sram_b_csb1,
    output logic [AW-1:0] sram_b_addr1,
    input  logic [31:0]   sram_b_dout1,

    // SRAM C
    output logic          sram_c_csb0, sram_c_web0,
    output logic [3:0]    sram_c_wmask0,
    output logic [AW-1:0] sram_c_addr0,
    output logic [31:0]   sram_c_din0,
    input  logic [31:0]   sram_c_dout0,
    output logic          sram_c_csb1,
    output logic [AW-1:0] sram_c_addr1,
    input  logic [31:0]   sram_c_dout1
);

    // -------------------------------------------------------------------------
    // Inter-module signals
    // -------------------------------------------------------------------------
    logic [$clog2(REAL_K):0]       ti_row_tile;
    logic [$clog2(REAL_N):0]       ti_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0] ti_eff_rows, ti_eff_cols;
    logic [5:0]    ti_words_a, ti_words_b;
    logic [AW-1:0] ti_sram_a_base;
    logic          ti_valid, ti_all_done;
    logic          ti_advance;

    logic          bl_done, bl_busy;
    logic signed [31:0] bias_rf [REAL_K];

    logic          dl_load_ready, dl_busy;

    logic          sr_done, sr_busy;
    logic signed [7:0] x_rf [TILE_MAX][REAL_M];
    logic signed [7:0] y_rf [REAL_M][TILE_MAX];

    logic          fs_done, fs_busy;
    logic signed [7:0] a_feed [TILE_MAX];
    logic signed [7:0] b_feed [TILE_MAX];
    logic          local_rst;

    logic signed [31:0] tile_z [TILE_MAX][TILE_MAX];

    logic          ow_done, ow_busy;

    logic          dl_sram_a_csb0, dl_sram_a_web0;
    logic [3:0]    dl_sram_a_wmask0;
    logic [AW-1:0] dl_sram_a_addr0;
    logic [31:0]   dl_sram_a_din0;
    logic          dl_sram_b_csb0, dl_sram_b_web0;
    logic [3:0]    dl_sram_b_wmask0;
    logic [AW-1:0] dl_sram_b_addr0;
    logic [31:0]   dl_sram_b_din0;

    logic          ow_sram_c_csb0, ow_sram_c_web0;
    logic [3:0]    ow_sram_c_wmask0;
    logic [AW-1:0] ow_sram_c_addr0;
    logic [31:0]   ow_sram_c_din0;
    logic          ow_sram_a_csb0, ow_sram_a_web0;
    logic [3:0]    ow_sram_a_wmask0;
    logic [AW-1:0] ow_sram_a_addr0;
    logic [31:0]   ow_sram_a_din0;

    logic          sr_sram_a_csb1;
    logic [AW-1:0] sr_sram_a_addr1;
    logic          sr_sram_b_csb1;
    logic [AW-1:0] sr_sram_b_addr1;
    logic          sr_sram_c_csb1;
    logic [AW-1:0] sr_sram_c_addr1;

    // -------------------------------------------------------------------------
    // Top-level FSM (identical structure to FC version)
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TS_RESET        = 4'd0,
        TS_LOAD_BIAS    = 4'd1,
        TS_LOAD_DRAM    = 4'd2,
        TS_SHADOW       = 4'd3,
        TS_FEED         = 4'd4,
        TS_WRITE        = 4'd5,
        TS_ADVANCE      = 4'd6,
        TS_DONE         = 4'd7,
        TS_ADVANCE_CHK  = 4'd8,
        TS_ADVANCE_CHK2 = 4'd9
    } ts_t;
    ts_t ts_state;

    logic bl_start, dl_start, sr_start, fs_start, ow_start, ti_start, ti_advance_pulse;
    logic state_entry, state_entry_d1;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) state_entry_d1 <= 1'b0;
        else     state_entry_d1 <= state_entry;
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ts_state         <= TS_RESET;
            done             <= 1'b0;
            bl_start         <= 1'b0;
            dl_start         <= 1'b0;
            sr_start         <= 1'b0;
            fs_start         <= 1'b0;
            ow_start         <= 1'b0;
            ti_start         <= 1'b0;
            ti_advance_pulse <= 1'b0;
            state_entry      <= 1'b1;
        end else begin
            bl_start         <= 1'b0;
            dl_start         <= 1'b0;
            sr_start         <= 1'b0;
            fs_start         <= 1'b0;
            ow_start         <= 1'b0;
            ti_start         <= 1'b0;
            ti_advance_pulse <= 1'b0;
            state_entry      <= 1'b0;

            case (ts_state)
                TS_RESET: begin
                    ti_start    <= 1'b1;
                    bl_start    <= 1'b1;
                    ts_state    <= TS_LOAD_BIAS;
                    state_entry <= 1'b1;
                end

                TS_LOAD_BIAS: begin
                    if (bl_done) begin
                        ts_state    <= TS_LOAD_DRAM;
                        state_entry <= 1'b1;
                    end
                end

                TS_LOAD_DRAM: begin
                    if (state_entry_d1) dl_start <= 1'b1;
                    if (dl_load_ready) begin
                        ts_state    <= TS_SHADOW;
                        state_entry <= 1'b1;
                    end
                end

                TS_SHADOW: begin
                    if (state_entry_d1) sr_start <= 1'b1;
                    if (sr_done) begin
                        ts_state    <= TS_FEED;
                        state_entry <= 1'b1;
                    end
                end

                TS_FEED: begin
                    if (state_entry_d1) fs_start <= 1'b1;
                    if (fs_done) begin
                        ts_state    <= TS_WRITE;
                        state_entry <= 1'b1;
                    end
                end

                TS_WRITE: begin
                    if (state_entry_d1) ow_start <= 1'b1;
                    if (ow_done) begin
                        ts_state    <= TS_ADVANCE;
                        state_entry <= 1'b1;
                    end
                end

                TS_ADVANCE: begin
                    ti_advance_pulse <= 1'b1;
                    ts_state         <= TS_ADVANCE_CHK;
                end

                TS_ADVANCE_CHK: begin
                    ts_state <= TS_ADVANCE_CHK2;
                end

                TS_ADVANCE_CHK2: begin
                    if (ti_all_done) begin
                        ts_state    <= TS_DONE;
                        state_entry <= 1'b1;
                    end else begin
                        ts_state    <= TS_LOAD_DRAM;
                        state_entry <= 1'b1;
                    end
                end

                TS_DONE: done <= 1'b1;

                default: begin
                    ts_state    <= TS_RESET;
                    state_entry <= 1'b1;
                end
            endcase
        end
    end

    assign ti_advance = ti_advance_pulse;

    // -------------------------------------------------------------------------
    // SRAM port 0 mux
    // -------------------------------------------------------------------------
    always_comb begin
        if (ts_state == TS_WRITE && !is_last_layer && out_sel) begin
            sram_a_csb0   = ow_sram_a_csb0;
            sram_a_web0   = ow_sram_a_web0;
            sram_a_wmask0 = ow_sram_a_wmask0;
            sram_a_addr0  = ow_sram_a_addr0;
            sram_a_din0   = ow_sram_a_din0;
        end else begin
            sram_a_csb0   = dl_sram_a_csb0;
            sram_a_web0   = dl_sram_a_web0;
            sram_a_wmask0 = dl_sram_a_wmask0;
            sram_a_addr0  = dl_sram_a_addr0;
            sram_a_din0   = dl_sram_a_din0;
        end

        sram_b_csb0   = dl_sram_b_csb0;
        sram_b_web0   = dl_sram_b_web0;
        sram_b_wmask0 = dl_sram_b_wmask0;
        sram_b_addr0  = dl_sram_b_addr0;
        sram_b_din0   = dl_sram_b_din0;

        sram_c_csb0   = ow_sram_c_csb0;
        sram_c_web0   = ow_sram_c_web0;
        sram_c_wmask0 = ow_sram_c_wmask0;
        sram_c_addr0  = ow_sram_c_addr0;
        sram_c_din0   = ow_sram_c_din0;
    end

    assign sram_a_csb1  = sr_sram_a_csb1;
    assign sram_a_addr1 = sr_sram_a_addr1;
    assign sram_b_csb1  = sr_sram_b_csb1;
    assign sram_b_addr1 = sr_sram_b_addr1;
    assign sram_c_csb1  = sr_sram_c_csb1;
    assign sram_c_addr1 = sr_sram_c_addr1;

    // -------------------------------------------------------------------------
    // Submodule instantiations
    // -------------------------------------------------------------------------
    tile_iter #(
        .TILE_MAX(TILE_MAX), .REAL_K(REAL_K), .REAL_M(REAL_M),
        .REAL_N(REAL_N), .AW(AW)
    ) u_tile_iter (
        .clk(clk), .rst(rst),
        .start_i(ti_start), .advance_i(ti_advance),
        .valid_o(ti_valid), .all_done_o(ti_all_done),
        .row_tile_o(ti_row_tile), .col_tile_o(ti_col_tile),
        .eff_rows_o(ti_eff_rows), .eff_cols_o(ti_eff_cols),
        .words_a_o(ti_words_a), .words_b_o(ti_words_b),
        .sram_a_base_o(ti_sram_a_base)
    );

    bias_loader #(
        .REAL_K(REAL_K), .BIAS_BASE(BIAS_BASE)
    ) u_bias_loader (
        .clk(clk), .rst(rst),
        .start_i(bl_start), .done_o(bl_done), .busy_o(bl_busy),
        .ext_b_addr_o(ext_b_addr_o), .ext_b_rd_en_o(ext_b_rd_en_o),
        .ext_b_data_i(ext_b_data_i),
        .bias_rf_o(bias_rf)
    );

    dram_loader #(
        .TILE_MAX(TILE_MAX), .REAL_K(REAL_K), .REAL_M(REAL_M), .REAL_N(REAL_N),
        .IN_H(IN_H), .IN_W(IN_W), .IN_C(IN_C),
        .K_H(K_H), .K_W(K_W), .STRIDE(STRIDE), .PAD(PAD), .AW(AW)
    ) u_dram_loader (
        .clk(clk), .rst(rst),
        .start_i(dl_start), .skip_x_i(skip_x),
        .load_ready_o(dl_load_ready), .busy_o(dl_busy),
        .row_tile_i(ti_row_tile), .col_tile_i(ti_col_tile),
        .eff_rows_i(ti_eff_rows), .eff_cols_i(ti_eff_cols),
        .words_a_i(ti_words_a), .words_b_i(ti_words_b),
        .sram_a_base_i(ti_sram_a_base),
        .ext_x_addr_o(ext_x_addr_o), .ext_x_rd_en_o(ext_x_rd_en_o), .ext_x_data_i(ext_x_data_i),
        .ext_y_addr_o(ext_y_addr_o), .ext_y_rd_en_o(ext_y_rd_en_o), .ext_y_data_i(ext_y_data_i),
        .sram_a_csb0(dl_sram_a_csb0), .sram_a_web0(dl_sram_a_web0),
        .sram_a_wmask0(dl_sram_a_wmask0), .sram_a_addr0(dl_sram_a_addr0), .sram_a_din0(dl_sram_a_din0),
        .sram_b_csb0(dl_sram_b_csb0), .sram_b_web0(dl_sram_b_web0),
        .sram_b_wmask0(dl_sram_b_wmask0), .sram_b_addr0(dl_sram_b_addr0), .sram_b_din0(dl_sram_b_din0)
    );

    sram_shadow_reader #(
        .TILE_MAX(TILE_MAX), .REAL_K(REAL_K), .REAL_M(REAL_M), .REAL_N(REAL_N), .AW(AW)
    ) u_shadow_reader (
        .clk(clk), .rst(rst),
        .start_i(sr_start), .done_o(sr_done), .busy_o(sr_busy),
        .act_sel_i(act_sel), .sram_a_base_i(ti_sram_a_base),
        .words_a_i(ti_words_a), .words_b_i(ti_words_b), .eff_cols_i(ti_eff_cols),
        .sram_a_csb1(sr_sram_a_csb1), .sram_a_addr1(sr_sram_a_addr1), .sram_a_dout1(sram_a_dout1),
        .sram_b_csb1(sr_sram_b_csb1), .sram_b_addr1(sr_sram_b_addr1), .sram_b_dout1(sram_b_dout1),
        .sram_c_csb1(sr_sram_c_csb1), .sram_c_addr1(sr_sram_c_addr1), .sram_c_dout1(sram_c_dout1),
        .x_rf_o(x_rf), .y_rf_o(y_rf)
    );

    feed_sequencer #(
        .TILE_MAX(TILE_MAX), .REAL_M(REAL_M)
    ) u_feed_seq (
        .clk(clk), .rst(rst),
        .start_i(fs_start), .done_o(fs_done), .busy_o(fs_busy),
        .eff_rows_i(ti_eff_rows), .eff_cols_i(ti_eff_cols),
        .x_rf_i(x_rf), .y_rf_i(y_rf),
        .a_feed_o(a_feed), .b_feed_o(b_feed),
        .local_rst_o(local_rst)
    );

    systolic_tiled #(
        .TILE_MAX(TILE_MAX)
    ) u_systolic (
        .clk(clk), .rst(local_rst),
        .a(a_feed), .b(b_feed),
        .c(tile_z)
    );

    output_writer #(
        .TILE_MAX(TILE_MAX), .REAL_K(REAL_K), .REAL_N(REAL_N), .AW(AW)
    ) u_output_writer (
        .clk(clk), .rst(rst),
        .start_i(ow_start), .done_o(ow_done), .busy_o(ow_busy),
        .is_last_layer(is_last_layer), .out_sel(out_sel),
        .M_int(M_int), .shift(shift),
        .row_tile_i(ti_row_tile), .col_tile_i(ti_col_tile),
        .eff_rows_i(ti_eff_rows), .eff_cols_i(ti_eff_cols),
        .tile_z_i(tile_z), .bias_rf_i(bias_rf),
        .ext_z_addr_o(ext_z_addr_o), .ext_z_wr_en_o(ext_z_wr_en_o),
        .ext_z_data_o(ext_z_data_o), .ext_z_wmask_o(ext_z_wmask_o),
        .sram_c_csb0(ow_sram_c_csb0), .sram_c_web0(ow_sram_c_web0),
        .sram_c_wmask0(ow_sram_c_wmask0), .sram_c_addr0(ow_sram_c_addr0), .sram_c_din0(ow_sram_c_din0),
        .sram_a_csb0(ow_sram_a_csb0), .sram_a_web0(ow_sram_a_web0),
        .sram_a_wmask0(ow_sram_a_wmask0), .sram_a_addr0(ow_sram_a_addr0), .sram_a_din0(ow_sram_a_din0)
    );

endmodule
