// =============================================================================
// systolic_pipe_conv.sv  ?  top-level shell
//
// Three submodules:
//   bias_loader     ? loads bias values from DRAM once per inference
//   dram_loader     ? fetches image (im2col) + weights from DRAM each tile
//   compute_engine  ? owns everything else: tile iteration, master FSM,
//                     SRAM read, feed, compute, write
//
// This module is purely wiring ? no logic of its own.
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

    // Derived ? do not override
    parameter  OUT_H  = (IN_H + 2*PAD - K_H) / STRIDE + 1,
    parameter  OUT_W  = (IN_W + 2*PAD - K_W) / STRIDE + 1,
    parameter  REAL_K = OUT_H * OUT_W,
    parameter  REAL_M = K_H * K_W * IN_C,
    parameter  REAL_N = OUT_C
)(
    input  logic clk,
    input  logic rst,
    output logic done,

    input  logic is_last_layer,
    input  logic act_sel,
    input  logic out_sel,
    input  logic skip_x,

    // X DRAM ? image
    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    // Y DRAM ? weights
    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    // Z DRAM ? output (last layer only)
    output logic [15:0] ext_z_addr_o,
    output logic        ext_z_wr_en_o,
    output logic [31:0] ext_z_data_o,
    output logic [3:0]  ext_z_wmask_o,

    // B DRAM ? biases
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
    // Internal wires between submodules
    // -------------------------------------------------------------------------

    // bias_loader ? compute_engine
    logic        bl_start, bl_done;
    logic signed [31:0] bias_rf [REAL_K];

    // compute_engine ? dram_loader (tile geometry + control)
    logic        dl_start, dl_load_ready;
    logic [$clog2(REAL_K):0]        dl_row_tile;
    logic [$clog2(REAL_N):0]        dl_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0]  dl_eff_rows, dl_eff_cols;
    logic [5:0]    dl_words_a, dl_words_b;
    logic [AW-1:0] dl_sram_a_base;

    // dram_loader ? compute_engine (SRAM port 0 signals, muxed inside CE)
    logic          dl_sram_a_csb0, dl_sram_a_web0;
    logic [3:0]    dl_sram_a_wmask0;
    logic [AW-1:0] dl_sram_a_addr0;
    logic [31:0]   dl_sram_a_din0;
    logic          dl_sram_b_csb0, dl_sram_b_web0;
    logic [3:0]    dl_sram_b_wmask0;
    logic [AW-1:0] dl_sram_b_addr0;
    logic [31:0]   dl_sram_b_din0;

    // -------------------------------------------------------------------------
    // bias_loader
    // -------------------------------------------------------------------------
    bias_loader #(
        .REAL_K(REAL_K), .BIAS_BASE(BIAS_BASE)
    ) u_bias_loader (
        .clk(clk), .rst(rst),
        .start_i(bl_start), .done_o(bl_done), .busy_o(),
        .ext_b_addr_o(ext_b_addr_o), .ext_b_rd_en_o(ext_b_rd_en_o),
        .ext_b_data_i(ext_b_data_i),
        .bias_rf_o(bias_rf)
    );

    // -------------------------------------------------------------------------
    // dram_loader
    // -------------------------------------------------------------------------
    dram_loader #(
        .TILE_MAX(TILE_MAX), .REAL_K(REAL_K), .REAL_M(REAL_M), .REAL_N(REAL_N),
        .IN_H(IN_H), .IN_W(IN_W), .IN_C(IN_C),
        .K_H(K_H), .K_W(K_W), .STRIDE(STRIDE), .PAD(PAD), .AW(AW)
    ) u_dram_loader (
        .clk(clk), .rst(rst),
        .start_i(dl_start), .skip_x_i(skip_x),
        .load_ready_o(dl_load_ready), .busy_o(),
        .row_tile_i(dl_row_tile), .col_tile_i(dl_col_tile),
        .eff_rows_i(dl_eff_rows), .eff_cols_i(dl_eff_cols),
        .words_a_i(dl_words_a), .words_b_i(dl_words_b),
        .sram_a_base_i(dl_sram_a_base),
        .ext_x_addr_o(ext_x_addr_o), .ext_x_rd_en_o(ext_x_rd_en_o),
        .ext_x_data_i(ext_x_data_i),
        .ext_y_addr_o(ext_y_addr_o), .ext_y_rd_en_o(ext_y_rd_en_o),
        .ext_y_data_i(ext_y_data_i),
        .sram_a_csb0(dl_sram_a_csb0), .sram_a_web0(dl_sram_a_web0),
        .sram_a_wmask0(dl_sram_a_wmask0), .sram_a_addr0(dl_sram_a_addr0),
        .sram_a_din0(dl_sram_a_din0),
        .sram_b_csb0(dl_sram_b_csb0), .sram_b_web0(dl_sram_b_web0),
        .sram_b_wmask0(dl_sram_b_wmask0), .sram_b_addr0(dl_sram_b_addr0),
        .sram_b_din0(dl_sram_b_din0)
    );

    // -------------------------------------------------------------------------
    // compute_engine
    // -------------------------------------------------------------------------
    compute_engine #(
        .TILE_MAX(TILE_MAX), .REAL_K(REAL_K), .REAL_M(REAL_M),
        .REAL_N(REAL_N), .AW(AW), .BIAS_BASE(BIAS_BASE)
    ) u_compute_engine (
        .clk(clk), .rst(rst),
        .done(done),
        .is_last_layer(is_last_layer), .act_sel(act_sel),
        .out_sel(out_sel), .skip_x(skip_x),

        // bias loader interface
        .bl_start_o(bl_start), .bl_done_i(bl_done),
        .bias_rf_i(bias_rf),

        // dram loader interface
        .dl_start_o(dl_start), .dl_load_ready_i(dl_load_ready),
        .dl_row_tile_o(dl_row_tile), .dl_col_tile_o(dl_col_tile),
        .dl_eff_rows_o(dl_eff_rows), .dl_eff_cols_o(dl_eff_cols),
        .dl_words_a_o(dl_words_a), .dl_words_b_o(dl_words_b),
        .dl_sram_a_base_o(dl_sram_a_base),

        // dram_loader SRAM port 0 signals (muxed inside compute_engine)
        .dl_sram_a_csb0(dl_sram_a_csb0), .dl_sram_a_web0(dl_sram_a_web0),
        .dl_sram_a_wmask0(dl_sram_a_wmask0), .dl_sram_a_addr0(dl_sram_a_addr0),
        .dl_sram_a_din0(dl_sram_a_din0),
        .dl_sram_b_csb0(dl_sram_b_csb0), .dl_sram_b_web0(dl_sram_b_web0),
        .dl_sram_b_wmask0(dl_sram_b_wmask0), .dl_sram_b_addr0(dl_sram_b_addr0),
        .dl_sram_b_din0(dl_sram_b_din0),

        // SRAM port 0 outputs (muxed)
        .sram_a_csb0(sram_a_csb0), .sram_a_web0(sram_a_web0),
        .sram_a_wmask0(sram_a_wmask0), .sram_a_addr0(sram_a_addr0),
        .sram_a_din0(sram_a_din0),
        .sram_b_csb0(sram_b_csb0), .sram_b_web0(sram_b_web0),
        .sram_b_wmask0(sram_b_wmask0), .sram_b_addr0(sram_b_addr0),
        .sram_b_din0(sram_b_din0),
        .sram_c_csb0(sram_c_csb0), .sram_c_web0(sram_c_web0),
        .sram_c_wmask0(sram_c_wmask0), .sram_c_addr0(sram_c_addr0),
        .sram_c_din0(sram_c_din0),

        // SRAM port 1 (read)
        .sram_a_csb1(sram_a_csb1), .sram_a_addr1(sram_a_addr1),
        .sram_a_dout1(sram_a_dout1),
        .sram_b_csb1(sram_b_csb1), .sram_b_addr1(sram_b_addr1),
        .sram_b_dout1(sram_b_dout1),
        .sram_c_csb1(sram_c_csb1), .sram_c_addr1(sram_c_addr1),
        .sram_c_dout1(sram_c_dout1),

        // DRAM Z output
        .ext_z_addr_o(ext_z_addr_o), .ext_z_wr_en_o(ext_z_wr_en_o),
        .ext_z_data_o(ext_z_data_o), .ext_z_wmask_o(ext_z_wmask_o)
    );

endmodule
