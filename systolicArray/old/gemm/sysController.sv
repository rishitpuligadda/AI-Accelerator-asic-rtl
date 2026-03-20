// =============================================================================
// systolic_pipe_relu.sv  ?  top-level integration shell
// =============================================================================

module systolic_pipe_relu #(
    parameter  TILE_MAX  = 4,
    parameter  REAL_K    = 6,
    parameter  REAL_M    = 6,
    parameter  REAL_N    = 6,
    parameter  AW        = 8,
    parameter  BIAS_BASE = 16'd0
)(
    input  logic clk, rst,
    output logic done,

    // Layer control
    input  logic        is_last_layer,
    input  logic        act_sel,
    input  logic        out_sel,
    input  logic        skip_x,

    input  logic [15:0] M_int,
    input  logic  [4:0] shift,

    // X DRAM
    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    // Y DRAM
    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    // Z DRAM ? FIX: added ext_z_wmask_o for byte-masked writes
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

    // tile_iter outputs
    logic [$clog2(REAL_K):0]       ti_row_tile;
    logic [$clog2(REAL_N):0]       ti_col_tile;
    logic [$clog2(TILE_MAX+1)-1:0] ti_eff_rows, ti_eff_cols;
    logic [5:0]    ti_words_a, ti_words_b;
    logic [AW-1:0] ti_sram_a_base;
    logic          ti_valid, ti_all_done;
    logic          ti_advance;

    // bias_loader outputs
    logic          bl_done, bl_busy;
    logic signed [31:0] bias_rf [REAL_K];

    // dram_loader outputs
    logic          dl_load_ready, dl_busy;

    // sram_shadow_reader outputs
    logic          sr_done, sr_busy;
    logic signed [7:0] x_rf [TILE_MAX][REAL_M];
    logic signed [7:0] y_rf [REAL_M][TILE_MAX];

    // feed_sequencer outputs
    logic          fs_done, fs_busy;
    logic signed [7:0] a_feed [TILE_MAX];
    logic signed [7:0] b_feed [TILE_MAX];
    logic          local_rst;

    // systolic_tiled outputs
    logic signed [31:0] tile_z [TILE_MAX][TILE_MAX];

    // output_writer outputs
    logic          ow_done, ow_busy;

    // SRAM port 0 arbitration signals
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

    // sram_shadow_reader port 1
    logic          sr_sram_a_csb1;
    logic [AW-1:0] sr_sram_a_addr1;
    logic          sr_sram_b_csb1;
    logic [AW-1:0] sr_sram_b_addr1;
    logic          sr_sram_c_csb1;
    logic [AW-1:0] sr_sram_c_addr1;

    // -------------------------------------------------------------------------
    // Top-level FSM
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TS_RESET       = 4'd0,
        TS_LOAD_BIAS   = 4'd1,
        TS_LOAD_DRAM   = 4'd2,
        TS_SHADOW      = 4'd3,
        TS_FEED        = 4'd4,
        TS_WRITE       = 4'd5,
        TS_ADVANCE     = 4'd6,
        TS_DONE        = 4'd7,
        TS_ADVANCE_CHK = 4'd8,   // cycle N+1: tile_iter registers the advance
        TS_ADVANCE_CHK2= 4'd9    // cycle N+2: all_done_o is now stable and readable
    } ts_t;
    ts_t ts_state;

    logic bl_start, dl_start, sr_start, fs_start, ow_start, ti_start, ti_advance_pulse;
    logic state_entry;
    logic state_entry_d1;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state_entry_d1 <= 1'b0;
        end else begin
            state_entry_d1 <= state_entry;
        end
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
                    // Cycle N: pulse advance to tile_iter.
                    // tile_iter sees the pulse at posedge N and REGISTERS new
                    // state (including all_done_o) ? that takes effect at posedge N+1.
                    // We must therefore wait TWO cycles after the pulse.
                    ti_advance_pulse <= 1'b1;
                    ts_state         <= TS_ADVANCE_CHK;
                end

                TS_ADVANCE_CHK: begin
                    // Cycle N+1: tile_iter has registered the advance.
                    // all_done_o will be stable after posedge N+1 (i.e. readable
                    // combinationally in cycle N+2 but NOT yet here because it
                    // was just registered this same edge).
                    ts_state <= TS_ADVANCE_CHK2;
                end

                TS_ADVANCE_CHK2: begin
                    // Cycle N+2: all_done_o is now the fully-settled registered value.
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
        // SRAM A port 0
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

        // SRAM B port 0 ? only dram_loader writes it
        sram_b_csb0   = dl_sram_b_csb0;
        sram_b_web0   = dl_sram_b_web0;
        sram_b_wmask0 = dl_sram_b_wmask0;
        sram_b_addr0  = dl_sram_b_addr0;
        sram_b_din0   = dl_sram_b_din0;

        // SRAM C port 0 ? only output_writer writes it
        sram_c_csb0   = ow_sram_c_csb0;
        sram_c_web0   = ow_sram_c_web0;
        sram_c_wmask0 = ow_sram_c_wmask0;
        sram_c_addr0  = ow_sram_c_addr0;
        sram_c_din0   = ow_sram_c_din0;
    end

    // SRAM port 1 ? only sram_shadow_reader reads
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
        .TILE_MAX(TILE_MAX),.REAL_K(REAL_K),.REAL_M(REAL_M),.REAL_N(REAL_N),.AW(AW)
    ) u_tile_iter (
        .clk(clk),.rst(rst),
        .start_i(ti_start),.advance_i(ti_advance),
        .valid_o(ti_valid),.all_done_o(ti_all_done),
        .row_tile_o(ti_row_tile),.col_tile_o(ti_col_tile),
        .eff_rows_o(ti_eff_rows),.eff_cols_o(ti_eff_cols),
        .words_a_o(ti_words_a),.words_b_o(ti_words_b),
        .sram_a_base_o(ti_sram_a_base)
    );

    bias_loader #(
        .REAL_K(REAL_K),.BIAS_BASE(BIAS_BASE)
    ) u_bias_loader (
        .clk(clk),.rst(rst),
        .start_i(bl_start),.done_o(bl_done),.busy_o(bl_busy),
        .ext_b_addr_o(ext_b_addr_o),.ext_b_rd_en_o(ext_b_rd_en_o),
        .ext_b_data_i(ext_b_data_i),
        .bias_rf_o(bias_rf)
    );

    dram_loader #(
        .TILE_MAX(TILE_MAX),.REAL_K(REAL_K),.REAL_M(REAL_M),.REAL_N(REAL_N),.AW(AW)
    ) u_dram_loader (
        .clk(clk),.rst(rst),
        .start_i(dl_start),.skip_x_i(skip_x),
        .load_ready_o(dl_load_ready),.busy_o(dl_busy),
        .row_tile_i(ti_row_tile),.col_tile_i(ti_col_tile),
        .eff_rows_i(ti_eff_rows),.eff_cols_i(ti_eff_cols),
        .words_a_i(ti_words_a),.words_b_i(ti_words_b),
        .sram_a_base_i(ti_sram_a_base),
        .ext_x_addr_o(ext_x_addr_o),.ext_x_rd_en_o(ext_x_rd_en_o),.ext_x_data_i(ext_x_data_i),
        .ext_y_addr_o(ext_y_addr_o),.ext_y_rd_en_o(ext_y_rd_en_o),.ext_y_data_i(ext_y_data_i),
        .sram_a_csb0(dl_sram_a_csb0),.sram_a_web0(dl_sram_a_web0),
        .sram_a_wmask0(dl_sram_a_wmask0),.sram_a_addr0(dl_sram_a_addr0),.sram_a_din0(dl_sram_a_din0),
        .sram_b_csb0(dl_sram_b_csb0),.sram_b_web0(dl_sram_b_web0),
        .sram_b_wmask0(dl_sram_b_wmask0),.sram_b_addr0(dl_sram_b_addr0),.sram_b_din0(dl_sram_b_din0)
    );

    sram_shadow_reader #(
        .TILE_MAX(TILE_MAX),.REAL_K(REAL_K),.REAL_M(REAL_M),.REAL_N(REAL_N),.AW(AW)
    ) u_shadow_reader (
        .clk(clk),.rst(rst),
        .start_i(sr_start),.done_o(sr_done),.busy_o(sr_busy),
        .act_sel_i(act_sel),.sram_a_base_i(ti_sram_a_base),
        .words_a_i(ti_words_a),.words_b_i(ti_words_b),.eff_cols_i(ti_eff_cols),
        .sram_a_csb1(sr_sram_a_csb1),.sram_a_addr1(sr_sram_a_addr1),.sram_a_dout1(sram_a_dout1),
        .sram_b_csb1(sr_sram_b_csb1),.sram_b_addr1(sr_sram_b_addr1),.sram_b_dout1(sram_b_dout1),
        .sram_c_csb1(sr_sram_c_csb1),.sram_c_addr1(sr_sram_c_addr1),.sram_c_dout1(sram_c_dout1),
        .x_rf_o(x_rf),.y_rf_o(y_rf)
    );

    feed_sequencer #(
        .TILE_MAX(TILE_MAX),.REAL_M(REAL_M)
    ) u_feed_seq (
        .clk(clk),.rst(rst),
        .start_i(fs_start),.done_o(fs_done),.busy_o(fs_busy),
        .eff_rows_i(ti_eff_rows),.eff_cols_i(ti_eff_cols),
        .x_rf_i(x_rf),.y_rf_i(y_rf),
        .a_feed_o(a_feed),.b_feed_o(b_feed),
        .local_rst_o(local_rst)
    );

    systolic_tiled #(
        .TILE_MAX(TILE_MAX)
    ) u_systolic (
        .clk(clk),.rst(local_rst),
        .a(a_feed),.b(b_feed),
        .c(tile_z)
    );

    output_writer #(
        .TILE_MAX(TILE_MAX),.REAL_K(REAL_K),.REAL_N(REAL_N),.AW(AW)
    ) u_output_writer (
        .clk(clk),.rst(rst),
        .start_i(ow_start),.done_o(ow_done),.busy_o(ow_busy),
        .is_last_layer(is_last_layer),.out_sel(out_sel),
        .M_int(M_int),.shift(shift),
        .row_tile_i(ti_row_tile),.col_tile_i(ti_col_tile),
        .eff_rows_i(ti_eff_rows),.eff_cols_i(ti_eff_cols),
        .tile_z_i(tile_z),.bias_rf_i(bias_rf),
        .ext_z_addr_o(ext_z_addr_o),.ext_z_wr_en_o(ext_z_wr_en_o),
        .ext_z_data_o(ext_z_data_o),.ext_z_wmask_o(ext_z_wmask_o),  // FIX: wire mask
        .sram_c_csb0(ow_sram_c_csb0),.sram_c_web0(ow_sram_c_web0),
        .sram_c_wmask0(ow_sram_c_wmask0),.sram_c_addr0(ow_sram_c_addr0),.sram_c_din0(ow_sram_c_din0),
        .sram_a_csb0(ow_sram_a_csb0),.sram_a_web0(ow_sram_a_web0),
        .sram_a_wmask0(ow_sram_a_wmask0),.sram_a_addr0(ow_sram_a_addr0),.sram_a_din0(ow_sram_a_din0)
    );

endmodule
// =============================================================================
// tb_3layer.sv  ?  3-layer systolic inference testbench
//
// Test A  (3-layer chain, non-square tiles)
//   Layer 0: X(6x8) @ W(8x6) -> Z1(6x6)   REAL_K=6 REAL_M=8 REAL_N=6
//            act_sel=0  out_sel=0  skip_x=0  is_last=0  M=73  shift=15
//   Layer 1: Z1(6x6) @ W(6x4) -> Z2(6x4)  REAL_K=6 REAL_M=6 REAL_N=4
//            act_sel=1  out_sel=1  skip_x=1  is_last=0  M=55  shift=15
//   Layer 2: Z2(6x4) @ W(4x5) -> Z3(6x5)  REAL_K=6 REAL_M=4 REAL_N=5
//            act_sel=0  out_sel=0  skip_x=1  is_last=1  M=45  shift=15
//   Gold Z3 independently verified with Python requant model.
//
// Test B  (single layer, perfectly-tiled, no edge cases)
//   Layer:   X(4x6) @ W(6x4) -> Z(4x4)   REAL_K=4 REAL_M=6 REAL_N=4
//            act_sel=0  out_sel=0  skip_x=0  is_last=1  M=60  shift=15
//   Gold independently verified with Python requant model.
//
// DRAM layout:
//   X / Y  ? 32-bit word-addressed, byte-packed row-major (set_x_byte/set_y_byte)
//   Bias   ? 32-bit signed word per output row
//   Z      ? 32-bit word-addressed, byte-masked writes from output_writer
// =============================================================================

`timescale 1ns/1ps

module tb_3layer;

    localparam int TILE_MAX = 4;
    localparam int AW       = 8;

    logic clk = 0;
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // DRAM models ? 32-bit word-addressed
    // -------------------------------------------------------------------------
    reg [31:0] dram_x [0:1023];
    reg [31:0] dram_y [0:1023];
    reg [31:0] dram_b [0:1023];
    reg [31:0] dram_z [0:1023];

    // -------------------------------------------------------------------------
    // Layer select (selects which DUT drives the shared SRAMs)
    // -------------------------------------------------------------------------
    logic [1:0] layer_sel;

    // -------------------------------------------------------------------------
    // DUT 0 ? Test A Layer 0  (REAL_K=6 REAL_M=8 REAL_N=6)
    // -------------------------------------------------------------------------
    logic        rst0, done0;
    logic [15:0] x0a, y0a, z0a, b0a;
    logic        x0e, y0e, z0e, b0e;
    logic [3:0]  z0m;
    logic [31:0] x0d, y0d, z0d, b0d;
    logic d0_sa_csb0, d0_sa_web0; logic [3:0] d0_sa_wm; logic [AW-1:0] d0_sa_a0; logic [31:0] d0_sa_di;
    logic d0_sa_csb1; logic [AW-1:0] d0_sa_a1;
    logic d0_sb_csb0, d0_sb_web0; logic [3:0] d0_sb_wm; logic [AW-1:0] d0_sb_a0; logic [31:0] d0_sb_di;
    logic d0_sb_csb1; logic [AW-1:0] d0_sb_a1;
    logic d0_sc_csb0, d0_sc_web0; logic [3:0] d0_sc_wm; logic [AW-1:0] d0_sc_a0; logic [31:0] d0_sc_di;
    logic d0_sc_csb1; logic [AW-1:0] d0_sc_a1;

    // -------------------------------------------------------------------------
    // DUT 1 ? Test A Layer 1  (REAL_K=6 REAL_M=6 REAL_N=4)
    // -------------------------------------------------------------------------
    logic        rst1, done1;
    logic [15:0] x1a, y1a, z1a, b1a;
    logic        x1e, y1e, z1e, b1e;
    logic [3:0]  z1m;
    logic [31:0] x1d, y1d, z1d, b1d;
    logic d1_sa_csb0, d1_sa_web0; logic [3:0] d1_sa_wm; logic [AW-1:0] d1_sa_a0; logic [31:0] d1_sa_di;
    logic d1_sa_csb1; logic [AW-1:0] d1_sa_a1;
    logic d1_sb_csb0, d1_sb_web0; logic [3:0] d1_sb_wm; logic [AW-1:0] d1_sb_a0; logic [31:0] d1_sb_di;
    logic d1_sb_csb1; logic [AW-1:0] d1_sb_a1;
    logic d1_sc_csb0, d1_sc_web0; logic [3:0] d1_sc_wm; logic [AW-1:0] d1_sc_a0; logic [31:0] d1_sc_di;
    logic d1_sc_csb1; logic [AW-1:0] d1_sc_a1;

    // -------------------------------------------------------------------------
    // DUT 2 ? Test A Layer 2  (REAL_K=6 REAL_M=4 REAL_N=5)
    // -------------------------------------------------------------------------
    logic        rst2, done2;
    logic [15:0] x2a, y2a, z2a, b2a;
    logic        x2e, y2e, z2e, b2e;
    logic [3:0]  z2m;
    logic [31:0] x2d, y2d, z2d, b2d;
    logic d2_sa_csb0, d2_sa_web0; logic [3:0] d2_sa_wm; logic [AW-1:0] d2_sa_a0; logic [31:0] d2_sa_di;
    logic d2_sa_csb1; logic [AW-1:0] d2_sa_a1;
    logic d2_sb_csb0, d2_sb_web0; logic [3:0] d2_sb_wm; logic [AW-1:0] d2_sb_a0; logic [31:0] d2_sb_di;
    logic d2_sb_csb1; logic [AW-1:0] d2_sb_a1;
    logic d2_sc_csb0, d2_sc_web0; logic [3:0] d2_sc_wm; logic [AW-1:0] d2_sc_a0; logic [31:0] d2_sc_di;
    logic d2_sc_csb1; logic [AW-1:0] d2_sc_a1;

    // -------------------------------------------------------------------------
    // DUT 3 ? Test B  (REAL_K=4 REAL_M=6 REAL_N=4, single layer)
    // -------------------------------------------------------------------------
    logic        rst3, done3;
    logic [15:0] x3a, y3a, z3a, b3a;
    logic        x3e, y3e, z3e, b3e;
    logic [3:0]  z3m;
    logic [31:0] x3d, y3d, z3d, b3d;
    logic d3_sa_csb0, d3_sa_web0; logic [3:0] d3_sa_wm; logic [AW-1:0] d3_sa_a0; logic [31:0] d3_sa_di;
    logic d3_sa_csb1; logic [AW-1:0] d3_sa_a1;
    logic d3_sb_csb0, d3_sb_web0; logic [3:0] d3_sb_wm; logic [AW-1:0] d3_sb_a0; logic [31:0] d3_sb_di;
    logic d3_sb_csb1; logic [AW-1:0] d3_sb_a1;
    logic d3_sc_csb0, d3_sc_web0; logic [3:0] d3_sc_wm; logic [AW-1:0] d3_sc_a0; logic [31:0] d3_sc_di;
    logic d3_sc_csb1; logic [AW-1:0] d3_sc_a1;

    // -------------------------------------------------------------------------
    // Shared SRAM outputs
    // -------------------------------------------------------------------------
    logic [31:0] sa_do0, sa_do1, sb_do0, sb_do1, sc_do0, sc_do1;

    // -------------------------------------------------------------------------
    // SRAM control mux ? layer_sel routes active DUT to shared SRAMs
    // -------------------------------------------------------------------------
    logic sa_csb0, sa_web0; logic [3:0] sa_wm; logic [AW-1:0] sa_a0; logic [31:0] sa_di;
    logic sa_csb1; logic [AW-1:0] sa_a1;
    logic sb_csb0, sb_web0; logic [3:0] sb_wm; logic [AW-1:0] sb_a0; logic [31:0] sb_di;
    logic sb_csb1; logic [AW-1:0] sb_a1;
    logic sc_csb0, sc_web0; logic [3:0] sc_wm; logic [AW-1:0] sc_a0; logic [31:0] sc_di;
    logic sc_csb1; logic [AW-1:0] sc_a1;

    assign sa_csb0 = (layer_sel==3)?d3_sa_csb0:(layer_sel==2)?d2_sa_csb0:(layer_sel==1)?d1_sa_csb0:d0_sa_csb0;
    assign sa_web0 = (layer_sel==3)?d3_sa_web0:(layer_sel==2)?d2_sa_web0:(layer_sel==1)?d1_sa_web0:d0_sa_web0;
    assign sa_wm   = (layer_sel==3)?d3_sa_wm  :(layer_sel==2)?d2_sa_wm  :(layer_sel==1)?d1_sa_wm  :d0_sa_wm;
    assign sa_a0   = (layer_sel==3)?d3_sa_a0  :(layer_sel==2)?d2_sa_a0  :(layer_sel==1)?d1_sa_a0  :d0_sa_a0;
    assign sa_di   = (layer_sel==3)?d3_sa_di  :(layer_sel==2)?d2_sa_di  :(layer_sel==1)?d1_sa_di  :d0_sa_di;
    assign sa_csb1 = (layer_sel==3)?d3_sa_csb1:(layer_sel==2)?d2_sa_csb1:(layer_sel==1)?d1_sa_csb1:d0_sa_csb1;
    assign sa_a1   = (layer_sel==3)?d3_sa_a1  :(layer_sel==2)?d2_sa_a1  :(layer_sel==1)?d1_sa_a1  :d0_sa_a1;

    assign sb_csb0 = (layer_sel==3)?d3_sb_csb0:(layer_sel==2)?d2_sb_csb0:(layer_sel==1)?d1_sb_csb0:d0_sb_csb0;
    assign sb_web0 = (layer_sel==3)?d3_sb_web0:(layer_sel==2)?d2_sb_web0:(layer_sel==1)?d1_sb_web0:d0_sb_web0;
    assign sb_wm   = (layer_sel==3)?d3_sb_wm  :(layer_sel==2)?d2_sb_wm  :(layer_sel==1)?d1_sb_wm  :d0_sb_wm;
    assign sb_a0   = (layer_sel==3)?d3_sb_a0  :(layer_sel==2)?d2_sb_a0  :(layer_sel==1)?d1_sb_a0  :d0_sb_a0;
    assign sb_di   = (layer_sel==3)?d3_sb_di  :(layer_sel==2)?d2_sb_di  :(layer_sel==1)?d1_sb_di  :d0_sb_di;
    assign sb_csb1 = (layer_sel==3)?d3_sb_csb1:(layer_sel==2)?d2_sb_csb1:(layer_sel==1)?d1_sb_csb1:d0_sb_csb1;
    assign sb_a1   = (layer_sel==3)?d3_sb_a1  :(layer_sel==2)?d2_sb_a1  :(layer_sel==1)?d1_sb_a1  :d0_sb_a1;

    assign sc_csb0 = (layer_sel==3)?d3_sc_csb0:(layer_sel==2)?d2_sc_csb0:(layer_sel==1)?d1_sc_csb0:d0_sc_csb0;
    assign sc_web0 = (layer_sel==3)?d3_sc_web0:(layer_sel==2)?d2_sc_web0:(layer_sel==1)?d1_sc_web0:d0_sc_web0;
    assign sc_wm   = (layer_sel==3)?d3_sc_wm  :(layer_sel==2)?d2_sc_wm  :(layer_sel==1)?d1_sc_wm  :d0_sc_wm;
    assign sc_a0   = (layer_sel==3)?d3_sc_a0  :(layer_sel==2)?d2_sc_a0  :(layer_sel==1)?d1_sc_a0  :d0_sc_a0;
    assign sc_di   = (layer_sel==3)?d3_sc_di  :(layer_sel==2)?d2_sc_di  :(layer_sel==1)?d1_sc_di  :d0_sc_di;
    assign sc_csb1 = (layer_sel==3)?d3_sc_csb1:(layer_sel==2)?d2_sc_csb1:(layer_sel==1)?d1_sc_csb1:d0_sc_csb1;
    assign sc_a1   = (layer_sel==3)?d3_sc_a1  :(layer_sel==2)?d2_sc_a1  :(layer_sel==1)?d1_sc_a1  :d0_sc_a1;

    // -------------------------------------------------------------------------
    // sky130 dual-port SRAM instances (shared across all DUTs)
    // -------------------------------------------------------------------------
    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) sram_a (
        .clk0(clk), .csb0(sa_csb0), .web0(sa_web0), .wmask0(sa_wm),
        .addr0(sa_a0), .din0(sa_di), .dout0(sa_do0),
        .clk1(clk), .csb1(sa_csb1), .addr1(sa_a1), .dout1(sa_do1));

    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) sram_b (
        .clk0(clk), .csb0(sb_csb0), .web0(sb_web0), .wmask0(sb_wm),
        .addr0(sb_a0), .din0(sb_di), .dout0(sb_do0),
        .clk1(clk), .csb1(sb_csb1), .addr1(sb_a1), .dout1(sb_do1));

    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) sram_c (
        .clk0(clk), .csb0(sc_csb0), .web0(sc_web0), .wmask0(sc_wm),
        .addr0(sc_a0), .din0(sc_di), .dout0(sc_do0),
        .clk1(clk), .csb1(sc_csb1), .addr1(sc_a1), .dout1(sc_do1));

    // -------------------------------------------------------------------------
    // DRAM models ? registered read, byte-masked write for Z
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        case (layer_sel)
            2'd0: begin
                if (x0e) x0d <= dram_x[x0a];
                if (y0e) y0d <= dram_y[y0a];
                if (b0e) b0d <= dram_b[b0a];
                if (z0e) begin
                    if (z0m[0]) dram_z[z0a][ 7: 0] <= z0d[ 7: 0];
                    if (z0m[1]) dram_z[z0a][15: 8] <= z0d[15: 8];
                    if (z0m[2]) dram_z[z0a][23:16] <= z0d[23:16];
                    if (z0m[3]) dram_z[z0a][31:24] <= z0d[31:24];
                end
            end
            2'd1: begin
                if (x1e) x1d <= dram_x[x1a];
                if (y1e) y1d <= dram_y[y1a];
                if (b1e) b1d <= dram_b[b1a];
                if (z1e) begin
                    if (z1m[0]) dram_z[z1a][ 7: 0] <= z1d[ 7: 0];
                    if (z1m[1]) dram_z[z1a][15: 8] <= z1d[15: 8];
                    if (z1m[2]) dram_z[z1a][23:16] <= z1d[23:16];
                    if (z1m[3]) dram_z[z1a][31:24] <= z1d[31:24];
                end
            end
            2'd2: begin
                if (x2e) x2d <= dram_x[x2a];
                if (y2e) y2d <= dram_y[y2a];
                if (b2e) b2d <= dram_b[b2a];
                if (z2e) begin
                    if (z2m[0]) dram_z[z2a][ 7: 0] <= z2d[ 7: 0];
                    if (z2m[1]) dram_z[z2a][15: 8] <= z2d[15: 8];
                    if (z2m[2]) dram_z[z2a][23:16] <= z2d[23:16];
                    if (z2m[3]) dram_z[z2a][31:24] <= z2d[31:24];
                end
            end
            2'd3: begin
                if (x3e) x3d <= dram_x[x3a];
                if (y3e) y3d <= dram_y[y3a];
                if (b3e) b3d <= dram_b[b3a];
                if (z3e) begin
                    if (z3m[0]) dram_z[z3a][ 7: 0] <= z3d[ 7: 0];
                    if (z3m[1]) dram_z[z3a][15: 8] <= z3d[15: 8];
                    if (z3m[2]) dram_z[z3a][23:16] <= z3d[23:16];
                    if (z3m[3]) dram_z[z3a][31:24] <= z3d[31:24];
                end
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // DUT instantiations
    // -------------------------------------------------------------------------
    // Test A ? Layer 0: X->SRAM A, Z1->SRAM C
    systolic_pipe_relu #(.TILE_MAX(TILE_MAX),.REAL_K(6),.REAL_M(8),.REAL_N(6),.AW(AW)) dut0 (
        .clk(clk),.rst(rst0),.done(done0),
        .is_last_layer(1'b0),.act_sel(1'b0),.out_sel(1'b0),.skip_x(1'b0),
        .M_int(16'd73),.shift(5'd15),
        .ext_x_addr_o(x0a),.ext_x_rd_en_o(x0e),.ext_x_data_i(x0d),
        .ext_y_addr_o(y0a),.ext_y_rd_en_o(y0e),.ext_y_data_i(y0d),
        .ext_z_addr_o(z0a),.ext_z_wr_en_o(z0e),.ext_z_data_o(z0d),.ext_z_wmask_o(z0m),
        .ext_b_addr_o(b0a),.ext_b_rd_en_o(b0e),.ext_b_data_i(b0d),
        .sram_a_csb0(d0_sa_csb0),.sram_a_web0(d0_sa_web0),.sram_a_wmask0(d0_sa_wm),
        .sram_a_addr0(d0_sa_a0),.sram_a_din0(d0_sa_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(d0_sa_csb1),.sram_a_addr1(d0_sa_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(d0_sb_csb0),.sram_b_web0(d0_sb_web0),.sram_b_wmask0(d0_sb_wm),
        .sram_b_addr0(d0_sb_a0),.sram_b_din0(d0_sb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(d0_sb_csb1),.sram_b_addr1(d0_sb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(d0_sc_csb0),.sram_c_web0(d0_sc_web0),.sram_c_wmask0(d0_sc_wm),
        .sram_c_addr0(d0_sc_a0),.sram_c_din0(d0_sc_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(d0_sc_csb1),.sram_c_addr1(d0_sc_a1),.sram_c_dout1(sc_do1));

    // Test A ? Layer 1: SRAM C->acts, Z2->SRAM A
    systolic_pipe_relu #(.TILE_MAX(TILE_MAX),.REAL_K(6),.REAL_M(6),.REAL_N(4),.AW(AW)) dut1 (
        .clk(clk),.rst(rst1),.done(done1),
        .is_last_layer(1'b0),.act_sel(1'b1),.out_sel(1'b1),.skip_x(1'b1),
        .M_int(16'd55),.shift(5'd15),
        .ext_x_addr_o(x1a),.ext_x_rd_en_o(x1e),.ext_x_data_i(x1d),
        .ext_y_addr_o(y1a),.ext_y_rd_en_o(y1e),.ext_y_data_i(y1d),
        .ext_z_addr_o(z1a),.ext_z_wr_en_o(z1e),.ext_z_data_o(z1d),.ext_z_wmask_o(z1m),
        .ext_b_addr_o(b1a),.ext_b_rd_en_o(b1e),.ext_b_data_i(b1d),
        .sram_a_csb0(d1_sa_csb0),.sram_a_web0(d1_sa_web0),.sram_a_wmask0(d1_sa_wm),
        .sram_a_addr0(d1_sa_a0),.sram_a_din0(d1_sa_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(d1_sa_csb1),.sram_a_addr1(d1_sa_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(d1_sb_csb0),.sram_b_web0(d1_sb_web0),.sram_b_wmask0(d1_sb_wm),
        .sram_b_addr0(d1_sb_a0),.sram_b_din0(d1_sb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(d1_sb_csb1),.sram_b_addr1(d1_sb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(d1_sc_csb0),.sram_c_web0(d1_sc_web0),.sram_c_wmask0(d1_sc_wm),
        .sram_c_addr0(d1_sc_a0),.sram_c_din0(d1_sc_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(d1_sc_csb1),.sram_c_addr1(d1_sc_a1),.sram_c_dout1(sc_do1));

    // Test A ? Layer 2: SRAM A->acts, Z3->DRAM Z (last layer)
    systolic_pipe_relu #(.TILE_MAX(TILE_MAX),.REAL_K(6),.REAL_M(4),.REAL_N(5),.AW(AW)) dut2 (
        .clk(clk),.rst(rst2),.done(done2),
        .is_last_layer(1'b1),.act_sel(1'b0),.out_sel(1'b0),.skip_x(1'b1),
        .M_int(16'd45),.shift(5'd15),
        .ext_x_addr_o(x2a),.ext_x_rd_en_o(x2e),.ext_x_data_i(x2d),
        .ext_y_addr_o(y2a),.ext_y_rd_en_o(y2e),.ext_y_data_i(y2d),
        .ext_z_addr_o(z2a),.ext_z_wr_en_o(z2e),.ext_z_data_o(z2d),.ext_z_wmask_o(z2m),
        .ext_b_addr_o(b2a),.ext_b_rd_en_o(b2e),.ext_b_data_i(b2d),
        .sram_a_csb0(d2_sa_csb0),.sram_a_web0(d2_sa_web0),.sram_a_wmask0(d2_sa_wm),
        .sram_a_addr0(d2_sa_a0),.sram_a_din0(d2_sa_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(d2_sa_csb1),.sram_a_addr1(d2_sa_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(d2_sb_csb0),.sram_b_web0(d2_sb_web0),.sram_b_wmask0(d2_sb_wm),
        .sram_b_addr0(d2_sb_a0),.sram_b_din0(d2_sb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(d2_sb_csb1),.sram_b_addr1(d2_sb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(d2_sc_csb0),.sram_c_web0(d2_sc_web0),.sram_c_wmask0(d2_sc_wm),
        .sram_c_addr0(d2_sc_a0),.sram_c_din0(d2_sc_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(d2_sc_csb1),.sram_c_addr1(d2_sc_a1),.sram_c_dout1(sc_do1));

    // Test B ? single layer, square tile, last layer -> DRAM Z
    systolic_pipe_relu #(.TILE_MAX(TILE_MAX),.REAL_K(4),.REAL_M(6),.REAL_N(4),.AW(AW)) dut3 (
        .clk(clk),.rst(rst3),.done(done3),
        .is_last_layer(1'b1),.act_sel(1'b0),.out_sel(1'b0),.skip_x(1'b0),
        .M_int(16'd60),.shift(5'd15),
        .ext_x_addr_o(x3a),.ext_x_rd_en_o(x3e),.ext_x_data_i(x3d),
        .ext_y_addr_o(y3a),.ext_y_rd_en_o(y3e),.ext_y_data_i(y3d),
        .ext_z_addr_o(z3a),.ext_z_wr_en_o(z3e),.ext_z_data_o(z3d),.ext_z_wmask_o(z3m),
        .ext_b_addr_o(b3a),.ext_b_rd_en_o(b3e),.ext_b_data_i(b3d),
        .sram_a_csb0(d3_sa_csb0),.sram_a_web0(d3_sa_web0),.sram_a_wmask0(d3_sa_wm),
        .sram_a_addr0(d3_sa_a0),.sram_a_din0(d3_sa_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(d3_sa_csb1),.sram_a_addr1(d3_sa_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(d3_sb_csb0),.sram_b_web0(d3_sb_web0),.sram_b_wmask0(d3_sb_wm),
        .sram_b_addr0(d3_sb_a0),.sram_b_din0(d3_sb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(d3_sb_csb1),.sram_b_addr1(d3_sb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(d3_sc_csb0),.sram_c_web0(d3_sc_web0),.sram_c_wmask0(d3_sc_wm),
        .sram_c_addr0(d3_sc_a0),.sram_c_din0(d3_sc_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(d3_sc_csb1),.sram_c_addr1(d3_sc_a1),.sram_c_dout1(sc_do1));

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    function automatic logic signed [7:0] z_byte(input int flat);
        return $signed(dram_z[flat/4][(flat%4)*8 +: 8]);
    endfunction

    task automatic set_x_byte(input int idx, input logic signed [7:0] val);
        dram_x[idx/4][(idx%4)*8 +: 8] = val;
    endtask

    task automatic set_y_byte(input int idx, input logic signed [7:0] val);
        dram_y[idx/4][(idx%4)*8 +: 8] = val;
    endtask

    task automatic check(
        input string name,
        input int    flat_idx,
        input logic signed [7:0] gold,
        inout int pass_c, fail_c
    );
        logic signed [7:0] sv;
        sv = z_byte(flat_idx);
        if (sv !== gold) begin
            $display("  FAIL  %s  flat=%0d  got=%0d  exp=%0d", name, flat_idx, sv, gold);
            fail_c++;
        end else begin
            pass_c++;
        end
    endtask

    int pass_c, fail_c, total_pass, total_fail;

    // =========================================================================
    // Stimulus
    // =========================================================================
    initial begin
        rst0=1; rst1=1; rst2=1; rst3=1; layer_sel=2'd0;
        total_pass=0; total_fail=0;

        for (int a=0;a<1024;a++) begin
            dram_x[a]=32'd0; dram_y[a]=32'd0;
            dram_b[a]=32'd0; dram_z[a]=32'd0;
        end

        // =====================================================================
        // TEST A ? 3-layer chain
        // =====================================================================
        $display("========================================");
        $display("TEST A: 3-layer chain (6x8@8x6, 6x6@6x4, 6x4@4x5)");
        $display("========================================");

        // ----- Layer 0 inputs ------------------------------------------------
        // X(6x8): 6 rows, 8 cols, stored row-major as 32-bit packed words
        set_x_byte( 0,119); set_x_byte( 1, 25); set_x_byte( 2,102); set_x_byte( 3, 59);
        set_x_byte( 4, 34); set_x_byte( 5, 76); set_x_byte( 6, 25); set_x_byte( 7, 42);
        set_x_byte( 8, 25); set_x_byte( 9, 59); set_x_byte(10, 42); set_x_byte(11, 76);
        set_x_byte(12, 59); set_x_byte(13, 17); set_x_byte(14, 34); set_x_byte(15, 76);
        set_x_byte(16,102); set_x_byte(17,119); set_x_byte(18, 17); set_x_byte(19, 85);
        set_x_byte(20, 76); set_x_byte(21, 85); set_x_byte(22, 42); set_x_byte(23, 17);
        set_x_byte(24, 34); set_x_byte(25,102); set_x_byte(26,127); set_x_byte(27,102);
        set_x_byte(28, 59); set_x_byte(29,102); set_x_byte(30,110); set_x_byte(31, 68);
        set_x_byte(32,127); set_x_byte(33, 25); set_x_byte(34,119); set_x_byte(35,  8);
        set_x_byte(36, 34); set_x_byte(37, 17); set_x_byte(38, 68); set_x_byte(39, 34);
        set_x_byte(40, 17); set_x_byte(41,119); set_x_byte(42, 51); set_x_byte(43, 51);
        set_x_byte(44, 85); set_x_byte(45, 34); set_x_byte(46, 51); set_x_byte(47,110);

        // W(8x6): tile-packed, col-tile 0 (cols 0-3): words 0-7, col-tile 1 (cols 4-5): words 8-11
        set_y_byte( 0, 59); set_y_byte( 1, 34); set_y_byte( 2,110); set_y_byte( 3,127);
        set_y_byte( 4,110); set_y_byte( 5, 42); set_y_byte( 6, 59); set_y_byte( 7, 85);
        set_y_byte( 8, 93); set_y_byte( 9, 93); set_y_byte(10, 68); set_y_byte(11, 42);
        set_y_byte(12, 68); set_y_byte(13, 25); set_y_byte(14, 51); set_y_byte(15, 42);
        set_y_byte(16,102); set_y_byte(17,119); set_y_byte(18, 51); set_y_byte(19, 17);
        set_y_byte(20,  8); set_y_byte(21,102); set_y_byte(22, 85); set_y_byte(23, 51);
        set_y_byte(24, 76); set_y_byte(25,  8); set_y_byte(26, 93); set_y_byte(27, 93);
        set_y_byte(28,102); set_y_byte(29,102); set_y_byte(30,127); set_y_byte(31,119);
        set_y_byte(32, 93); set_y_byte(33, 68); set_y_byte(34, 25); set_y_byte(35, 59);
        set_y_byte(36, 34); set_y_byte(37, 68); set_y_byte(38, 17); set_y_byte(39, 68);
        set_y_byte(40,102); set_y_byte(41, 42); set_y_byte(42,110); set_y_byte(43,102);
        set_y_byte(44,127); set_y_byte(45, 85); set_y_byte(46,119); set_y_byte(47,127);

        dram_b[0]=32'(-287); dram_b[1]=32'(-1147);
        dram_b[2]=32'(1935); dram_b[3]=32'(2151);
        dram_b[4]=32'(358);  dram_b[5]=32'(-860);

        // Run layer 0
        #50; rst0=0;
        wait (done0===1'b1); #20;
        // Verify DRAM Z untouched (layer 0 is not last layer)
        pass_c=0; fail_c=0;
        for (int a=0;a<9;a++)
            if (dram_z[a]!==32'd0) fail_c++; else pass_c++;
        $display("  L0 DRAM Z unchanged: %0d ok / %0d wrong", pass_c, fail_c);
        total_pass+=pass_c; total_fail+=fail_c;

        // ----- Layer 1 inputs ------------------------------------------------
        for (int a=0;a<1024;a++) dram_y[a]=32'd0;
        // W(6x4): one col-tile, words 0-5 (perfectly aligned rows)
        set_y_byte( 0,127); set_y_byte( 1, 17); set_y_byte( 2, 85); set_y_byte( 3,102);
        set_y_byte( 4,119); set_y_byte( 5, 34); set_y_byte( 6,119); set_y_byte( 7,127);
        set_y_byte( 8,119); set_y_byte( 9, 59); set_y_byte(10,102); set_y_byte(11, 76);
        set_y_byte(12, 42); set_y_byte(13,110); set_y_byte(14, 17); set_y_byte(15,127);
        set_y_byte(16, 85); set_y_byte(17, 76); set_y_byte(18,102); set_y_byte(19,102);
        set_y_byte(20, 76); set_y_byte(21,110); set_y_byte(22,127); set_y_byte(23,110);

        dram_b[0]=32'(14);  dram_b[1]=32'(-11);
        dram_b[2]=32'(20);  dram_b[3]=32'(-7);
        dram_b[4]=32'(4);   dram_b[5]=32'(9);

        // Run layer 1
        rst0=1; layer_sel=2'd1; rst1=1; #30; rst1=0;
        wait (done1===1'b1); #20;

        // ----- Layer 2 inputs ------------------------------------------------
        for (int a=0;a<1024;a++) dram_y[a]=32'd0;
        for (int a=0;a<8;a++) dram_z[a]=32'd0;
        // W(4x5): col-tile 0 (cols 0-3) words 0-3, col-tile 1 (col 4) word 4
        set_y_byte( 0, 17); set_y_byte( 1, 34); set_y_byte( 2, 51); set_y_byte( 3, 68);
        set_y_byte( 4, 25); set_y_byte( 5, 42); set_y_byte( 6, 59); set_y_byte( 7, 76);
        set_y_byte( 8,  8); set_y_byte( 9, 25); set_y_byte(10, 42); set_y_byte(11, 59);
        set_y_byte(12, 51); set_y_byte(13, 34); set_y_byte(14, 17); set_y_byte(15, 85);
        set_y_byte(16, 85); set_y_byte(17, 93); set_y_byte(18, 76); set_y_byte(19,102);

        dram_b[0]=32'(100);  dram_b[1]=32'(-50);
        dram_b[2]=32'(200);  dram_b[3]=32'(-100);
        dram_b[4]=32'(150);  dram_b[5]=32'(0);

        // Run layer 2
        rst1=1; layer_sel=2'd2; rst2=1; #30; rst2=0;
        wait (done2===1'b1); #20;

        // Check Z3(6x5): gold from Python requant model
        pass_c=0; fail_c=0;
        begin
            logic signed [7:0] gold[6][5];
            gold[0]='{8'sd11,8'sd13,8'sd16,8'sd29,8'sd35};
            gold[1]='{8'sd8, 8'sd10,8'sd12,8'sd22,8'sd28};
            gold[2]='{8'sd12,8'sd15,8'sd18,8'sd32,8'sd40};
            gold[3]='{8'sd15,8'sd20,8'sd24,8'sd43,8'sd53};
            gold[4]='{8'sd10,8'sd13,8'sd15,8'sd27,8'sd34};
            gold[5]='{8'sd11,8'sd14,8'sd18,8'sd32,8'sd39};
            for (int i=0;i<6;i++)
                for (int j=0;j<5;j++)
                    check("Z3", i*5+j, gold[i][j], pass_c, fail_c);
        end
        $display("  Test A (Z3 6x5): %0d PASS / %0d FAIL", pass_c, fail_c);
        total_pass+=pass_c; total_fail+=fail_c;

        // =====================================================================
        // TEST B ? single layer, square tile  X(4x6)@W(6x4)->Z(4x4)
        // =====================================================================
        $display("\n========================================");
        $display("TEST B: single layer (4x6 @ 6x4, one full tile)");
        $display("========================================");

        for (int a=0;a<1024;a++) begin
            dram_x[a]=32'd0; dram_y[a]=32'd0;
            dram_b[a]=32'd0; dram_z[a]=32'd0;
        end

        // X(4x6): row-major, 24 bytes
        set_x_byte( 0, 61); set_x_byte( 1, 24); set_x_byte( 2, 81); set_x_byte( 3, 70);
        set_x_byte( 4, 30); set_x_byte( 5, 92); set_x_byte( 6, 96); set_x_byte( 7, 84);
        set_x_byte( 8, 84); set_x_byte( 9, 97); set_x_byte(10, 33); set_x_byte(11, 12);
        set_x_byte(12, 31); set_x_byte(13, 62); set_x_byte(14, 11); set_x_byte(15, 97);
        set_x_byte(16, 39); set_x_byte(17, 47); set_x_byte(18, 11); set_x_byte(19, 73);
        set_x_byte(20, 69); set_x_byte(21, 30); set_x_byte(22, 42); set_x_byte(23, 85);

        // W(6x4): one col-tile, 6 words (one row per word, 4 bytes each)
        set_y_byte( 0, 67); set_y_byte( 1, 31); set_y_byte( 2, 98); set_y_byte( 3, 58);
        set_y_byte( 4,100); set_y_byte( 5, 68); set_y_byte( 6, 51); set_y_byte( 7, 69);
        set_y_byte( 8, 89); set_y_byte( 9, 24); set_y_byte(10, 71); set_y_byte(11, 71);
        set_y_byte(12, 56); set_y_byte(13, 71); set_y_byte(14, 60); set_y_byte(15, 64);
        set_y_byte(16, 73); set_y_byte(17, 12); set_y_byte(18, 60); set_y_byte(19, 16);
        set_y_byte(20, 30); set_y_byte(21, 82); set_y_byte(22, 48); set_y_byte(23, 27);

        dram_b[0]=32'(187); dram_b[1]=32'(-112);
        dram_b[2]=32'(115); dram_b[3]=32'(-187);

        rst2=1; layer_sel=2'd3; rst3=1; #30; rst3=0;
        wait (done3===1'b1); #20;

        // Check Z(4x4): gold from Python requant model
        pass_c=0; fail_c=0;
        begin
            logic signed [7:0] gold[4][4];
            gold[0]='{8'sd42,8'sd34,8'sd43,8'sd34};
            gold[1]='{8'sd56,8'sd35,8'sd51,8'sd44};
            gold[2]='{8'sd35,8'sd31,8'sd32,8'sd28};
            gold[3]='{8'sd39,8'sd30,8'sd33,8'sd28};
            for (int i=0;i<4;i++)
                for (int j=0;j<4;j++)
                    check("ZB", i*4+j, gold[i][j], pass_c, fail_c);
        end
        $display("  Test B (Z 4x4):  %0d PASS / %0d FAIL", pass_c, fail_c);
        total_pass+=pass_c; total_fail+=fail_c;

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n========================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", total_pass, total_fail);
        if (total_fail==0) $display("  ALL CORRECT.");
        else               $display("  FAILURES ? see above.");
        $display("========================================");
        $finish;
    end

    // Safety watchdog
    initial begin #2000000; $display("TIMEOUT: simulation exceeded 2ms"); $finish; end

endmodule
