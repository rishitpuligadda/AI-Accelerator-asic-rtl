// =============================================================================
// tbStress.sv  -  Comprehensive tiling stress test
//
// Three DUT instances (E, F, G), each with their own signal buses.
// A test_sel register muxes the active DUT's signals onto the shared
// DRAM/SRAM.  Only the active DUT has rst=0; the others are held in rst=1.
//
// Test E  -  8x8x2 | 3x3 | stride=1 pad=0 | 5 filters
//   REAL_K=36  REAL_M=18  REAL_N=5    18 tile passes
//   M%4=2 -> pack-buf partial flush.  Multi-channel im2col.
//   K tiles: 9x4 (exact).  N tiles: 4+1 (partial last).
//
// Test F  -  7x7x1 | 3x3 | stride=2 pad=1 | 5 filters
//   REAL_K=16  REAL_M=9   REAL_N=5     8 tile passes
//   Non-unit stride + padding: non-uniform output grid.
//   N tiles: 4+1 (partial last).
//
// Test G  -  5x5x1 | 3x3 | stride=1 pad=0 | 6 filters
//   REAL_K=9   REAL_M=9   REAL_N=6     6 tile passes
//   K tiles: 4+4+1 (last K-tile = 1 row only).
//   N tiles: 4+2.  M%4=1.  Triple non-alignment.
// =============================================================================

`timescale 1ns/1ps

module tb_stress;

    localparam int TILE_MAX = 4;
    localparam int AW       = 8;

    // =========================================================================
    // Test parameters
    // =========================================================================

    // --- E ---
    localparam int E_IN_H=8, E_IN_W=8, E_IN_C=2, E_K_H=3, E_K_W=3, E_ST=1, E_PD=0, E_OC=5;
    localparam int E_OH=(E_IN_H+2*E_PD-E_K_H)/E_ST+1;  // 6
    localparam int E_OW=(E_IN_W+2*E_PD-E_K_W)/E_ST+1;  // 6
    localparam int E_RK=E_OH*E_OW;   // 36
    localparam int E_RM=E_K_H*E_K_W*E_IN_C;  // 18
    localparam int E_RN=E_OC;        // 5

    // --- F ---
    localparam int F_IN_H=7, F_IN_W=7, F_IN_C=1, F_K_H=3, F_K_W=3, F_ST=2, F_PD=1, F_OC=5;
    localparam int F_OH=(F_IN_H+2*F_PD-F_K_H)/F_ST+1;  // 4
    localparam int F_OW=(F_IN_W+2*F_PD-F_K_W)/F_ST+1;  // 4
    localparam int F_RK=F_OH*F_OW;   // 16
    localparam int F_RM=F_K_H*F_K_W*F_IN_C;  // 9
    localparam int F_RN=F_OC;        // 5

    // --- G ---
    localparam int G_IN_H=5, G_IN_W=5, G_IN_C=1, G_K_H=3, G_K_W=3, G_ST=1, G_PD=0, G_OC=6;
    localparam int G_OH=(G_IN_H+2*G_PD-G_K_H)/G_ST+1;  // 3
    localparam int G_OW=(G_IN_W+2*G_PD-G_K_W)/G_ST+1;  // 3
    localparam int G_RK=G_OH*G_OW;   // 9
    localparam int G_RM=G_K_H*G_K_W*G_IN_C;  // 9
    localparam int G_RN=G_OC;        // 6

    // =========================================================================
    // Clock
    // =========================================================================
    logic clk = 0;
    always #5 clk = ~clk;

    // =========================================================================
    // Shared DRAM
    // =========================================================================
    reg [31:0] dram_x[0:1023], dram_y[0:1023], dram_b[0:1023], dram_z[0:1023];
    reg clear_z;
    always @(posedge clk)
        if (clear_z) for (int i=0; i<1024; i++) dram_z[i] <= 32'd0;

    // =========================================================================
    // Per-DUT signal buses (E, F, G)
    // =========================================================================

    // rst - one per DUT so only the active one is deasserted
    logic rst_e, rst_f, rst_g;
    logic done_e, done_f, done_g;

    // DRAM-side signals per DUT
    logic [15:0] xe_addr, ye_addr, ze_addr, be_addr;
    logic        xe_en, ye_en, ze_en, be_en;
    logic [3:0]  ze_mask; logic [31:0] ze_data;

    logic [15:0] xf_addr, yf_addr, zf_addr, bf_addr;
    logic        xf_en, yf_en, zf_en, bf_en;
    logic [3:0]  zf_mask; logic [31:0] zf_data;

    logic [15:0] xg_addr, yg_addr, zg_addr, bg_addr;
    logic        xg_en, yg_en, zg_en, bg_en;
    logic [3:0]  zg_mask; logic [31:0] zg_data;

    // SRAM signals per DUT
    logic        sea_csb0,sea_web0; logic[3:0] sea_wm; logic[AW-1:0] sea_a0; logic[31:0] sea_di;
    logic        sea_csb1;          logic[AW-1:0] sea_a1;
    logic        seb_csb0,seb_web0; logic[3:0] seb_wm; logic[AW-1:0] seb_a0; logic[31:0] seb_di;
    logic        seb_csb1;          logic[AW-1:0] seb_a1;
    logic        sec_csb0,sec_web0; logic[3:0] sec_wm; logic[AW-1:0] sec_a0; logic[31:0] sec_di;
    logic        sec_csb1;          logic[AW-1:0] sec_a1;

    logic        sfa_csb0,sfa_web0; logic[3:0] sfa_wm; logic[AW-1:0] sfa_a0; logic[31:0] sfa_di;
    logic        sfa_csb1;          logic[AW-1:0] sfa_a1;
    logic        sfb_csb0,sfb_web0; logic[3:0] sfb_wm; logic[AW-1:0] sfb_a0; logic[31:0] sfb_di;
    logic        sfb_csb1;          logic[AW-1:0] sfb_a1;
    logic        sfc_csb0,sfc_web0; logic[3:0] sfc_wm; logic[AW-1:0] sfc_a0; logic[31:0] sfc_di;
    logic        sfc_csb1;          logic[AW-1:0] sfc_a1;

    logic        sga_csb0,sga_web0; logic[3:0] sga_wm; logic[AW-1:0] sga_a0; logic[31:0] sga_di;
    logic        sga_csb1;          logic[AW-1:0] sga_a1;
    logic        sgb_csb0,sgb_web0; logic[3:0] sgb_wm; logic[AW-1:0] sgb_a0; logic[31:0] sgb_di;
    logic        sgb_csb1;          logic[AW-1:0] sgb_a1;
    logic        sgc_csb0,sgc_web0; logic[3:0] sgc_wm; logic[AW-1:0] sgc_a0; logic[31:0] sgc_di;
    logic        sgc_csb1;          logic[AW-1:0] sgc_a1;

    // =========================================================================
    // Mux selector: 0=E, 1=F, 2=G
    // =========================================================================
    logic [1:0] test_sel;

    // Muxed DRAM/SRAM signals
    logic [15:0] x_addr, y_addr, z_addr, b_addr;
    logic        x_en, y_en, z_en, b_en;
    logic [3:0]  z_mask; logic [31:0] z_data;
    logic        sa_csb0,sa_web0; logic[3:0] sa_wm; logic[AW-1:0] sa_a0; logic[31:0] sa_di;
    logic        sa_csb1;         logic[AW-1:0] sa_a1; logic[31:0] sa_do0, sa_do1;
    logic        sb_csb0,sb_web0; logic[3:0] sb_wm; logic[AW-1:0] sb_a0; logic[31:0] sb_di;
    logic        sb_csb1;         logic[AW-1:0] sb_a1; logic[31:0] sb_do0, sb_do1;
    logic        sc_csb0,sc_web0; logic[3:0] sc_wm; logic[AW-1:0] sc_a0; logic[31:0] sc_di;
    logic        sc_csb1;         logic[AW-1:0] sc_a1; logic[31:0] sc_do0, sc_do1;

    // Mux combinational
    always_comb begin
        case (test_sel)
            2'd0: begin
                x_addr=xe_addr; y_addr=ye_addr; b_addr=be_addr;
                x_en=xe_en;     y_en=ye_en;     b_en=be_en;
                z_addr=ze_addr; z_en=ze_en; z_mask=ze_mask; z_data=ze_data;
                sa_csb0=sea_csb0; sa_web0=sea_web0; sa_wm=sea_wm; sa_a0=sea_a0; sa_di=sea_di;
                sa_csb1=sea_csb1; sa_a1=sea_a1;
                sb_csb0=seb_csb0; sb_web0=seb_web0; sb_wm=seb_wm; sb_a0=seb_a0; sb_di=seb_di;
                sb_csb1=seb_csb1; sb_a1=seb_a1;
                sc_csb0=sec_csb0; sc_web0=sec_web0; sc_wm=sec_wm; sc_a0=sec_a0; sc_di=sec_di;
                sc_csb1=sec_csb1; sc_a1=sec_a1;
            end
            2'd1: begin
                x_addr=xf_addr; y_addr=yf_addr; b_addr=bf_addr;
                x_en=xf_en;     y_en=yf_en;     b_en=bf_en;
                z_addr=zf_addr; z_en=zf_en; z_mask=zf_mask; z_data=zf_data;
                sa_csb0=sfa_csb0; sa_web0=sfa_web0; sa_wm=sfa_wm; sa_a0=sfa_a0; sa_di=sfa_di;
                sa_csb1=sfa_csb1; sa_a1=sfa_a1;
                sb_csb0=sfb_csb0; sb_web0=sfb_web0; sb_wm=sfb_wm; sb_a0=sfb_a0; sb_di=sfb_di;
                sb_csb1=sfb_csb1; sb_a1=sfb_a1;
                sc_csb0=sfc_csb0; sc_web0=sfc_web0; sc_wm=sfc_wm; sc_a0=sfc_a0; sc_di=sfc_di;
                sc_csb1=sfc_csb1; sc_a1=sfc_a1;
            end
            default: begin
                x_addr=xg_addr; y_addr=yg_addr; b_addr=bg_addr;
                x_en=xg_en;     y_en=yg_en;     b_en=bg_en;
                z_addr=zg_addr; z_en=zg_en; z_mask=zg_mask; z_data=zg_data;
                sa_csb0=sga_csb0; sa_web0=sga_web0; sa_wm=sga_wm; sa_a0=sga_a0; sa_di=sga_di;
                sa_csb1=sga_csb1; sa_a1=sga_a1;
                sb_csb0=sgb_csb0; sb_web0=sgb_web0; sb_wm=sgb_wm; sb_a0=sgb_a0; sb_di=sgb_di;
                sb_csb1=sgb_csb1; sb_a1=sgb_a1;
                sc_csb0=sgc_csb0; sc_web0=sgc_web0; sc_wm=sgc_wm; sc_a0=sgc_a0; sc_di=sgc_di;
                sc_csb1=sgc_csb1; sc_a1=sgc_a1;
            end
        endcase
    end

    // =========================================================================
    // DRAM read/write models
    // =========================================================================
    logic [15:0] x_addr_r, y_addr_r, b_addr_r;
    logic        x_en_r,   y_en_r,   b_en_r;
    always_ff @(posedge clk) begin
        x_addr_r<=x_addr; x_en_r<=x_en;
        y_addr_r<=y_addr; y_en_r<=y_en;
        b_addr_r<=b_addr; b_en_r<=b_en;
    end
    logic [31:0] x_data, y_data, b_data;
    assign x_data = x_en_r ? dram_x[x_addr_r] : 32'hDEADBEEF;
    assign y_data = y_en_r ? dram_y[y_addr_r] : 32'hDEADBEEF;
    assign b_data = b_en_r ? dram_b[b_addr_r] : 32'hDEADBEEF;
    always @(posedge clk) if (z_en) begin
        if (z_mask[0]) dram_z[z_addr][ 7: 0] <= z_data[ 7: 0];
        if (z_mask[1]) dram_z[z_addr][15: 8] <= z_data[15: 8];
        if (z_mask[2]) dram_z[z_addr][23:16] <= z_data[23:16];
        if (z_mask[3]) dram_z[z_addr][31:24] <= z_data[31:24];
    end

    // =========================================================================
    // SRAMs (shared physical SRAMs, each test gets full exclusive access)
    // =========================================================================
    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) sram_a (
        .clk0(clk),.csb0(sa_csb0),.web0(sa_web0),.wmask0(sa_wm),
        .addr0(sa_a0),.din0(sa_di),.dout0(sa_do0),
        .clk1(clk),.csb1(sa_csb1),.addr1(sa_a1),.dout1(sa_do1));
    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) sram_b (
        .clk0(clk),.csb0(sb_csb0),.web0(sb_web0),.wmask0(sb_wm),
        .addr0(sb_a0),.din0(sb_di),.dout0(sb_do0),
        .clk1(clk),.csb1(sb_csb1),.addr1(sb_a1),.dout1(sb_do1));
    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) sram_c (
        .clk0(clk),.csb0(sc_csb0),.web0(sc_web0),.wmask0(sc_wm),
        .addr0(sc_a0),.din0(sc_di),.dout0(sc_do0),
        .clk1(clk),.csb1(sc_csb1),.addr1(sc_a1),.dout1(sc_do1));

    // =========================================================================
    // DUT E
    // =========================================================================
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX),.IN_H(E_IN_H),.IN_W(E_IN_W),.IN_C(E_IN_C),
        .OUT_C(E_OC),.K_H(E_K_H),.K_W(E_K_W),.STRIDE(E_ST),.PAD(E_PD),.AW(AW)
    ) dut_e (
        .clk(clk),.rst(rst_e),.done(done_e),
        .is_last_layer(1'b1),.act_sel(1'b0),.out_sel(1'b0),.skip_x(1'b0),
        .ext_x_addr_o(xe_addr),.ext_x_rd_en_o(xe_en),.ext_x_data_i(x_data),
        .ext_y_addr_o(ye_addr),.ext_y_rd_en_o(ye_en),.ext_y_data_i(y_data),
        .ext_z_addr_o(ze_addr),.ext_z_wr_en_o(ze_en),.ext_z_data_o(ze_data),.ext_z_wmask_o(ze_mask),
        .ext_b_addr_o(be_addr),.ext_b_rd_en_o(be_en),.ext_b_data_i(b_data),
        .sram_a_csb0(sea_csb0),.sram_a_web0(sea_web0),.sram_a_wmask0(sea_wm),
        .sram_a_addr0(sea_a0),.sram_a_din0(sea_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(sea_csb1),.sram_a_addr1(sea_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(seb_csb0),.sram_b_web0(seb_web0),.sram_b_wmask0(seb_wm),
        .sram_b_addr0(seb_a0),.sram_b_din0(seb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(seb_csb1),.sram_b_addr1(seb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(sec_csb0),.sram_c_web0(sec_web0),.sram_c_wmask0(sec_wm),
        .sram_c_addr0(sec_a0),.sram_c_din0(sec_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(sec_csb1),.sram_c_addr1(sec_a1),.sram_c_dout1(sc_do1));

    // =========================================================================
    // DUT F
    // =========================================================================
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX),.IN_H(F_IN_H),.IN_W(F_IN_W),.IN_C(F_IN_C),
        .OUT_C(F_OC),.K_H(F_K_H),.K_W(F_K_W),.STRIDE(F_ST),.PAD(F_PD),.AW(AW)
    ) dut_f (
        .clk(clk),.rst(rst_f),.done(done_f),
        .is_last_layer(1'b1),.act_sel(1'b0),.out_sel(1'b0),.skip_x(1'b0),
        .ext_x_addr_o(xf_addr),.ext_x_rd_en_o(xf_en),.ext_x_data_i(x_data),
        .ext_y_addr_o(yf_addr),.ext_y_rd_en_o(yf_en),.ext_y_data_i(y_data),
        .ext_z_addr_o(zf_addr),.ext_z_wr_en_o(zf_en),.ext_z_data_o(zf_data),.ext_z_wmask_o(zf_mask),
        .ext_b_addr_o(bf_addr),.ext_b_rd_en_o(bf_en),.ext_b_data_i(b_data),
        .sram_a_csb0(sfa_csb0),.sram_a_web0(sfa_web0),.sram_a_wmask0(sfa_wm),
        .sram_a_addr0(sfa_a0),.sram_a_din0(sfa_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(sfa_csb1),.sram_a_addr1(sfa_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(sfb_csb0),.sram_b_web0(sfb_web0),.sram_b_wmask0(sfb_wm),
        .sram_b_addr0(sfb_a0),.sram_b_din0(sfb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(sfb_csb1),.sram_b_addr1(sfb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(sfc_csb0),.sram_c_web0(sfc_web0),.sram_c_wmask0(sfc_wm),
        .sram_c_addr0(sfc_a0),.sram_c_din0(sfc_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(sfc_csb1),.sram_c_addr1(sfc_a1),.sram_c_dout1(sc_do1));

    // =========================================================================
    // DUT G
    // =========================================================================
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX),.IN_H(G_IN_H),.IN_W(G_IN_W),.IN_C(G_IN_C),
        .OUT_C(G_OC),.K_H(G_K_H),.K_W(G_K_W),.STRIDE(G_ST),.PAD(G_PD),.AW(AW)
    ) dut_g (
        .clk(clk),.rst(rst_g),.done(done_g),
        .is_last_layer(1'b1),.act_sel(1'b0),.out_sel(1'b0),.skip_x(1'b0),
        .ext_x_addr_o(xg_addr),.ext_x_rd_en_o(xg_en),.ext_x_data_i(x_data),
        .ext_y_addr_o(yg_addr),.ext_y_rd_en_o(yg_en),.ext_y_data_i(y_data),
        .ext_z_addr_o(zg_addr),.ext_z_wr_en_o(zg_en),.ext_z_data_o(zg_data),.ext_z_wmask_o(zg_mask),
        .ext_b_addr_o(bg_addr),.ext_b_rd_en_o(bg_en),.ext_b_data_i(b_data),
        .sram_a_csb0(sga_csb0),.sram_a_web0(sga_web0),.sram_a_wmask0(sga_wm),
        .sram_a_addr0(sga_a0),.sram_a_din0(sga_di),.sram_a_dout0(sa_do0),
        .sram_a_csb1(sga_csb1),.sram_a_addr1(sga_a1),.sram_a_dout1(sa_do1),
        .sram_b_csb0(sgb_csb0),.sram_b_web0(sgb_web0),.sram_b_wmask0(sgb_wm),
        .sram_b_addr0(sgb_a0),.sram_b_din0(sgb_di),.sram_b_dout0(sb_do0),
        .sram_b_csb1(sgb_csb1),.sram_b_addr1(sgb_a1),.sram_b_dout1(sb_do1),
        .sram_c_csb0(sgc_csb0),.sram_c_web0(sgc_web0),.sram_c_wmask0(sgc_wm),
        .sram_c_addr0(sgc_a0),.sram_c_din0(sgc_di),.sram_c_dout0(sc_do0),
        .sram_c_csb1(sgc_csb1),.sram_c_addr1(sgc_a1),.sram_c_dout1(sc_do1));

    // =========================================================================
    // Helpers
    // =========================================================================
    task automatic set_x_byte(input int idx, input logic signed [7:0] val);
        dram_x[idx/4][(idx%4)*8 +: 8] = val;
    endtask
    task automatic set_y_byte(input int idx, input logic signed [7:0] val);
        dram_y[idx/4][(idx%4)*8 +: 8] = val;
    endtask
    function automatic logic signed [7:0] z_byte(input int flat);
        return $signed(dram_z[flat/4][(flat%4)*8 +: 8]);
    endfunction
    function automatic logic signed [7:0] clip8(input logic signed [31:0] v);
        if      (v >  127) return  8'sd127;
        else if (v < -128) return -8'sd128;
        else               return  v[7:0];
    endfunction

    // Gold convolution (handles multi-channel, stride, pad)
    function automatic logic signed [31:0] gold_conv(
        input logic signed [7:0] img  [],
        input logic signed [7:0] kern [],
        input int IH, IW, KH, KW, ST, PD, RM, RN, op, f
    );
        int OW, oh, ow; logic signed [31:0] acc;
        acc = 32'sd0;
        OW  = (IW + 2*PD - KW) / ST + 1;
        oh  = op / OW; ow = op % OW;
        for (int t = 0; t < RM; t++) begin
            int ic, t_sp, kh_i, kw_i, sr, sc;
            ic   = t / (KH * KW);
            t_sp = t % (KH * KW);
            kh_i = t_sp / KW;
            kw_i = t_sp % KW;
            sr   = oh*ST + kh_i - PD;
            sc   = ow*ST + kw_i - PD;
            if (sr>=0 && sr<IH && sc>=0 && sc<IW)
                acc += $signed(32'(img[ic*IH*IW + sr*IW + sc]))
                     * $signed(32'(kern[t*RN + f]));
        end
        return acc;
    endfunction

    task automatic print_grid(
        input string title, input int rows, cols,
        input logic signed [7:0] data []
    );
        $display("  %s", title);
        $write("          ");
        for (int c=0;c<cols;c++) $write(" col%0d  ",c);
        $display("");
        $write("          ");
        for (int c=0;c<cols;c++) $write("+------");
        $display("+");
        for (int r=0;r<rows;r++) begin
            $write("  row%0d   ",r);
            for (int c=0;c<cols;c++) $write("| %4d ",data[r*cols+c]);
            $display("|");
            $write("          ");
            for (int c=0;c<cols;c++) $write("+------");
            $display("+");
        end
    endtask

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin
        #200_000_000;
        $display("WATCHDOG: timeout");
        $finish;
    end

    // =========================================================================
    // Main stimulus
    // =========================================================================
    initial begin
        // All DUTs in reset, mux on E
        rst_e=1; rst_f=1; rst_g=1; test_sel=0; clear_z=0;
        for (int i=0;i<1024;i++) begin
            dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; dram_z[i]='0;
        end
        @(posedge clk); #1;

        $display("");
        $display("================================================================");
        $display("STRESS TEST SUITE  -  Tiling edge case coverage");
        $display("================================================================");

        // ================================================================
        // TEST E  -  8x8x2 | 3x3 | stride=1 pad=0 | 5 filters
        // REAL_K=36 (9 tiles of 4, exact K-alignment)
        // REAL_M=18 (M%4=2, pack-buf partial flush)
        // REAL_N=5  (N tiles: 4+1, partial last N-tile)
        // Multi-channel: IN_C=2 so im2col must cross channel boundary
        // Total tile passes: 18
        // ================================================================
        $display("");
        $display("================================================================");
        $display("TEST E  -  8x8x2 | 3x3 kernel | 5 filters | stride=1 pad=0");
        $display("  REAL_K=36  REAL_M=18  REAL_N=5  |  18 tile passes");
        $display("  K tiles : 9 x 4  (exact, no partial K)");
        $display("  N tiles : 4 + 1  (partial last N-tile)");
        $display("  M%%4    = 2       (pack-buf partial flush every tile)");
        $display("  IN_C=2  : im2col must stride correctly across both channels");
        $display("  Expected (uniform): f0=-16 f1=-32 f2=-48 f3=-64 f4=-80");
        $display("================================================================");
        begin
            logic signed [7:0] img_e  [E_IN_C*E_IN_H*E_IN_W];
            logic signed [7:0] kern_e [E_RM*E_RN];
            logic signed [7:0] sobelX [9];
            sobelX = '{8'sd1,8'sd0,-8'sd1,8'sd2,8'sd0,-8'sd2,8'sd1,8'sd0,-8'sd1};

            // image: ch0 = 1..64, ch1 = 65..128
            for (int ic=0;ic<E_IN_C;ic++)
                for (int r=0;r<E_IN_H;r++)
                    for (int c=0;c<E_IN_W;c++)
                        img_e[ic*E_IN_H*E_IN_W + r*E_IN_W + c] =
                            8'(ic*E_IN_H*E_IN_W + r*E_IN_W + c + 1);

            // weights: same sobelX pattern for each channel, scaled per filter
            for (int t=0;t<E_RM;t++) begin
                int t_sp; t_sp = t%(E_K_H*E_K_W);
                for (int f=0;f<E_RN;f++)
                    kern_e[t*E_RN+f] = 8'($signed(sobelX[t_sp])*(f+1));
            end

            // Load DRAM
            for (int i=0;i<1024;i++) begin dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; end
            @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;
            for (int ic=0;ic<E_IN_C;ic++)
                for (int r=0;r<E_IN_H;r++)
                    for (int c=0;c<E_IN_W;c++)
                        set_x_byte(ic*E_IN_H*E_IN_W+r*E_IN_W+c,
                                   img_e[ic*E_IN_H*E_IN_W+r*E_IN_W+c]);
            for (int t=0;t<E_RM;t++)
                for (int f=0;f<E_RN;f++)
                    set_y_byte(t*E_RN+f, kern_e[t*E_RN+f]);
            for (int i=0;i<E_RK;i++) dram_b[i]=32'd0;

            // Run DUT E
            test_sel=0; rst_e=1; rst_f=1; rst_g=1;
            @(posedge clk); #1; rst_e=0;
            wait(done_e===1'b1);
            @(posedge clk); #1; @(posedge clk); #1;

            // Check
            begin
                int pass_c, fail_c;
                logic signed [7:0] out_grid [E_RK];
                pass_c=0; fail_c=0;
                $display("");
                for (int f=0;f<E_RN;f++) begin
                    int fp, ff; fp=0; ff=0;
                    for (int op=0;op<E_RK;op++) begin
                        logic signed [7:0] gv, hv;
                        logic signed [31:0] raw_acc;
                        raw_acc = gold_conv(img_e,kern_e,E_IN_H,E_IN_W,E_K_H,E_K_W,
                                           E_ST,E_PD,E_RM,E_RN,op,f);
                        gv = 8'(raw_acc);  // raw 8-bit truncation (matches hardware)
                        hv = z_byte(op*E_RN+f);
                        out_grid[op]=hv;
                        if (hv===gv) begin pass_c++; fp++; end
                        else begin
                            fail_c++; ff++;
                            $display("  FAIL: op=%0d f=%0d  got=%0d  gold=%0d  raw_acc=%0d  z_idx=%0d",
                                     op, f, hv, gv, raw_acc, op*E_RN+f);
                        end
                    end
                    begin
                        string t2;
                        $sformat(t2,"Filter %0d  (expected %0d everywhere)  %s",
                                 f,-(f+1)*16,(ff==0)?"PASS":$sformatf("FAIL(%0d)",ff));
                        print_grid(t2,E_OH,E_OW,out_grid);
                        $display("");
                    end
                end
                $display("================================================================");
                if (fail_c==0)
                    $display("TEST E: ALL %0d OUTPUTS CORRECT  (%0d x %0d)",
                             pass_c,E_RK,E_RN);
                else
                    $display("TEST E: %0d PASS  %0d FAIL",pass_c,fail_c);
                $display("================================================================");
            end
        end

        // ================================================================
        // TEST F  -  7x7x1 | 3x3 | stride=2 pad=1 | 5 filters
        // REAL_K=16 (4 tiles of 4, exact K-alignment)
        // REAL_N=5  (N tiles: 4+1, partial last N-tile)
        // stride=2, pad=1: non-uniform spatial output
        //   border positions overlap zero-padded region
        //   interior positions are fully covered by real pixels
        // Total tile passes: 8
        // ================================================================
        $display("");
        $display("================================================================");
        $display("TEST F  -  7x7x1 | 3x3 kernel | 5 filters | stride=2 pad=1");
        $display("  REAL_K=16  REAL_M=9  REAL_N=5  |  8 tile passes");
        $display("  K tiles : 4 x 4  (exact)");
        $display("  N tiles : 4 + 1  (partial last N-tile)");
        $display("  stride=2 + pad=1: non-uniform output grid");
        $display("  Gold f0: row0=[-13,-6,-6,25] row1=[-64,-8,-8,80]");
        $display("           row2=[-120,-8,-8,127] row3=[-125,-6,-6,127]");
        $display("================================================================");
        begin
            logic signed [7:0] img_f  [F_IN_H*F_IN_W];
            logic signed [7:0] kern_f [F_RM*F_RN];
            logic signed [7:0] sobelX [9];
            sobelX = '{8'sd1,8'sd0,-8'sd1,8'sd2,8'sd0,-8'sd2,8'sd1,8'sd0,-8'sd1};

            for (int r=0;r<F_IN_H;r++)
                for (int c=0;c<F_IN_W;c++)
                    img_f[r*F_IN_W+c] = 8'(r*F_IN_W+c+1);

            for (int t=0;t<F_RM;t++)
                for (int f=0;f<F_RN;f++)
                    kern_f[t*F_RN+f] = 8'($signed(sobelX[t])*(f+1));

            for (int i=0;i<1024;i++) begin dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; end
            @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;
            for (int r=0;r<F_IN_H;r++)
                for (int c=0;c<F_IN_W;c++)
                    set_x_byte(r*F_IN_W+c, img_f[r*F_IN_W+c]);
            for (int t=0;t<F_RM;t++)
                for (int f=0;f<F_RN;f++)
                    set_y_byte(t*F_RN+f, kern_f[t*F_RN+f]);
            for (int i=0;i<F_RK;i++) dram_b[i]=32'd0;

            test_sel=1; rst_e=1; rst_f=1; rst_g=1;
            @(posedge clk); #1; rst_f=0;
            wait(done_f===1'b1);
            @(posedge clk); #1; @(posedge clk); #1;

            begin
                int pass_c, fail_c;
                logic signed [7:0] out_grid [F_RK];
                pass_c=0; fail_c=0;
                $display("");
                for (int f=0;f<F_RN;f++) begin
                    int fp,ff; fp=0; ff=0;
                    for (int op=0;op<F_RK;op++) begin
                        logic signed [7:0] gv, hv;
                        logic signed [31:0] raw_acc;
                        raw_acc = gold_conv(img_f,kern_f,F_IN_H,F_IN_W,F_K_H,F_K_W,
                                           F_ST,F_PD,F_RM,F_RN,op,f);
                        gv = 8'(raw_acc);  // raw 8-bit truncation (matches hardware)
                        hv = z_byte(op*F_RN+f);
                        out_grid[op]=hv;
                        if (hv===gv) begin pass_c++; fp++; end
                        else begin
                            fail_c++; ff++;
                            $display("  FAIL: op=%0d f=%0d  got=%0d  gold=%0d  raw_acc=%0d",
                                     op, f, hv, gv, raw_acc);
                        end
                    end
                    begin
                        string t2;
                        $sformat(t2,"Filter %0d  (spatially varying)  %s",
                                 f,(ff==0)?"PASS":$sformatf("FAIL(%0d)",ff));
                        print_grid(t2,F_OH,F_OW,out_grid);
                        $display("");
                    end
                end
                $display("================================================================");
                if (fail_c==0)
                    $display("TEST F: ALL %0d OUTPUTS CORRECT  (%0d x %0d)",
                             pass_c,F_RK,F_RN);
                else
                    $display("TEST F: %0d PASS  %0d FAIL",pass_c,fail_c);
                $display("================================================================");
            end
        end

        // ================================================================
        // TEST G  -  5x5x1 | 3x3 | stride=1 pad=0 | 6 filters
        // REAL_K=9  (K tiles: 4+4+1  ->  last K-tile has only 1 row!)
        // REAL_M=9  (M%4=1  ->  pack-buf flush at 1 valid byte)
        // REAL_N=6  (N tiles: 4+2)
        // Triple non-alignment: K%4!=0, N%4!=0, M%4!=0 simultaneously
        // Total tile passes: 6
        // ================================================================
        $display("");
        $display("================================================================");
        $display("TEST G  -  5x5x1 | 3x3 kernel | 6 filters | stride=1 pad=0");
        $display("  REAL_K=9   REAL_M=9   REAL_N=6  |  6 tile passes");
        $display("  K tiles : 4 + 4 + 1  (LAST TILE = 1 ROW ONLY!)");
        $display("  N tiles : 4 + 2");
        $display("  M%%4=1  : pack-buf flushes after just 1 byte");
        $display("  TRIPLE non-alignment: K%%4!=0  N%%4!=0  M%%4!=0");
        $display("  Expected (uniform): f0=-8 f1=-16 f2=-24 f3=-32 f4=-40 f5=-48");
        $display("================================================================");
        begin
            logic signed [7:0] img_g  [G_IN_H*G_IN_W];
            logic signed [7:0] kern_g [G_RM*G_RN];
            logic signed [7:0] sobelX [9];
            sobelX = '{8'sd1,8'sd0,-8'sd1,8'sd2,8'sd0,-8'sd2,8'sd1,8'sd0,-8'sd1};

            for (int r=0;r<G_IN_H;r++)
                for (int c=0;c<G_IN_W;c++)
                    img_g[r*G_IN_W+c] = 8'(r*G_IN_W+c+1);

            for (int t=0;t<G_RM;t++)
                for (int f=0;f<G_RN;f++)
                    kern_g[t*G_RN+f] = 8'($signed(sobelX[t])*(f+1));

            for (int i=0;i<1024;i++) begin dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; end
            @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;
            for (int r=0;r<G_IN_H;r++)
                for (int c=0;c<G_IN_W;c++)
                    set_x_byte(r*G_IN_W+c, img_g[r*G_IN_W+c]);
            for (int t=0;t<G_RM;t++)
                for (int f=0;f<G_RN;f++)
                    set_y_byte(t*G_RN+f, kern_g[t*G_RN+f]);
            for (int i=0;i<G_RK;i++) dram_b[i]=32'd0;

            test_sel=2; rst_e=1; rst_f=1; rst_g=1;
            @(posedge clk); #1; rst_g=0;
            wait(done_g===1'b1);
            @(posedge clk); #1; @(posedge clk); #1;

            begin
                int pass_c, fail_c;
                logic signed [7:0] out_grid [G_RK];
                pass_c=0; fail_c=0;
                $display("");
                for (int f=0;f<G_RN;f++) begin
                    int fp,ff; fp=0; ff=0;
                    for (int op=0;op<G_RK;op++) begin
                        logic signed [7:0] gv, hv;
                        logic signed [31:0] raw_acc;
                        raw_acc = gold_conv(img_g,kern_g,G_IN_H,G_IN_W,G_K_H,G_K_W,
                                           G_ST,G_PD,G_RM,G_RN,op,f);
                        gv = 8'(raw_acc);  // raw 8-bit truncation (matches hardware)
                        hv = z_byte(op*G_RN+f);
                        out_grid[op]=hv;
                        if (hv===gv) begin pass_c++; fp++; end
                        else begin
                            fail_c++; ff++;
                            $display("  FAIL: op=%0d f=%0d  got=%0d  gold=%0d  raw_acc=%0d",
                                     op, f, hv, gv, raw_acc);
                        end
                    end
                    begin
                        string t2;
                        $sformat(t2,"Filter %0d  (scale=%0d, expected %0d)  %s",
                                 f,f+1,-(f+1)*8,(ff==0)?"PASS":$sformatf("FAIL(%0d)",ff));
                        print_grid(t2,G_OH,G_OW,out_grid);
                        $display("");
                    end
                end
                $display("================================================================");
                if (fail_c==0)
                    $display("TEST G: ALL %0d OUTPUTS CORRECT  (%0d x %0d)",
                             pass_c,G_RK,G_RN);
                else
                    $display("TEST G: %0d PASS  %0d FAIL",pass_c,fail_c);
                $display("================================================================");
            end
        end

        $display("");
        $display("================================================================");
        $display("ALL STRESS TESTS COMPLETE");
        $display("================================================================");
        #200;
        $finish;
    end

endmodule
