// =============================================================================
// tb_conv_top.sv  -  Stress testbench for conv_top
//
// Instantiates three conv_top DUTs (E, F, G) ? each with its own
// parameterisation.  SRAMs now live inside conv_top, so this TB only
// needs to drive the DRAM interface and check outputs.
//
// Tests are run sequentially; only the active DUT has rst=0.
//
// Test E  -  8x8x2 | 3x3 | stride=1 pad=0 | 5 filters
//   REAL_K=36  REAL_M=18  REAL_N=5    18 tile passes
//   M%4=2  ->  pack-buf partial flush.  Multi-channel im2col.
//
// Test F  -  7x7x1 | 3x3 | stride=2 pad=1 | 5 filters
//   REAL_K=16  REAL_M=9   REAL_N=5     8 tile passes
//   Non-unit stride + padding.  N tiles: 4+1.
//
// Test G  -  5x5x1 | 3x3 | stride=1 pad=0 | 6 filters
//   REAL_K=9   REAL_M=9   REAL_N=6     6 tile passes
//   K tiles: 4+4+1  (last K-tile = 1 row only).
//   M%4=1.  Triple non-alignment.
// =============================================================================

`timescale 1ns/1ps

module tb_conv_top;

    // =========================================================================
    // Shared constants
    // =========================================================================
    localparam int TILE_MAX = 4;
    localparam int AW       = 8;

    // =========================================================================
    // Derived geometry per test
    // =========================================================================

    // --- E ---
    localparam int E_IN_H=8, E_IN_W=8, E_IN_C=2, E_K_H=3, E_K_W=3, E_ST=1, E_PD=0, E_OC=5;
    localparam int E_OH  = (E_IN_H+2*E_PD-E_K_H)/E_ST+1;   // 6
    localparam int E_OW  = (E_IN_W+2*E_PD-E_K_W)/E_ST+1;   // 6
    localparam int E_RK  = E_OH*E_OW;                        // 36
    localparam int E_RM  = E_K_H*E_K_W*E_IN_C;              // 18
    localparam int E_RN  = E_OC;                             // 5

    // --- F ---
    localparam int F_IN_H=7, F_IN_W=7, F_IN_C=1, F_K_H=3, F_K_W=3, F_ST=2, F_PD=1, F_OC=5;
    localparam int F_OH  = (F_IN_H+2*F_PD-F_K_H)/F_ST+1;   // 4
    localparam int F_OW  = (F_IN_W+2*F_PD-F_K_W)/F_ST+1;   // 4
    localparam int F_RK  = F_OH*F_OW;                        // 16
    localparam int F_RM  = F_K_H*F_K_W*F_IN_C;              // 9
    localparam int F_RN  = F_OC;                             // 5

    // --- G ---
    localparam int G_IN_H=5, G_IN_W=5, G_IN_C=1, G_K_H=3, G_K_W=3, G_ST=1, G_PD=0, G_OC=6;
    localparam int G_OH  = (G_IN_H+2*G_PD-G_K_H)/G_ST+1;   // 3
    localparam int G_OW  = (G_IN_W+2*G_PD-G_K_W)/G_ST+1;   // 3
    localparam int G_RK  = G_OH*G_OW;                        // 9
    localparam int G_RM  = G_K_H*G_K_W*G_IN_C;              // 9
    localparam int G_RN  = G_OC;                             // 6

    // =========================================================================
    // Clock
    // =========================================================================
    logic clk = 0;
    always #5 clk = ~clk;

    // =========================================================================
    // Shared DRAM model  (behavioural, word-addressed, byte-masked writes)
    // =========================================================================
    reg [31:0] dram_x[0:1023];
    reg [31:0] dram_y[0:1023];
    reg [31:0] dram_b[0:1023];
    reg [31:0] dram_z[0:1023];

    // Clear-Z helper ? driven by stimulus
    reg clear_z;
    always @(posedge clk)
        if (clear_z) for (int i=0; i<1024; i++) dram_z[i] <= 32'd0;

    // =========================================================================
    // Per-DUT reset / done
    // =========================================================================
    logic rst_e, rst_f, rst_g;
    logic done_e, done_f, done_g;

    // =========================================================================
    // Per-DUT DRAM address/enable buses
    //   (all three DUTs share the same DRAM arrays; the active DUT wins
    //    because only it has rst=0 and is thus driving valid addresses)
    // =========================================================================

    // -- E --
    logic [15:0] xe_addr, ye_addr, ze_addr, be_addr;
    logic        xe_en,   ye_en,   ze_en,   be_en;
    logic [3:0]  ze_mask; logic [31:0] ze_wdata;

    // -- F --
    logic [15:0] xf_addr, yf_addr, zf_addr, bf_addr;
    logic        xf_en,   yf_en,   zf_en,   bf_en;
    logic [3:0]  zf_mask; logic [31:0] zf_wdata;

    // -- G --
    logic [15:0] xg_addr, yg_addr, zg_addr, bg_addr;
    logic        xg_en,   yg_en,   zg_en,   bg_en;
    logic [3:0]  zg_mask; logic [31:0] zg_wdata;

    // =========================================================================
    // DRAM read model  (1-cycle registered read, shared data bus)
    //   Each DUT gets the same read data ? only the active one has valid
    //   addresses so the others will consume garbage, which is harmless
    //   while they are held in reset.
    // =========================================================================
    logic [15:0] x_addr_r, y_addr_r, b_addr_r;
    logic        x_en_r,   y_en_r,   b_en_r;

    // Mux the active DUT's read addresses onto the shared bus
    logic [1:0] test_sel;   // 0=E 1=F 2=G

    logic [15:0] mx_addr, my_addr, mb_addr;
    logic        mx_en,   my_en,   mb_en;

    always_comb begin
        case (test_sel)
            2'd0: begin mx_addr=xe_addr; my_addr=ye_addr; mb_addr=be_addr;
                        mx_en=xe_en;     my_en=ye_en;     mb_en=be_en; end
            2'd1: begin mx_addr=xf_addr; my_addr=yf_addr; mb_addr=bf_addr;
                        mx_en=xf_en;     my_en=yf_en;     mb_en=bf_en; end
            default: begin mx_addr=xg_addr; my_addr=yg_addr; mb_addr=bg_addr;
                           mx_en=xg_en;     my_en=yg_en;     mb_en=bg_en; end
        endcase
    end

    always_ff @(posedge clk) begin
        x_addr_r <= mx_addr; x_en_r <= mx_en;
        y_addr_r <= my_addr; y_en_r <= my_en;
        b_addr_r <= mb_addr; b_en_r <= mb_en;
    end

    logic [31:0] x_rdata, y_rdata, b_rdata;
    assign x_rdata = x_en_r ? dram_x[x_addr_r] : 32'hDEAD_BEEF;
    assign y_rdata = y_en_r ? dram_y[y_addr_r] : 32'hDEAD_BEEF;
    assign b_rdata = b_en_r ? dram_b[b_addr_r] : 32'hDEAD_BEEF;

    // DRAM Z write  (byte-masked, from whichever DUT is active)
    logic [15:0] mz_addr;
    logic        mz_en;
    logic [3:0]  mz_mask;
    logic [31:0] mz_wdata;

    always_comb begin
        case (test_sel)
            2'd0: begin mz_addr=ze_addr; mz_en=ze_en; mz_mask=ze_mask; mz_wdata=ze_wdata; end
            2'd1: begin mz_addr=zf_addr; mz_en=zf_en; mz_mask=zf_mask; mz_wdata=zf_wdata; end
            default: begin mz_addr=zg_addr; mz_en=zg_en; mz_mask=zg_mask; mz_wdata=zg_wdata; end
        endcase
    end

    always @(posedge clk) if (mz_en) begin
        if (mz_mask[0]) dram_z[mz_addr][ 7: 0] <= mz_wdata[ 7: 0];
        if (mz_mask[1]) dram_z[mz_addr][15: 8] <= mz_wdata[15: 8];
        if (mz_mask[2]) dram_z[mz_addr][23:16] <= mz_wdata[23:16];
        if (mz_mask[3]) dram_z[mz_addr][31:24] <= mz_wdata[31:24];
    end

    // =========================================================================
    // DUT E  ?  conv_top with Test-E parameters (default params, no overrides)
    // =========================================================================
    conv_top #(
        .TILE_MAX(TILE_MAX), .AW(AW),
        .IN_H(E_IN_H), .IN_W(E_IN_W), .IN_C(E_IN_C),
        .OUT_C(E_OC), .K_H(E_K_H), .K_W(E_K_W), .STRIDE(E_ST), .PAD(E_PD)
    ) dut_e (
        .clk           (clk),     .rst(rst_e),    .done(done_e),
        .is_last_layer (1'b1),    .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .ext_x_addr_o  (xe_addr), .ext_x_rd_en_o(xe_en),  .ext_x_data_i(x_rdata),
        .ext_y_addr_o  (ye_addr), .ext_y_rd_en_o(ye_en),  .ext_y_data_i(y_rdata),
        .ext_z_addr_o  (ze_addr), .ext_z_wr_en_o(ze_en),
        .ext_z_data_o  (ze_wdata),.ext_z_wmask_o(ze_mask),
        .ext_b_addr_o  (be_addr), .ext_b_rd_en_o(be_en),  .ext_b_data_i(b_rdata)
    );

    // =========================================================================
    // DUT F  ?  conv_top with Test-F parameters
    // =========================================================================
    conv_top #(
        .TILE_MAX(TILE_MAX), .AW(AW),
        .IN_H(F_IN_H), .IN_W(F_IN_W), .IN_C(F_IN_C),
        .OUT_C(F_OC), .K_H(F_K_H), .K_W(F_K_W), .STRIDE(F_ST), .PAD(F_PD)
    ) dut_f (
        .clk           (clk),     .rst(rst_f),    .done(done_f),
        .is_last_layer (1'b1),    .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .ext_x_addr_o  (xf_addr), .ext_x_rd_en_o(xf_en),  .ext_x_data_i(x_rdata),
        .ext_y_addr_o  (yf_addr), .ext_y_rd_en_o(yf_en),  .ext_y_data_i(y_rdata),
        .ext_z_addr_o  (zf_addr), .ext_z_wr_en_o(zf_en),
        .ext_z_data_o  (zf_wdata),.ext_z_wmask_o(zf_mask),
        .ext_b_addr_o  (bf_addr), .ext_b_rd_en_o(bf_en),  .ext_b_data_i(b_rdata)
    );

    // =========================================================================
    // DUT G  ?  conv_top with Test-G parameters
    // =========================================================================
    conv_top #(
        .TILE_MAX(TILE_MAX), .AW(AW),
        .IN_H(G_IN_H), .IN_W(G_IN_W), .IN_C(G_IN_C),
        .OUT_C(G_OC), .K_H(G_K_H), .K_W(G_K_W), .STRIDE(G_ST), .PAD(G_PD)
    ) dut_g (
        .clk           (clk),     .rst(rst_g),    .done(done_g),
        .is_last_layer (1'b1),    .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .ext_x_addr_o  (xg_addr), .ext_x_rd_en_o(xg_en),  .ext_x_data_i(x_rdata),
        .ext_y_addr_o  (yg_addr), .ext_y_rd_en_o(yg_en),  .ext_y_data_i(y_rdata),
        .ext_z_addr_o  (zg_addr), .ext_z_wr_en_o(zg_en),
        .ext_z_data_o  (zg_wdata),.ext_z_wmask_o(zg_mask),
        .ext_b_addr_o  (bg_addr), .ext_b_rd_en_o(bg_en),  .ext_b_data_i(b_rdata)
    );

    // =========================================================================
    // Utility tasks / functions  (identical semantics to tbStress.sv)
    // =========================================================================

    // Pack a signed byte into the correct byte-lane of the 32-bit DRAM word
    task automatic set_x_byte(input int idx, input logic signed [7:0] val);
        dram_x[idx/4][(idx%4)*8 +: 8] = val;
    endtask
    task automatic set_y_byte(input int idx, input logic signed [7:0] val);
        dram_y[idx/4][(idx%4)*8 +: 8] = val;
    endtask

    // Read a signed byte from the output DRAM
    function automatic logic signed [7:0] z_byte(input int flat);
        return $signed(dram_z[flat/4][(flat%4)*8 +: 8]);
    endfunction

    // Saturating clip to int8
    function automatic logic signed [7:0] clip8(input logic signed [31:0] v);
        if      (v >  127) return  8'sd127;
        else if (v < -128) return -8'sd128;
        else               return  v[7:0];
    endfunction

    // -------------------------------------------------------------------------
    // gold_conv  -  reference convolution (multi-channel, stride, pad)
    //
    // img  : flattened [IC][IH][IW]  (row-major within each channel)
    // kern : flattened [RM][RN]      (RM = KH*KW*IC, RN = OUT_C)
    // -------------------------------------------------------------------------
    function automatic logic signed [31:0] gold_conv(
        input logic signed [7:0] img  [],
        input logic signed [7:0] kern [],
        input int IH, IW, KH, KW, ST, PD, RM, RN, op, f
    );
        int OW, oh, ow;
        logic signed [31:0] acc;
        OW  = (IW + 2*PD - KW) / ST + 1;
        oh  = op / OW;
        ow  = op % OW;
        acc = 32'sd0;
        for (int t = 0; t < RM; t++) begin
            int ic, t_sp, kh_i, kw_i, sr, sc;
            ic   = t / (KH * KW);
            t_sp = t % (KH * KW);
            kh_i = t_sp / KW;
            kw_i = t_sp % KW;
            sr   = oh*ST + kh_i - PD;
            sc   = ow*ST + kw_i - PD;
            if (sr >= 0 && sr < IH && sc >= 0 && sc < IW)
                acc += $signed(32'(img[ic*IH*IW + sr*IW + sc]))
                     * $signed(32'(kern[t*RN + f]));
        end
        return acc;
    endfunction

    // -------------------------------------------------------------------------
    // print_grid  -  pretty-print a 2-D int8 output grid
    // -------------------------------------------------------------------------
    task automatic print_grid(
        input string title,
        input int    rows, cols,
        input logic signed [7:0] data []
    );
        $display("  %s", title);
        $write("          ");
        for (int c = 0; c < cols; c++) $write(" col%0d  ", c);
        $display("");
        $write("          ");
        for (int c = 0; c < cols; c++) $write("+------");
        $display("+");
        for (int r = 0; r < rows; r++) begin
            $write("  row%0d   ", r);
            for (int c = 0; c < cols; c++) $write("| %4d ", data[r*cols+c]);
            $display("|");
            $write("          ");
            for (int c = 0; c < cols; c++) $write("+------");
            $display("+");
        end
    endtask

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin
        #200_000_000;
        $display("WATCHDOG: simulation timeout ? hung DUT?");
        $finish;
    end

    // =========================================================================
    // Main stimulus
    // =========================================================================
    initial begin
        // All DUTs in reset, start with mux on E
        rst_e=1; rst_f=1; rst_g=1;
        test_sel=0; clear_z=0;
        for (int i=0; i<1024; i++) begin
            dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; dram_z[i]='0;
        end
        @(posedge clk); #1;

        $display("");
        $display("================================================================");
        $display("tb_conv_top  -  Stress test suite (conv_top wrapper)");
        $display("================================================================");

        // ====================================================================
        // TEST E  ?  8x8x2 | 3x3 | stride=1 pad=0 | 5 filters
        //   REAL_K=36 (9 tiles of 4, exact K-alignment)
        //   REAL_M=18 (M%4=2, pack-buf partial flush)
        //   REAL_N=5  (N tiles: 4+1, partial last N-tile)
        //   Multi-channel: IN_C=2  =>  im2col crosses channel boundary
        //   Total tile passes: 18
        // ====================================================================
        $display("");
        $display("================================================================");
        $display("TEST E  -  8x8x2 | 3x3 kernel | 5 filters | stride=1 pad=0");
        $display("  REAL_K=36  REAL_M=18  REAL_N=5  |  18 tile passes");
        $display("  K tiles : 9 x 4  (exact, no partial K)");
        $display("  N tiles : 4 + 1  (partial last N-tile)");
        $display("  M%%4    = 2  (pack-buf partial flush every tile)");
        $display("  IN_C=2  : im2col must stride correctly across both channels");
        $display("  Expected (uniform): f0=-16 f1=-32 f2=-48 f3=-64 f4=-80");
        $display("================================================================");
        begin
            logic signed [7:0] img_e  [E_IN_C*E_IN_H*E_IN_W];
            logic signed [7:0] kern_e [E_RM*E_RN];
            logic signed [7:0] sobelX [9];
            sobelX = '{8'sd1,8'sd0,-8'sd1,8'sd2,8'sd0,-8'sd2,8'sd1,8'sd0,-8'sd1};

            // image: ch0 = 1..64, ch1 = 65..128
            for (int ic=0; ic<E_IN_C; ic++)
                for (int r=0; r<E_IN_H; r++)
                    for (int c=0; c<E_IN_W; c++)
                        img_e[ic*E_IN_H*E_IN_W + r*E_IN_W + c] =
                            8'(ic*E_IN_H*E_IN_W + r*E_IN_W + c + 1);

            // weights: same sobelX pattern for each channel, scaled per filter
            for (int t=0; t<E_RM; t++) begin
                int t_sp; t_sp = t % (E_K_H*E_K_W);
                for (int f=0; f<E_RN; f++)
                    kern_e[t*E_RN+f] = 8'($signed(sobelX[t_sp]) * (f+1));
            end

            // --- Load DRAM ---
            for (int i=0; i<1024; i++) begin dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; end
            @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;

            for (int ic=0; ic<E_IN_C; ic++)
                for (int r=0; r<E_IN_H; r++)
                    for (int c=0; c<E_IN_W; c++)
                        set_x_byte(ic*E_IN_H*E_IN_W + r*E_IN_W + c,
                                   img_e[ic*E_IN_H*E_IN_W + r*E_IN_W + c]);

            for (int t=0; t<E_RM; t++)
                for (int f=0; f<E_RN; f++)
                    set_y_byte(t*E_RN+f, kern_e[t*E_RN+f]);

            for (int i=0; i<E_RK; i++) dram_b[i] = 32'd0;

            // --- Run DUT E ---
            test_sel=0;
            rst_e=1; rst_f=1; rst_g=1;
            @(posedge clk); #1;
            rst_e=0;
            wait(done_e === 1'b1);
            @(posedge clk); #1; @(posedge clk); #1;
            rst_e=1;

            // --- Check ---
            begin
                int pass_c, fail_c;
                logic signed [7:0] out_grid [E_RK];
                pass_c=0; fail_c=0;
                $display("");
                for (int f=0; f<E_RN; f++) begin
                    int ff; ff=0;
                    for (int op=0; op<E_RK; op++) begin
                        logic signed [7:0]  gv, hv;
                        logic signed [31:0] raw_acc;
                        raw_acc = gold_conv(img_e, kern_e,
                                            E_IN_H, E_IN_W, E_K_H, E_K_W,
                                            E_ST, E_PD, E_RM, E_RN, op, f);
                        gv = 8'(raw_acc);   // raw 8-bit truncation (matches HW)
                        hv = z_byte(op*E_RN + f);
                        out_grid[op] = hv;
                        if (hv === gv) pass_c++;
                        else begin
                            fail_c++; ff++;
                            $display("  FAIL E: op=%0d f=%0d  got=%0d  gold=%0d  raw_acc=%0d  z_idx=%0d",
                                     op, f, hv, gv, raw_acc, op*E_RN+f);
                        end
                    end
                    begin
                        string lbl;
                        $sformat(lbl, "Filter %0d  (expected %0d everywhere)  %s",
                                 f, -(f+1)*16, (ff==0)?"PASS":$sformatf("FAIL(%0d)",ff));
                        print_grid(lbl, E_OH, E_OW, out_grid);
                        $display("");
                    end
                end
                $display("================================================================");
                if (fail_c==0)
                    $display("TEST E: ALL %0d OUTPUTS CORRECT  (%0d x %0d)",
                             pass_c, E_RK, E_RN);
                else
                    $display("TEST E: %0d PASS  %0d FAIL", pass_c, fail_c);
                $display("================================================================");
            end
        end

        // ====================================================================
        // TEST F  ?  7x7x1 | 3x3 | stride=2 pad=1 | 5 filters
        //   REAL_K=16 (4 tiles of 4, exact K-alignment)
        //   REAL_N=5  (N tiles: 4+1, partial last N-tile)
        //   stride=2, pad=1: non-uniform spatial output
        //   Total tile passes: 8
        // ====================================================================
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

            for (int r=0; r<F_IN_H; r++)
                for (int c=0; c<F_IN_W; c++)
                    img_f[r*F_IN_W+c] = 8'(r*F_IN_W+c+1);

            for (int t=0; t<F_RM; t++)
                for (int f=0; f<F_RN; f++)
                    kern_f[t*F_RN+f] = 8'($signed(sobelX[t]) * (f+1));

            // --- Load DRAM ---
            for (int i=0; i<1024; i++) begin dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; end
            @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;

            for (int r=0; r<F_IN_H; r++)
                for (int c=0; c<F_IN_W; c++)
                    set_x_byte(r*F_IN_W+c, img_f[r*F_IN_W+c]);

            for (int t=0; t<F_RM; t++)
                for (int f=0; f<F_RN; f++)
                    set_y_byte(t*F_RN+f, kern_f[t*F_RN+f]);

            for (int i=0; i<F_RK; i++) dram_b[i] = 32'd0;

            // --- Run DUT F ---
            test_sel=1;
            rst_e=1; rst_f=1; rst_g=1;
            @(posedge clk); #1;
            rst_f=0;
            wait(done_f === 1'b1);
            @(posedge clk); #1; @(posedge clk); #1;
            rst_f=1;

            // --- Check ---
            begin
                int pass_c, fail_c;
                logic signed [7:0] out_grid [F_RK];
                pass_c=0; fail_c=0;
                $display("");
                for (int f=0; f<F_RN; f++) begin
                    int ff; ff=0;
                    for (int op=0; op<F_RK; op++) begin
                        logic signed [7:0]  gv, hv;
                        logic signed [31:0] raw_acc;
                        raw_acc = gold_conv(img_f, kern_f,
                                            F_IN_H, F_IN_W, F_K_H, F_K_W,
                                            F_ST, F_PD, F_RM, F_RN, op, f);
                        gv = 8'(raw_acc);
                        hv = z_byte(op*F_RN + f);
                        out_grid[op] = hv;
                        if (hv === gv) pass_c++;
                        else begin
                            fail_c++; ff++;
                            $display("  FAIL F: op=%0d f=%0d  got=%0d  gold=%0d  raw_acc=%0d",
                                     op, f, hv, gv, raw_acc);
                        end
                    end
                    begin
                        string lbl;
                        $sformat(lbl, "Filter %0d  (spatially varying)  %s",
                                 f, (ff==0)?"PASS":$sformatf("FAIL(%0d)",ff));
                        print_grid(lbl, F_OH, F_OW, out_grid);
                        $display("");
                    end
                end
                $display("================================================================");
                if (fail_c==0)
                    $display("TEST F: ALL %0d OUTPUTS CORRECT  (%0d x %0d)",
                             pass_c, F_RK, F_RN);
                else
                    $display("TEST F: %0d PASS  %0d FAIL", pass_c, fail_c);
                $display("================================================================");
            end
        end

        // ====================================================================
        // TEST G  ?  5x5x1 | 3x3 | stride=1 pad=0 | 6 filters
        //   REAL_K=9  (K tiles: 4+4+1 ? last K-tile = 1 row only!)
        //   REAL_M=9  (M%4=1 ? pack-buf flush at 1 valid byte)
        //   REAL_N=6  (N tiles: 4+2)
        //   Triple non-alignment: K%4!=0, N%4!=0, M%4!=0 simultaneously
        //   Total tile passes: 6
        // ====================================================================
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

            for (int r=0; r<G_IN_H; r++)
                for (int c=0; c<G_IN_W; c++)
                    img_g[r*G_IN_W+c] = 8'(r*G_IN_W+c+1);

            for (int t=0; t<G_RM; t++)
                for (int f=0; f<G_RN; f++)
                    kern_g[t*G_RN+f] = 8'($signed(sobelX[t]) * (f+1));

            // --- Load DRAM ---
            for (int i=0; i<1024; i++) begin dram_x[i]='0; dram_y[i]='0; dram_b[i]='0; end
            @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;

            for (int r=0; r<G_IN_H; r++)
                for (int c=0; c<G_IN_W; c++)
                    set_x_byte(r*G_IN_W+c, img_g[r*G_IN_W+c]);

            for (int t=0; t<G_RM; t++)
                for (int f=0; f<G_RN; f++)
                    set_y_byte(t*G_RN+f, kern_g[t*G_RN+f]);

            for (int i=0; i<G_RK; i++) dram_b[i] = 32'd0;

            // --- Run DUT G ---
            test_sel=2;
            rst_e=1; rst_f=1; rst_g=1;
            @(posedge clk); #1;
            rst_g=0;
            wait(done_g === 1'b1);
            @(posedge clk); #1; @(posedge clk); #1;
            rst_g=1;

            // --- Check ---
            begin
                int pass_c, fail_c;
                logic signed [7:0] out_grid [G_RK];
                pass_c=0; fail_c=0;
                $display("");
                for (int f=0; f<G_RN; f++) begin
                    int ff; ff=0;
                    for (int op=0; op<G_RK; op++) begin
                        logic signed [7:0]  gv, hv;
                        logic signed [31:0] raw_acc;
                        raw_acc = gold_conv(img_g, kern_g,
                                            G_IN_H, G_IN_W, G_K_H, G_K_W,
                                            G_ST, G_PD, G_RM, G_RN, op, f);
                        gv = 8'(raw_acc);
                        hv = z_byte(op*G_RN + f);
                        out_grid[op] = hv;
                        if (hv === gv) pass_c++;
                        else begin
                            fail_c++; ff++;
                            $display("  FAIL G: op=%0d f=%0d  got=%0d  gold=%0d  raw_acc=%0d",
                                     op, f, hv, gv, raw_acc);
                        end
                    end
                    begin
                        string lbl;
                        $sformat(lbl, "Filter %0d  (scale=%0d, expected %0d)  %s",
                                 f, f+1, -(f+1)*8,
                                 (ff==0)?"PASS":$sformatf("FAIL(%0d)",ff));
                        print_grid(lbl, G_OH, G_OW, out_grid);
                        $display("");
                    end
                end
                $display("================================================================");
                if (fail_c==0)
                    $display("TEST G: ALL %0d OUTPUTS CORRECT  (%0d x %0d)",
                             pass_c, G_RK, G_RN);
                else
                    $display("TEST G: %0d PASS  %0d FAIL", pass_c, fail_c);
                $display("================================================================");
            end
        end

        // ====================================================================
        // Summary
        // ====================================================================
        $display("");
        $display("================================================================");
        $display("ALL STRESS TESTS COMPLETE");
        $display("================================================================");
        #200;
        $finish;
    end

endmodule
