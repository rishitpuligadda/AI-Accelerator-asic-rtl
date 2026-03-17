// =============================================================================
// tb_conv.sv  -  conv systolic array testbench
//
// Test A: 4x4x1 input, 3x3 kernel, 1 filter, stride=1, pad=0
//   OUT_H=OUT_W=2, REAL_K=4, REAL_M=9, REAL_N=1
//   Uses Sobel-X kernel, gold computed in SV loops
//
// Test B: 4x4x1 input, 3x3 kernel, 4 filters, stride=1, pad=0
//   OUT_H=OUT_W=2, REAL_K=4, REAL_M=9, REAL_N=4
//   Gold computed in SV loops using same requant formula as HW
//
// Gold model:
//   acc      = sum over REAL_M of im2col_row[k] * kernel[k][out_ch]
//   biased   = acc + bias[out_pixel]
//   scaled   = biased * M_int
//   shifted  = (scaled + (1 << (shift-1))) >> shift
//   out      = clip(shifted, -128, 127)   -- no ReLU
// =============================================================================

`timescale 1ns/1ps

module tb_conv;

    localparam int TILE_MAX = 4;
    localparam int AW       = 8;

    logic clk = 0;
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // DRAM models
    // -------------------------------------------------------------------------
    reg [31:0] dram_x [0:1023];
    reg [31:0] dram_y [0:1023];
    reg [31:0] dram_b [0:1023];
    reg [31:0] dram_z [0:1023];

    logic [1:0] layer_sel;

    // -------------------------------------------------------------------------
    // DUT A  -  Test A (1 filter)
    // IN_H=4, IN_W=4, IN_C=1, OUT_C=1, K_H=3, K_W=3, STRIDE=1, PAD=0
    // OUT_H=2, OUT_W=2, REAL_K=4, REAL_M=9, REAL_N=1
    // -------------------------------------------------------------------------
    logic        rstA, doneA;
    logic [15:0] xAa, yAa, zAa, bAa;
    logic        xAe, yAe, zAe, bAe;
    logic [3:0]  zAm;
    logic [31:0] xAd, yAd, zAd, bAd;
    logic dA_sa_csb0, dA_sa_web0; logic [3:0] dA_sa_wm; logic [AW-1:0] dA_sa_a0; logic [31:0] dA_sa_di;
    logic dA_sa_csb1; logic [AW-1:0] dA_sa_a1;
    logic dA_sb_csb0, dA_sb_web0; logic [3:0] dA_sb_wm; logic [AW-1:0] dA_sb_a0; logic [31:0] dA_sb_di;
    logic dA_sb_csb1; logic [AW-1:0] dA_sb_a1;
    logic dA_sc_csb0, dA_sc_web0; logic [3:0] dA_sc_wm; logic [AW-1:0] dA_sc_a0; logic [31:0] dA_sc_di;
    logic dA_sc_csb1; logic [AW-1:0] dA_sc_a1;

    // -------------------------------------------------------------------------
    // DUT B  -  Test B (4 filters)
    // IN_H=4, IN_W=4, IN_C=1, OUT_C=4, K_H=3, K_W=3, STRIDE=1, PAD=0
    // OUT_H=2, OUT_W=2, REAL_K=4, REAL_M=9, REAL_N=4
    // -------------------------------------------------------------------------
    logic        rstB, doneB;
    logic [15:0] xBa, yBa, zBa, bBa;
    logic        xBe, yBe, zBe, bBe;
    logic [3:0]  zBm;
    logic [31:0] xBd, yBd, zBd, bBd;
    logic dB_sa_csb0, dB_sa_web0; logic [3:0] dB_sa_wm; logic [AW-1:0] dB_sa_a0; logic [31:0] dB_sa_di;
    logic dB_sa_csb1; logic [AW-1:0] dB_sa_a1;
    logic dB_sb_csb0, dB_sb_web0; logic [3:0] dB_sb_wm; logic [AW-1:0] dB_sb_a0; logic [31:0] dB_sb_di;
    logic dB_sb_csb1; logic [AW-1:0] dB_sb_a1;
    logic dB_sc_csb0, dB_sc_web0; logic [3:0] dB_sc_wm; logic [AW-1:0] dB_sc_a0; logic [31:0] dB_sc_di;
    logic dB_sc_csb1; logic [AW-1:0] dB_sc_a1;

    // -------------------------------------------------------------------------
    // DUT C  -  Test C (6x6x1, 3x3 kernel, 2 filters)
    // IN_H=6, IN_W=6, IN_C=1, OUT_C=2, K_H=3, K_W=3, STRIDE=1, PAD=0
    // OUT_H=4, OUT_W=4, REAL_K=16, REAL_M=9, REAL_N=2
    // -------------------------------------------------------------------------
    logic        rstC, doneC;
    logic [15:0] xCa, yCa, zCa, bCa;
    logic        xCe, yCe, zCe, bCe;
    logic [3:0]  zCm;
    logic [31:0] xCd, yCd, zCd, bCd;
    logic dC_sa_csb0, dC_sa_web0; logic [3:0] dC_sa_wm; logic [AW-1:0] dC_sa_a0; logic [31:0] dC_sa_di;
    logic dC_sa_csb1; logic [AW-1:0] dC_sa_a1;
    logic dC_sb_csb0, dC_sb_web0; logic [3:0] dC_sb_wm; logic [AW-1:0] dC_sb_a0; logic [31:0] dC_sb_di;
    logic dC_sb_csb1; logic [AW-1:0] dC_sb_a1;
    logic dC_sc_csb0, dC_sc_web0; logic [3:0] dC_sc_wm; logic [AW-1:0] dC_sc_a0; logic [31:0] dC_sc_di;
    logic dC_sc_csb1; logic [AW-1:0] dC_sc_a1;

    // -------------------------------------------------------------------------
    // Shared SRAM outputs
    // -------------------------------------------------------------------------
    logic [31:0] sa_do0, sa_do1, sb_do0, sb_do1, sc_do0, sc_do1;

    // -------------------------------------------------------------------------
    // SRAM mux
    // -------------------------------------------------------------------------
    logic sa_csb0, sa_web0; logic [3:0] sa_wm; logic [AW-1:0] sa_a0; logic [31:0] sa_di;
    logic sa_csb1; logic [AW-1:0] sa_a1;
    logic sb_csb0, sb_web0; logic [3:0] sb_wm; logic [AW-1:0] sb_a0; logic [31:0] sb_di;
    logic sb_csb1; logic [AW-1:0] sb_a1;
    logic sc_csb0, sc_web0; logic [3:0] sc_wm; logic [AW-1:0] sc_a0; logic [31:0] sc_di;
    logic sc_csb1; logic [AW-1:0] sc_a1;

    assign sa_csb0 = (layer_sel==2)?dC_sa_csb0:(layer_sel==1)?dB_sa_csb0:dA_sa_csb0;
    assign sa_web0 = (layer_sel==2)?dC_sa_web0:(layer_sel==1)?dB_sa_web0:dA_sa_web0;
    assign sa_wm   = (layer_sel==2)?dC_sa_wm  :(layer_sel==1)?dB_sa_wm  :dA_sa_wm;
    assign sa_a0   = (layer_sel==2)?dC_sa_a0  :(layer_sel==1)?dB_sa_a0  :dA_sa_a0;
    assign sa_di   = (layer_sel==2)?dC_sa_di  :(layer_sel==1)?dB_sa_di  :dA_sa_di;
    assign sa_csb1 = (layer_sel==2)?dC_sa_csb1:(layer_sel==1)?dB_sa_csb1:dA_sa_csb1;
    assign sa_a1   = (layer_sel==2)?dC_sa_a1  :(layer_sel==1)?dB_sa_a1  :dA_sa_a1;

    assign sb_csb0 = (layer_sel==2)?dC_sb_csb0:(layer_sel==1)?dB_sb_csb0:dA_sb_csb0;
    assign sb_web0 = (layer_sel==2)?dC_sb_web0:(layer_sel==1)?dB_sb_web0:dA_sb_web0;
    assign sb_wm   = (layer_sel==2)?dC_sb_wm  :(layer_sel==1)?dB_sb_wm  :dA_sb_wm;
    assign sb_a0   = (layer_sel==2)?dC_sb_a0  :(layer_sel==1)?dB_sb_a0  :dA_sb_a0;
    assign sb_di   = (layer_sel==2)?dC_sb_di  :(layer_sel==1)?dB_sb_di  :dA_sb_di;
    assign sb_csb1 = (layer_sel==2)?dC_sb_csb1:(layer_sel==1)?dB_sb_csb1:dA_sb_csb1;
    assign sb_a1   = (layer_sel==2)?dC_sb_a1  :(layer_sel==1)?dB_sb_a1  :dA_sb_a1;

    assign sc_csb0 = (layer_sel==2)?dC_sc_csb0:(layer_sel==1)?dB_sc_csb0:dA_sc_csb0;
    assign sc_web0 = (layer_sel==2)?dC_sc_web0:(layer_sel==1)?dB_sc_web0:dA_sc_web0;
    assign sc_wm   = (layer_sel==2)?dC_sc_wm  :(layer_sel==1)?dB_sc_wm  :dA_sc_wm;
    assign sc_a0   = (layer_sel==2)?dC_sc_a0  :(layer_sel==1)?dB_sc_a0  :dA_sc_a0;
    assign sc_di   = (layer_sel==2)?dC_sc_di  :(layer_sel==1)?dB_sc_di  :dA_sc_di;
    assign sc_csb1 = (layer_sel==2)?dC_sc_csb1:(layer_sel==1)?dB_sc_csb1:dA_sc_csb1;
    assign sc_a1   = (layer_sel==2)?dC_sc_a1  :(layer_sel==1)?dB_sc_a1  :dA_sc_a1;

    // -------------------------------------------------------------------------
    // SRAMs
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
    // DRAM models
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        case (layer_sel)
            2'd0: begin
                if (xAe) xAd <= dram_x[xAa];
                if (yAe) yAd <= dram_y[yAa];
                if (bAe) bAd <= dram_b[bAa];
                if (zAe) begin
                    if (zAm[0]) dram_z[zAa][ 7: 0] <= zAd[ 7: 0];
                    if (zAm[1]) dram_z[zAa][15: 8] <= zAd[15: 8];
                    if (zAm[2]) dram_z[zAa][23:16] <= zAd[23:16];
                    if (zAm[3]) dram_z[zAa][31:24] <= zAd[31:24];
                end
            end
            2'd1: begin
                if (xBe) xBd <= dram_x[xBa];
                if (yBe) yBd <= dram_y[yBa];
                if (bBe) bBd <= dram_b[bBa];
                if (zBe) begin
                    if (zBm[0]) dram_z[zBa][ 7: 0] <= zBd[ 7: 0];
                    if (zBm[1]) dram_z[zBa][15: 8] <= zBd[15: 8];
                    if (zBm[2]) dram_z[zBa][23:16] <= zBd[23:16];
                    if (zBm[3]) dram_z[zBa][31:24] <= zBd[31:24];
                end
            end
            2'd2: begin
                if (xCe) xCd <= dram_x[xCa];
                if (yCe) yCd <= dram_y[yCa];
                if (bCe) bCd <= dram_b[bCa];
                if (zCe) begin
                    if (zCm[0]) dram_z[zCa][ 7: 0] <= zCd[ 7: 0];
                    if (zCm[1]) dram_z[zCa][15: 8] <= zCd[15: 8];
                    if (zCm[2]) dram_z[zCa][23:16] <= zCd[23:16];
                    if (zCm[3]) dram_z[zCa][31:24] <= zCd[31:24];
                end
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------------------------
    // DUT instantiations
    // -------------------------------------------------------------------------
    // Test A: 4x4x1 -> 3x3x1 -> 2x2x1, last layer
    // M_int=32768, shift=15 -> scale factor = 32768/32768 = 1.0 (identity)
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX), .IN_H(4), .IN_W(4), .IN_C(1),
        .OUT_C(1), .K_H(3), .K_W(3), .STRIDE(1), .PAD(0), .AW(AW)
    ) dutA (
        .clk(clk), .rst(rstA), .done(doneA),
        .is_last_layer(1'b1), .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .M_int(16'd32768), .shift(5'd15),
        .ext_x_addr_o(xAa), .ext_x_rd_en_o(xAe), .ext_x_data_i(xAd),
        .ext_y_addr_o(yAa), .ext_y_rd_en_o(yAe), .ext_y_data_i(yAd),
        .ext_z_addr_o(zAa), .ext_z_wr_en_o(zAe), .ext_z_data_o(zAd), .ext_z_wmask_o(zAm),
        .ext_b_addr_o(bAa), .ext_b_rd_en_o(bAe), .ext_b_data_i(bAd),
        .sram_a_csb0(dA_sa_csb0), .sram_a_web0(dA_sa_web0), .sram_a_wmask0(dA_sa_wm),
        .sram_a_addr0(dA_sa_a0), .sram_a_din0(dA_sa_di), .sram_a_dout0(sa_do0),
        .sram_a_csb1(dA_sa_csb1), .sram_a_addr1(dA_sa_a1), .sram_a_dout1(sa_do1),
        .sram_b_csb0(dA_sb_csb0), .sram_b_web0(dA_sb_web0), .sram_b_wmask0(dA_sb_wm),
        .sram_b_addr0(dA_sb_a0), .sram_b_din0(dA_sb_di), .sram_b_dout0(sb_do0),
        .sram_b_csb1(dA_sb_csb1), .sram_b_addr1(dA_sb_a1), .sram_b_dout1(sb_do1),
        .sram_c_csb0(dA_sc_csb0), .sram_c_web0(dA_sc_web0), .sram_c_wmask0(dA_sc_wm),
        .sram_c_addr0(dA_sc_a0), .sram_c_din0(dA_sc_di), .sram_c_dout0(sc_do0),
        .sram_c_csb1(dA_sc_csb1), .sram_c_addr1(dA_sc_a1), .sram_c_dout1(sc_do1));

    // Test B: 4x4x1 -> 3x3x4 -> 2x2x4, last layer
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX), .IN_H(4), .IN_W(4), .IN_C(1),
        .OUT_C(4), .K_H(3), .K_W(3), .STRIDE(1), .PAD(0), .AW(AW)
    ) dutB (
        .clk(clk), .rst(rstB), .done(doneB),
        .is_last_layer(1'b1), .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .M_int(16'd32768), .shift(5'd15),
        .ext_x_addr_o(xBa), .ext_x_rd_en_o(xBe), .ext_x_data_i(xBd),
        .ext_y_addr_o(yBa), .ext_y_rd_en_o(yBe), .ext_y_data_i(yBd),
        .ext_z_addr_o(zBa), .ext_z_wr_en_o(zBe), .ext_z_data_o(zBd), .ext_z_wmask_o(zBm),
        .ext_b_addr_o(bBa), .ext_b_rd_en_o(bBe), .ext_b_data_i(bBd),
        .sram_a_csb0(dB_sa_csb0), .sram_a_web0(dB_sa_web0), .sram_a_wmask0(dB_sa_wm),
        .sram_a_addr0(dB_sa_a0), .sram_a_din0(dB_sa_di), .sram_a_dout0(sa_do0),
        .sram_a_csb1(dB_sa_csb1), .sram_a_addr1(dB_sa_a1), .sram_a_dout1(sa_do1),
        .sram_b_csb0(dB_sb_csb0), .sram_b_web0(dB_sb_web0), .sram_b_wmask0(dB_sb_wm),
        .sram_b_addr0(dB_sb_a0), .sram_b_din0(dB_sb_di), .sram_b_dout0(sb_do0),
        .sram_b_csb1(dB_sb_csb1), .sram_b_addr1(dB_sb_a1), .sram_b_dout1(sb_do1),
        .sram_c_csb0(dB_sc_csb0), .sram_c_web0(dB_sc_web0), .sram_c_wmask0(dB_sc_wm),
        .sram_c_addr0(dB_sc_a0), .sram_c_din0(dB_sc_di), .sram_c_dout0(sc_do0),
        .sram_c_csb1(dB_sc_csb1), .sram_c_addr1(dB_sc_a1), .sram_c_dout1(sc_do1));

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

    // Gold requant - same formula as hw (no ReLU)
    function automatic logic signed [7:0] requant(
        input logic signed [31:0] acc,
        input logic signed [31:0] bias,
        input logic        [15:0] M_int,
        input logic         [4:0] shift
    );
        logic signed [31:0] biased;
        logic signed [47:0] scaled;
        logic signed [31:0] shifted;
        biased  = acc + bias;
        scaled  = $signed(48'(signed'(biased))) * $signed(16'(M_int));
        shifted = ($signed(scaled) + $signed(48'(1) << (shift - 1))) >>> shift;
        if      (shifted >  127) return  8'sd127;
        else if (shifted < -128) return -8'sd128;
        else                     return  shifted[7:0];
    endfunction

    // im2col lookup for 4x4x1 input, 3x3 kernel, stride=1, pad=0
    // returns the image byte at position (oh*1+kh, ow*1+kw)
    function automatic logic signed [7:0] im2col_val(
        input logic signed [7:0] img[4][4],
        input int out_pixel,   // 0..3 (oh*OUT_W + ow)
        input int kern_elem    // 0..8 (kh*K_W + kw)
    );
        int oh, ow, kh, kw, sh, sw;
        oh = out_pixel / 2;
        ow = out_pixel % 2;
        kh = kern_elem / 3;
        kw = kern_elem % 3;
        sh = oh + kh;
        sw = ow + kw;
        return img[sh][sw];
    endfunction

    int pass_c, fail_c, total_pass, total_fail;

    // =========================================================================
    // Stimulus
    // =========================================================================
    initial begin
        rstA=1; rstB=1; rstC=1; layer_sel=2'd0;
        total_pass=0; total_fail=0;

        for (int a=0; a<1024; a++) begin
            dram_x[a]=32'd0; dram_y[a]=32'd0;
            dram_b[a]=32'd0; dram_z[a]=32'd0;
        end

        // =====================================================================
        // TEST A: 4x4x1 image, Sobel-X 3x3 kernel, 1 filter
        //
        // image:
        //   1  2  3  4
        //   5  6  7  8
        //   9 10 11 12
        //  13 14 15 16
        //
        // Sobel-X kernel:
        //   1  0 -1
        //   2  0 -2
        //   1  0 -1
        //
        // Expected (exact int, no scale needed, bias=0):
        //   out[0,0] = -8   out[0,1] = -8
        //   out[1,0] = -8   out[1,1] = -8
        // =====================================================================
        $display("========================================");
        $display("TEST A: 4x4x1 Sobel-X, 1 filter");
        $display("========================================");

        begin
            logic signed [7:0] img[4][4];
            logic signed [7:0] kern[9][1];
            logic signed [7:0] gold_A[4][1];
            logic signed [31:0] acc;
            int bi;

            // image (stored flat in dram_x, row-major)
            img[0]='{8'sd1, 8'sd2, 8'sd3, 8'sd4};
            img[1]='{8'sd5, 8'sd6, 8'sd7, 8'sd8};
            img[2]='{8'sd9, 8'sd10,8'sd11,8'sd12};
            img[3]='{8'sd13,8'sd14,8'sd15,8'sd16};

            for (int r=0; r<4; r++)
                for (int c=0; c<4; c++)
                    set_x_byte(r*4+c, img[r][c]);

            // Sobel-X kernel (REAL_M=9 rows, REAL_N=1 cols in dram_y)
            kern[0]='{8'sd1};  kern[1]='{8'sd0};  kern[2]='{-8'sd1};
            kern[3]='{8'sd2};  kern[4]='{8'sd0};  kern[5]='{-8'sd2};
            kern[6]='{8'sd1};  kern[7]='{8'sd0};  kern[8]='{-8'sd1};

            // col-tile 0 (only 1 filter so REAL_N=1, single col-tile)
            bi = 0;
            for (int k=0; k<9; k++)
                set_y_byte(bi++, kern[k][0]);

            // bias = 0 for all output pixels
            for (int i=0; i<4; i++) dram_b[i] = 32'd0;

            // Gold: compute im2col @ kernel
            for (int op=0; op<4; op++) begin
                acc = 32'sd0;
                for (int ke=0; ke<9; ke++)
                    acc += $signed(32'(im2col_val(img, op, ke))) *
                           $signed(32'(kern[ke][0]));
                gold_A[op][0] = requant(acc, 32'sd0, 16'd32768, 5'd15);
            end

            $display("  Gold output pixels:");
            for (int op=0; op<4; op++)
                $display("    out[%0d,0] = %0d", op, gold_A[op][0]);
        end

        #50; rstA = 0;
        wait (doneA === 1'b1); #20;

        // DEBUG: raw dram_z words
        $display("  [DBG] dram_z after Test A:");
        for (int a=0; a<4; a++)
            $display("    dram_z[%0d]=0x%08x b0=%0d b1=%0d b2=%0d b3=%0d",
                a, dram_z[a],
                $signed(dram_z[a][ 7: 0]),
                $signed(dram_z[a][15: 8]),
                $signed(dram_z[a][23:16]),
                $signed(dram_z[a][31:24]));

        pass_c=0; fail_c=0;
        begin
            logic signed [7:0] img[4][4];
            logic signed [7:0] kern[9][1];
            logic signed [7:0] gold_A[4][1];
            logic signed [31:0] acc;

            img[0]='{8'sd1, 8'sd2, 8'sd3, 8'sd4};
            img[1]='{8'sd5, 8'sd6, 8'sd7, 8'sd8};
            img[2]='{8'sd9, 8'sd10,8'sd11,8'sd12};
            img[3]='{8'sd13,8'sd14,8'sd15,8'sd16};
            kern[0]='{8'sd1};  kern[1]='{8'sd0};  kern[2]='{-8'sd1};
            kern[3]='{8'sd2};  kern[4]='{8'sd0};  kern[5]='{-8'sd2};
            kern[6]='{8'sd1};  kern[7]='{8'sd0};  kern[8]='{-8'sd1};

            for (int op=0; op<4; op++) begin
                acc = 32'sd0;
                for (int ke=0; ke<9; ke++)
                    acc += $signed(32'(im2col_val(img, op, ke))) *
                           $signed(32'(kern[ke][0]));
                gold_A[op][0] = requant(acc, 32'sd0, 16'd32768, 5'd15);
            end

            for (int op=0; op<4; op++) begin
                logic signed [7:0] got, exp;
                got = z_byte(op);
                exp = gold_A[op][0];
                if (got !== exp) begin
                    $display("  FAIL out[%0d,0]: got=%0d exp=%0d", op, got, exp);
                    fail_c++;
                end else begin
                    $display("  PASS out[%0d,0]: %0d", op, got);
                    pass_c++;
                end
            end
        end

        $display("  Test A: %0d PASS / %0d FAIL", pass_c, fail_c);
        total_pass += pass_c; total_fail += fail_c;

        // =====================================================================
        // TEST B: 4x4x1 image, 4 different 3x3 filters
        //
        // Same image as Test A.
        // 4 filters (edge detectors):
        //   f0: Sobel-X   f1: Sobel-Y   f2: all ones   f3: diagonal
        //
        // Output: 4 pixels x 4 filters = 16 values
        // =====================================================================
        $display("\n========================================");
        $display("TEST B: 4x4x1, 4 filters");
        $display("========================================");

        rstA=1; layer_sel=2'd1; rstB=1;
        for (int a=0; a<1024; a++) begin
            dram_x[a]=32'd0; dram_y[a]=32'd0;
            dram_b[a]=32'd0; dram_z[a]=32'd0;
        end

        begin
            logic signed [7:0] img[4][4];
            // kern[kernel_elem][filter]
            logic signed [7:0] kern[9][4];
            logic signed [31:0] bias_B[4];
            logic signed [7:0]  gold_B[4][4];
            logic signed [31:0] acc;
            int bi;

            img[0]='{8'sd1, 8'sd2, 8'sd3, 8'sd4};
            img[1]='{8'sd5, 8'sd6, 8'sd7, 8'sd8};
            img[2]='{8'sd9, 8'sd10,8'sd11,8'sd12};
            img[3]='{8'sd13,8'sd14,8'sd15,8'sd16};

            for (int r=0; r<4; r++)
                for (int c=0; c<4; c++)
                    set_x_byte(r*4+c, img[r][c]);

            // Filter 0: Sobel-X
            kern[0][0]= 8'sd1;  kern[1][0]= 8'sd0;  kern[2][0]=-8'sd1;
            kern[3][0]= 8'sd2;  kern[4][0]= 8'sd0;  kern[5][0]=-8'sd2;
            kern[6][0]= 8'sd1;  kern[7][0]= 8'sd0;  kern[8][0]=-8'sd1;

            // Filter 1: Sobel-Y
            kern[0][1]= 8'sd1;  kern[1][1]= 8'sd2;  kern[2][1]= 8'sd1;
            kern[3][1]= 8'sd0;  kern[4][1]= 8'sd0;  kern[5][1]= 8'sd0;
            kern[6][1]=-8'sd1;  kern[7][1]=-8'sd2;  kern[8][1]=-8'sd1;

            // Filter 2: all ones (box sum)
            for (int k=0; k<9; k++) kern[k][2] = 8'sd1;

            // Filter 3: diagonal emphasis
            kern[0][3]= 8'sd2;  kern[1][3]= 8'sd0;  kern[2][3]=-8'sd1;
            kern[3][3]= 8'sd0;  kern[4][3]= 8'sd1;  kern[5][3]= 8'sd0;
            kern[6][3]=-8'sd1;  kern[7][3]= 8'sd0;  kern[8][3]= 8'sd2;

            // Pack kernels into dram_y:
            // col-tile 0 (filters 0-3, all fit in TILE_MAX=4):
            //   REAL_M=9 rows x 4 cols packed sequentially
            bi = 0;
            for (int k=0; k<9; k++)
                for (int f=0; f<4; f++)
                    set_y_byte(bi++, kern[k][f]);

            // Small biases per output pixel (REAL_K=4 output pixels)
            bias_B[0]=32'sd10;  bias_B[1]=32'sd5;
            bias_B[2]=32'sd0;   bias_B[3]=-32'sd5;
            for (int i=0; i<4; i++) dram_b[i] = bias_B[i];

            // Gold
            for (int op=0; op<4; op++) begin
                for (int f=0; f<4; f++) begin
                    acc = 32'sd0;
                    for (int ke=0; ke<9; ke++)
                        acc += $signed(32'(im2col_val(img, op, ke))) *
                               $signed(32'(kern[ke][f]));
                    gold_B[op][f] = requant(acc, bias_B[op], 16'd32768, 5'd15);
                end
            end

            $display("  Gold (out_pixel x filter):");
            for (int op=0; op<4; op++) begin
                $write("    pix[%0d]: ", op);
                for (int f=0; f<4; f++) $write("%4d ", gold_B[op][f]);
                $write("\n");
            end

            #30; rstB = 0;
            wait (doneB === 1'b1); #20;

            pass_c=0; fail_c=0;
            // Output layout in dram_z:
            // pixel 0 -> bytes 0..3 (one byte per filter)
            // pixel 1 -> bytes 4..7
            // etc.
            for (int op=0; op<4; op++) begin
                for (int f=0; f<4; f++) begin
                    logic signed [7:0] got, exp;
                    got = z_byte(op*4 + f);
                    exp = gold_B[op][f];
                    if (got !== exp) begin
                        $display("  FAIL pix[%0d] filt[%0d]: got=%0d exp=%0d",
                                 op, f, got, exp);
                        fail_c++;
                    end else begin
                        $display("  PASS pix[%0d] filt[%0d]: %0d", op, f, got);
                        pass_c++;
                    end
                end
            end
        end

        $display("  Test B: %0d PASS / %0d FAIL", pass_c, fail_c);
        total_pass += pass_c; total_fail += fail_c;

        // =====================================================================
        // TEST C: 6x6x1, 3x3 kernel, 2 filters, stride=1, pad=0
        // OUT_H=4, OUT_W=4, REAL_K=16, REAL_M=9, REAL_N=2
        // Stresses 4 row-tiles (REAL_K=16 > TILE_MAX=4)
        // Gold computed from same requant formula
        // =====================================================================
        $display("\n========================================");
        $display("TEST C: 6x6x1, 3x3 kernel, 2 filters (4 row-tiles)");
        $display("========================================");

        rstB=1; layer_sel=2'd2; rstC=1;
        for (int a=0; a<1024; a++) begin
            dram_x[a]=32'd0; dram_y[a]=32'd0;
            dram_b[a]=32'd0; dram_z[a]=32'd0;
        end

        begin
            // 6x6 image (pi digits for fun)
            set_x_byte( 0, 3); set_x_byte( 1, 1); set_x_byte( 2, 4); set_x_byte( 3, 1);
            set_x_byte( 4, 5); set_x_byte( 5, 9); set_x_byte( 6, 2); set_x_byte( 7, 6);
            set_x_byte( 8, 5); set_x_byte( 9, 3); set_x_byte(10, 5); set_x_byte(11, 8);
            set_x_byte(12, 9); set_x_byte(13, 7); set_x_byte(14, 9); set_x_byte(15, 3);
            set_x_byte(16, 2); set_x_byte(17, 3); set_x_byte(18, 8); set_x_byte(19, 4);
            set_x_byte(20, 6); set_x_byte(21, 2); set_x_byte(22, 6); set_x_byte(23, 4);
            set_x_byte(24, 3); set_x_byte(25, 3); set_x_byte(26, 8); set_x_byte(27, 3);
            set_x_byte(28, 2); set_x_byte(29, 7); set_x_byte(30, 9); set_x_byte(31, 5);
            set_x_byte(32, 0); set_x_byte(33, 2); set_x_byte(34, 8); set_x_byte(35, 8);

            // kernels: [ke][filter]  REAL_M=9 rows x REAL_N=2 cols
            // f0=Sobel-X, f1=Sobel-Y, packed row-major per kernel element
            set_y_byte( 0,  1); set_y_byte( 1,  1);  // ke=0: f0=1  f1=1
            set_y_byte( 2,  0); set_y_byte( 3,  2);  // ke=1: f0=0  f1=2
            set_y_byte( 4, -1); set_y_byte( 5,  1);  // ke=2: f0=-1 f1=1
            set_y_byte( 6,  2); set_y_byte( 7,  0);  // ke=3: f0=2  f1=0
            set_y_byte( 8,  0); set_y_byte( 9,  0);  // ke=4: f0=0  f1=0
            set_y_byte(10, -2); set_y_byte(11,  0);  // ke=5: f0=-2 f1=0
            set_y_byte(12,  1); set_y_byte(13, -1);  // ke=6: f0=1  f1=-1
            set_y_byte(14,  0); set_y_byte(15, -2);  // ke=7: f0=0  f1=-2
            set_y_byte(16, -1); set_y_byte(17, -1);  // ke=8: f0=-1 f1=-1

            // bias per output pixel (REAL_K=16)
            dram_b[ 0]=32'sd10;  dram_b[ 1]=-32'sd5;
            dram_b[ 2]=32'sd3;   dram_b[ 3]=-32'sd8;
            dram_b[ 4]=32'sd12;  dram_b[ 5]=32'sd0;
            dram_b[ 6]=-32'sd3;  dram_b[ 7]=32'sd7;
            dram_b[ 8]=-32'sd10; dram_b[ 9]=32'sd5;
            dram_b[10]=-32'sd2;  dram_b[11]=32'sd8;
            dram_b[12]=32'sd1;   dram_b[13]=-32'sd6;
            dram_b[14]=32'sd4;   dram_b[15]=-32'sd1;

            #30; rstC = 0;
            wait (doneC === 1'b1); #20;

            // Gold: Python-verified values
            pass_c=0; fail_c=0;
            begin
                logic signed [7:0] gold[16][2];
                gold[ 0]='{8'sd3,   -8'sd13};
                gold[ 1]='{8'sd5,   -8'sd23};
                gold[ 2]='{8'sd9,   -8'sd3};
                gold[ 3]='{-8'sd26,  8'sd2};
                gold[ 4]='{8'sd11,   8'sd9};
                gold[ 5]='{8'sd13,   8'sd1};
                gold[ 6]='{8'sd11,  -8'sd3};
                gold[ 7]='{8'sd0,    8'sd10};
                gold[ 8]='{-8'sd11,  8'sd5};
                gold[ 9]='{8'sd13,   8'sd11};
                gold[10]='{8'sd11,  -8'sd1};
                gold[11]='{8'sd0,    8'sd4};
                gold[12]='{8'sd2,    8'sd4};
                gold[13]='{-8'sd1,   8'sd5};
                gold[14]='{8'sd8,    8'sd8};
                gold[15]='{-8'sd17, -8'sd9};

                // Output layout: pixel op -> bytes op*2 .. op*2+1
                // (REAL_N=2 so 2 bytes per output pixel, continuous)
                $display("  Z output (16 pixels x 2 filters):");
                for (int op=0; op<16; op++) begin
                    for (int f=0; f<2; f++) begin
                        logic signed [7:0] got, exp;
                        got = z_byte(op*2 + f);
                        exp = gold[op][f];
                        if (got !== exp) begin
                            $display("  FAIL pix[%0d] filt[%0d]: got=%0d exp=%0d",
                                     op, f, got, exp);
                            fail_c++;
                        end else begin
                            $display("  PASS pix[%0d] filt[%0d]: %0d", op, f, got);
                            pass_c++;
                        end
                    end
                end
            end
        end

        $display("  Test C: %0d PASS / %0d FAIL", pass_c, fail_c);
        total_pass += pass_c; total_fail += fail_c;

        // =====================================================================
        // Summary
        // =====================================================================
        $display("\n========================================");
        $display("  TOTAL: %0d PASS / %0d FAIL", total_pass, total_fail);
        if (total_fail==0) $display("  ALL CORRECT.");
        else               $display("  FAILURES - see above.");
        $display("========================================");
        $finish;
    end

    // Test C: 6x6x1, 3x3 kernel, 2 filters, last layer
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX), .IN_H(6), .IN_W(6), .IN_C(1),
        .OUT_C(2), .K_H(3), .K_W(3), .STRIDE(1), .PAD(0), .AW(AW)
    ) dutC (
        .clk(clk), .rst(rstC), .done(doneC),
        .is_last_layer(1'b1), .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .M_int(16'd32768), .shift(5'd15),
        .ext_x_addr_o(xCa), .ext_x_rd_en_o(xCe), .ext_x_data_i(xCd),
        .ext_y_addr_o(yCa), .ext_y_rd_en_o(yCe), .ext_y_data_i(yCd),
        .ext_z_addr_o(zCa), .ext_z_wr_en_o(zCe), .ext_z_data_o(zCd), .ext_z_wmask_o(zCm),
        .ext_b_addr_o(bCa), .ext_b_rd_en_o(bCe), .ext_b_data_i(bCd),
        .sram_a_csb0(dC_sa_csb0), .sram_a_web0(dC_sa_web0), .sram_a_wmask0(dC_sa_wm),
        .sram_a_addr0(dC_sa_a0), .sram_a_din0(dC_sa_di), .sram_a_dout0(sa_do0),
        .sram_a_csb1(dC_sa_csb1), .sram_a_addr1(dC_sa_a1), .sram_a_dout1(sa_do1),
        .sram_b_csb0(dC_sb_csb0), .sram_b_web0(dC_sb_web0), .sram_b_wmask0(dC_sb_wm),
        .sram_b_addr0(dC_sb_a0), .sram_b_din0(dC_sb_di), .sram_b_dout0(sb_do0),
        .sram_b_csb1(dC_sb_csb1), .sram_b_addr1(dC_sb_a1), .sram_b_dout1(sb_do1),
        .sram_c_csb0(dC_sc_csb0), .sram_c_web0(dC_sc_web0), .sram_c_wmask0(dC_sc_wm),
        .sram_c_addr0(dC_sc_a0), .sram_c_din0(dC_sc_di), .sram_c_dout0(sc_do0),
        .sram_c_csb1(dC_sc_csb1), .sram_c_addr1(dC_sc_a1), .sram_c_dout1(sc_do1));

    initial begin #8000000; $display("TIMEOUT"); $finish; end

endmodule
