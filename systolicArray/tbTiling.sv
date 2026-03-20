// =============================================================================
// tb_tiling.sv  ?  Test D: tiling stress test
//
// 6󬝭 grayscale image, 3�3 Sobel-X kernel, 6 filters, stride=1, pad=0
//
//   OUT_H = (6+0-3)/1+1 = 4
//   OUT_W = 4
//   REAL_K = OUT_H � OUT_W = 16   ? tiles K in 4 passes of 4 rows each
//   REAL_M = 3󫢩 = 9
//   REAL_N = 6                    ? tiles N in 2 passes: 4 cols then 2
//
// tile_iter sequence (8 tile passes):
//   pass 0: row_tile=0,  col_tile=0  eff_rows=4 eff_cols=4
//   pass 1: row_tile=0,  col_tile=4  eff_rows=4 eff_cols=2
//   pass 2: row_tile=4,  col_tile=0  eff_rows=4 eff_cols=4
//   pass 3: row_tile=4,  col_tile=4  eff_rows=4 eff_cols=2
//   pass 4: row_tile=8,  col_tile=0  eff_rows=4 eff_cols=4
//   pass 5: row_tile=8,  col_tile=4  eff_rows=4 eff_cols=2
//   pass 6: row_tile=12, col_tile=0  eff_rows=4 eff_cols=4
//   pass 7: row_tile=12, col_tile=4  eff_rows=4 eff_cols=2
//
// Filters:
//   f0: Sobel-X  weight  1  ?  result = -8  at all interior positions
//   f1: Sobel-X  weight  2  ?  result = -16
//   f2: Sobel-X  weight  3  ?  result = -24
//   f3: Sobel-X  weight  4  ?  result = -32 (clips to -32, within [-128,127])
//   f4: Sobel-X  weight  5  ?  result = -40
//   f5: Sobel-X  weight  6  ?  result = -48
//
// Each filter is a scaled Sobel-X: kern_f[kh][kw] = scale * sobelX[kh][kw]
// so output_f = scale * sobelX_result = scale * (-8)
// =============================================================================

`timescale 1ns/1ps

module tb_tiling;

    localparam int TILE_MAX = 4;
    localparam int AW       = 8;

    // =========================================================================
    // Test D parameters
    // =========================================================================
    localparam int IN_H   = 6;
    localparam int IN_W   = 6;
    localparam int IN_C   = 1;
    localparam int K_H    = 3;
    localparam int K_W    = 3;
    localparam int STRIDE = 1;
    localparam int PAD    = 0;
    localparam int OUT_C  = 6;
    localparam int OUT_H  = (IN_H + 2*PAD - K_H) / STRIDE + 1;  // 4
    localparam int OUT_W  = (IN_W + 2*PAD - K_W) / STRIDE + 1;  // 4
    localparam int REAL_K = OUT_H * OUT_W;                        // 16
    localparam int REAL_M = K_H * K_W * IN_C;                    // 9
    localparam int REAL_N = OUT_C;                                // 6

    // =========================================================================
    // Clock
    // =========================================================================
    logic clk = 0;
    always #5 clk = ~clk;

    // =========================================================================
    // DRAM
    // =========================================================================
    reg [31:0] dram_x [0:1023];
    reg [31:0] dram_y [0:1023];
    reg [31:0] dram_b [0:1023];
    reg [31:0] dram_z [0:1023];
    reg clear_z;

    always @(posedge clk)
        if (clear_z)
            for (int i = 0; i < 1024; i++) dram_z[i] <= 32'd0;

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic        rst, done;
    logic [15:0] x_addr, y_addr, z_addr, b_addr;
    logic        x_en, y_en, z_en, b_en;
    logic [3:0]  z_mask;
    logic [31:0] x_data, y_data, z_data, b_data;
    logic [15:0] x_addr_r, y_addr_r, b_addr_r;
    logic        x_en_r, y_en_r, b_en_r;

    logic        sa_csb0, sa_web0; logic [3:0] sa_wm; logic [AW-1:0] sa_a0; logic [31:0] sa_di;
    logic        sa_csb1;          logic [AW-1:0] sa_a1; logic [31:0] sa_do0, sa_do1;
    logic        sb_csb0, sb_web0; logic [3:0] sb_wm; logic [AW-1:0] sb_a0; logic [31:0] sb_di;
    logic        sb_csb1;          logic [AW-1:0] sb_a1; logic [31:0] sb_do0, sb_do1;
    logic        sc_csb0, sc_web0; logic [3:0] sc_wm; logic [AW-1:0] sc_a0; logic [31:0] sc_di;
    logic        sc_csb1;          logic [AW-1:0] sc_a1; logic [31:0] sc_do0, sc_do1;

    // =========================================================================
    // DRAM models ? 1 cycle latency
    // =========================================================================
    always_ff @(posedge clk) begin
        x_addr_r <= x_addr; x_en_r <= x_en;
        y_addr_r <= y_addr; y_en_r <= y_en;
        b_addr_r <= b_addr; b_en_r <= b_en;
    end
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
    // SRAMs
    // =========================================================================
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

    // =========================================================================
    // DUT
    // =========================================================================
    systolic_pipe_conv #(
        .TILE_MAX(TILE_MAX), .IN_H(IN_H), .IN_W(IN_W), .IN_C(IN_C),
        .OUT_C(OUT_C), .K_H(K_H), .K_W(K_W), .STRIDE(STRIDE), .PAD(PAD), .AW(AW)
    ) dut (
        .clk(clk), .rst(rst), .done(done),
        .is_last_layer(1'b1), .act_sel(1'b0), .out_sel(1'b0), .skip_x(1'b0),
        .ext_x_addr_o(x_addr), .ext_x_rd_en_o(x_en), .ext_x_data_i(x_data),
        .ext_y_addr_o(y_addr), .ext_y_rd_en_o(y_en), .ext_y_data_i(y_data),
        .ext_z_addr_o(z_addr), .ext_z_wr_en_o(z_en), .ext_z_data_o(z_data), .ext_z_wmask_o(z_mask),
        .ext_b_addr_o(b_addr), .ext_b_rd_en_o(b_en), .ext_b_data_i(b_data),
        .sram_a_csb0(sa_csb0), .sram_a_web0(sa_web0), .sram_a_wmask0(sa_wm),
        .sram_a_addr0(sa_a0),  .sram_a_din0(sa_di),   .sram_a_dout0(sa_do0),
        .sram_a_csb1(sa_csb1), .sram_a_addr1(sa_a1),  .sram_a_dout1(sa_do1),
        .sram_b_csb0(sb_csb0), .sram_b_web0(sb_web0), .sram_b_wmask0(sb_wm),
        .sram_b_addr0(sb_a0),  .sram_b_din0(sb_di),   .sram_b_dout0(sb_do0),
        .sram_b_csb1(sb_csb1), .sram_b_addr1(sb_a1),  .sram_b_dout1(sb_do1),
        .sram_c_csb0(sc_csb0), .sram_c_web0(sc_web0), .sram_c_wmask0(sc_wm),
        .sram_c_addr0(sc_a0),  .sram_c_din0(sc_di),   .sram_c_dout0(sc_do0),
        .sram_c_csb1(sc_csb1), .sram_c_addr1(sc_a1),  .sram_c_dout1(sc_do1));

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

    // Gold model
    function automatic logic signed [31:0] conv_gold(
        input logic signed [7:0] img  [],
        input logic signed [7:0] kern [],
        input int op, input int f
    );
        logic signed [31:0] acc;
        int oh, ow, kh, kw, sr, sc;
        acc = 32'sd0;
        oh = op / OUT_W;
        ow = op % OUT_W;
        for (int t = 0; t < REAL_M; t++) begin
            kh = t / K_W;
            kw = t % K_W;
            sr = oh * STRIDE + kh - PAD;
            sc = ow * STRIDE + kw - PAD;
            if (sr >= 0 && sr < IN_H && sc >= 0 && sc < IN_W)
                acc += $signed(32'(img[sr * IN_W + sc]))
                     * $signed(32'(kern[t * REAL_N + f]));
        end
        return acc;
    endfunction

    // Print a grid of signed bytes
    task automatic print_grid(input string title,
                               input int rows, input int cols,
                               input logic signed [7:0] data []);
        $display("  %s", title);
        $write("          ");
        for (int c=0; c<cols; c++) $write(" col%0d  ",c);
        $display("");
        $write("          ");
        for (int c=0; c<cols; c++) $write("+------");
        $display("+");
        for (int r=0; r<rows; r++) begin
            $write("  row%0d   ", r);
            for (int c=0; c<cols; c++)
                $write("| %4d ", data[r*cols+c]);
            $display("|");
            $write("          ");
            for (int c=0; c<cols; c++) $write("+------");
            $display("+");
        end
    endtask

    // =========================================================================
    // Main
    // =========================================================================
    initial begin
        rst = 1; clear_z = 0;

        // -----------------------------------------------------------------
        // Zero DRAM
        // -----------------------------------------------------------------
        for (int i=0; i<1024; i++) begin dram_x[i]=32'd0; dram_y[i]=32'd0; dram_b[i]=32'd0; end
        @(posedge clk); #1; clear_z=1; @(posedge clk); #1; clear_z=0;

        // -----------------------------------------------------------------
        // Load image: pixel[r][c] = r*IN_W + c + 1  (values 1..36)
        // -----------------------------------------------------------------
        for (int r=0; r<IN_H; r++)
            for (int c=0; c<IN_W; c++)
                set_x_byte(r*IN_W+c, 8'(r*IN_W+c+1));

        // -----------------------------------------------------------------
        // Load kernels: 6 filters, each a scaled Sobel-X
        //   kern_f[kh][kw] = scale_f * sobelX[kh][kw]
        //   scales: f0=1, f1=2, f2=3, f3=4, f4=5, f5=6
        //   sobelX = [1,0,-1 / 2,0,-2 / 1,0,-1]
        //   expected output for each filter = scale * (-8)
        //     f0?-8, f1?-16, f2?-24, f3?-32, f4?-40, f5?-48
        // -----------------------------------------------------------------
        begin
            logic signed [7:0] sobelX [3][3];
            sobelX[0] = '{8'sd1, 8'sd0, -8'sd1};
            sobelX[1] = '{8'sd2, 8'sd0, -8'sd2};
            sobelX[2] = '{8'sd1, 8'sd0, -8'sd1};
            for (int kh=0; kh<K_H; kh++) begin
                for (int kw=0; kw<K_W; kw++) begin
                    int tap;
                    tap = kh*K_W + kw;
                    for (int f=0; f<REAL_N; f++)
                        set_y_byte(tap*REAL_N + f, 8'($signed(sobelX[kh][kw]) * (f+1)));
                end
            end
        end

        // Bias = 0
        for (int i=0; i<REAL_K; i++) dram_b[i] = 32'd0;

        // -----------------------------------------------------------------
        // Print header
        // -----------------------------------------------------------------
        $display("");
        $display("================================================================");
        $display("TEST D  ?  tiling test");
        $display("================================================================");
        $display("");
        $display("  6x6x1 grayscale image  |  3x3 kernel  |  6 filters  |  stride=1  pad=0");
        $display("  OUT_H=%0d  OUT_W=%0d  REAL_K=%0d  REAL_N=%0d  REAL_M=%0d",
                 OUT_H, OUT_W, REAL_K, REAL_N, REAL_M);
        $display("  REAL_K=%0d > TILE_MAX=%0d  ?  K tiles into %0d passes",
                 REAL_K, TILE_MAX, (REAL_K+TILE_MAX-1)/TILE_MAX);
        $display("  REAL_N=%0d > TILE_MAX=%0d  ?  N tiles into %0d passes",
                 REAL_N, TILE_MAX, (REAL_N+TILE_MAX-1)/TILE_MAX);
        $display("  Total tile passes = %0d",
                 ((REAL_K+TILE_MAX-1)/TILE_MAX) * ((REAL_N+TILE_MAX-1)/TILE_MAX));
        $display("");

        // Print image
        begin
            logic signed [7:0] img_data [IN_H*IN_W];
            for (int r=0;r<IN_H;r++) for (int c=0;c<IN_W;c++)
                img_data[r*IN_W+c] = 8'(r*IN_W+c+1);
            print_grid("Input image  (6x6, pixel[r][c] = r*6+c+1)", IN_H, IN_W, img_data);
        end
        $display("");

        // Print kernel (sobelX, before scaling)
        $display("  Kernel (Sobel-X base, scaled per filter):");
        $display("          kw=0   kw=1   kw=2");
        $display("          +------+------+------+");
        $display("  kh=0   |    1 |    0 |   -1 |");
        $display("          +------+------+------+");
        $display("  kh=1   |    2 |    0 |   -2 |");
        $display("          +------+------+------+");
        $display("  kh=2   |    1 |    0 |   -1 |");
        $display("          +------+------+------+");
        $display("  f0=1譻obelX  f1=2譻obelX  f2=3譻obelX");
        $display("  f3=4譻obelX  f4=5譻obelX  f5=6譻obelX");
        $display("  Expected output: f0=-8  f1=-16  f2=-24  f3=-32  f4=-40  f5=-48");
        $display("  (uniform across all 16 output positions ? image is a linear ramp)");
        $display("");

        // -----------------------------------------------------------------
        // Run
        // -----------------------------------------------------------------
        @(posedge clk); #1; rst = 0;
        wait(done === 1'b1);
        @(posedge clk); #1; @(posedge clk); #1;

        // -----------------------------------------------------------------
        // Verify and print results
        // -----------------------------------------------------------------
        begin
            logic signed [7:0] img_flat [IN_H*IN_W];
            logic signed [7:0] kern_flat [REAL_M*REAL_N];
            logic signed [7:0] sobelX [3][3];
            logic signed [7:0] out_grid [OUT_H*OUT_W];
            int pass_c, fail_c;
            pass_c=0; fail_c=0;

            sobelX[0] = '{8'sd1, 8'sd0, -8'sd1};
            sobelX[1] = '{8'sd2, 8'sd0, -8'sd2};
            sobelX[2] = '{8'sd1, 8'sd0, -8'sd1};

            for (int r=0;r<IN_H;r++) for (int c=0;c<IN_W;c++)
                img_flat[r*IN_W+c] = 8'(r*IN_W+c+1);

            for (int kh=0;kh<K_H;kh++) for (int kw=0;kw<K_W;kw++) begin
                int tap;
                tap=kh*K_W+kw;
                for (int f=0;f<REAL_N;f++)
                    kern_flat[tap*REAL_N+f] = 8'($signed(sobelX[kh][kw])*(f+1));
            end

            // Per-filter output grids
            for (int f=0; f<REAL_N; f++) begin
                logic signed [7:0] gold_v, got_v;
                int f_pass, f_fail;
                f_pass=0; f_fail=0;

                for (int op=0; op<REAL_K; op++) begin
                    gold_v = clip8(conv_gold(img_flat, kern_flat, op, f));
                    got_v  = z_byte(op*REAL_N + f);
                    out_grid[op] = got_v;
                    if (got_v === gold_v) begin pass_c++; f_pass++; end
                    else                 begin fail_c++; f_fail++; end
                end

                begin
                    string title;
                    $sformat(title, "Filter %0d  (scale=%0d, expected %0d everywhere)  ?  %s",
                             f, f+1, -(f+1)*8,
                             (f_fail==0) ? "ALL PASS" : "FAIL");
                    print_grid(title, OUT_H, OUT_W, out_grid);
                end
                $display("");
            end

            // Summary
            $display("================================================================");
            if (fail_c==0)
                $display("TEST D: ALL %0d OUTPUTS CORRECT  (%0d positions � %0d filters)",
                         pass_c, REAL_K, REAL_N);
            else
                $display("TEST D: %0d PASS  %0d FAIL", pass_c, fail_c);
            $display("================================================================");
        end

        #200;
        $finish;
    end

    initial begin
        #50000000;
        $display("WATCHDOG: timeout");
        $finish;
    end

endmodule
