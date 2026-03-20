`timescale 1ns / 1ps

// =============================================================================
// MAC
// =============================================================================
module mac (
    output logic signed [31:0] south, east,
    output logic signed [63:0] result,
    input  logic signed [31:0] north, west,
    input  logic clk, rst
);
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 64'd0;
            south  <= 32'd0;
            east   <= 32'd0;
        end else begin
            result <= result + (north * west);
            south  <= north;
            east   <= west;
        end
    end
endmodule

// =============================================================================
// Systolic tile
// =============================================================================
module systolic_tiled #(parameter TILE = 4) (
    output logic signed [63:0] c[TILE][TILE],
    input  logic signed [31:0] a[TILE],
    input  logic signed [31:0] b[TILE],
    input  logic clk, rst
);
    logic signed [31:0] south[TILE][TILE], east[TILE][TILE];
    genvar i, j;
    generate
        for (i = 0; i < TILE; i++) begin: ROW
            for (j = 0; j < TILE; j++) begin: COL
                mac PE (
                    .clk   (clk), .rst(rst),
                    .north ((i == 0) ? a[j]      : south[i-1][j]),
                    .west  ((j == 0) ? b[i]      : east[i][j-1]),
                    .east  (east[i][j]), .south(south[i][j]),
                    .result(c[i][j])
                );
            end
        end
    endgenerate
endmodule

// =============================================================================
// Padding engine
// =============================================================================
module padding_engine #(
    parameter TILE   = 4,
    parameter REAL_K = 4,
    parameter REAL_M = 9,
    parameter REAL_N = 1,
    parameter PAD_K  = ((REAL_K + TILE - 1) / TILE) * TILE,
    parameter PAD_M  = ((REAL_M + TILE - 1) / TILE) * TILE,
    parameter PAD_N  = ((REAL_N + TILE - 1) / TILE) * TILE
)(
    input  logic signed [31:0] raw_x [REAL_K][REAL_M],
    input  logic signed [31:0] raw_y [REAL_M][REAL_N],
    output logic signed [31:0] x_mem [PAD_K][PAD_M],
    output logic signed [31:0] y_mem [PAD_M][PAD_N],
    output logic pad_done,
    input  logic clk, rst
);
    integer ri, rk, rj;
    typedef enum logic [1:0] {FILL_X, FILL_Y, DONE} pad_state_t;
    pad_state_t state;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state    <= FILL_X;
            pad_done <= 0;
            ri <= 0; rk <= 0; rj <= 0;
            for (int ii = 0; ii < PAD_K; ii++)
                for (int kk = 0; kk < PAD_M; kk++)
                    x_mem[ii][kk] <= 32'd0;
            for (int kk = 0; kk < PAD_M; kk++)
                for (int jj = 0; jj < PAD_N; jj++)
                    y_mem[kk][jj] <= 32'd0;
        end else begin
            case (state)
                FILL_X: begin
                    x_mem[ri][rk] <= (ri < REAL_K && rk < REAL_M)
                                     ? raw_x[ri][rk] : 32'd0;
                    if (rk < PAD_M - 1) rk <= rk + 1;
                    else begin
                        rk <= 0;
                        if (ri < PAD_K - 1) ri <= ri + 1;
                        else begin ri <= 0; rk <= 0; state <= FILL_Y; end
                    end
                end
                FILL_Y: begin
                    y_mem[ri][rj] <= (ri < REAL_M && rj < REAL_N)
                                     ? raw_y[ri][rj] : 32'd0;
                    if (rj < PAD_N - 1) rj <= rj + 1;
                    else begin
                        rj <= 0;
                        if (ri < PAD_M - 1) ri <= ri + 1;
                        else begin ri <= 0; rj <= 0; state <= DONE; end
                    end
                end
                DONE:    pad_done <= 1;
                default: state <= FILL_X;
            endcase
        end
    end
endmodule

// =============================================================================
// Testbench
// =============================================================================
`timescale 1ns / 1ps

module tb_all_examples;

    localparam TILE   = 4;
    localparam REAL_K = 4;
    localparam REAL_M = 9;
    localparam REAL_N = 1;
    localparam PAD_K  = ((REAL_K + TILE - 1) / TILE) * TILE;
    localparam PAD_M  = ((REAL_M + TILE - 1) / TILE) * TILE;
    localparam PAD_N  = ((REAL_N + TILE - 1) / TILE) * TILE;

    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    logic signed [31:0] col_matrix [REAL_K][REAL_M];
    logic signed [31:0] kern_flat  [REAL_M][REAL_N];
    logic signed [31:0] x_mem      [PAD_K][PAD_M];
    logic signed [31:0] y_mem      [PAD_M][PAD_N];
    logic pad_done;

    padding_engine #(
        .TILE(TILE), .REAL_K(REAL_K), .REAL_M(REAL_M), .REAL_N(REAL_N)
    ) pe (
        .raw_x(col_matrix), .raw_y(kern_flat),
        .x_mem(x_mem), .y_mem(y_mem),
        .pad_done(pad_done), .clk(clk), .rst(rst)
    );

    logic signed [31:0] a_feed [TILE];
    logic signed [31:0] b_feed [TILE];
    logic signed [63:0] c_out  [TILE][TILE];
    logic sa_rst = 1;

    systolic_tiled #(.TILE(TILE)) sa (
        .clk(clk), .rst(sa_rst),
        .a(a_feed), .b(b_feed), .c(c_out)
    );

    int feed_step;
    int pass_count, fail_count;
    logic signed [31:0] img [4][4];

    // -----------------------------------------------------------------------
    // Task: reset_dut  ? called AFTER build_col_matrix + load_kernel
    // -----------------------------------------------------------------------
    task automatic reset_dut();
        int i;
        rst    = 1;
        sa_rst = 1;
        for (i = 0; i < TILE; i++) begin
            a_feed[i] = '0;
            b_feed[i] = '0;
        end
        repeat(30) @(posedge clk); #1;
        rst = 0;
        repeat(20) @(posedge clk); #1;
        sa_rst = 0;
        repeat(4) @(posedge clk); #1;
    endtask

    // -----------------------------------------------------------------------
    // Task: wait_pad_done
    // -----------------------------------------------------------------------
    task automatic wait_pad_done();
        if (pad_done === 1'b1) begin
            @(negedge pad_done);
            @(posedge clk); #1;
        end
        wait(pad_done === 1'b1);
        @(posedge clk); #1;
    endtask

    // -----------------------------------------------------------------------
    // Task: build_col_matrix
    // -----------------------------------------------------------------------
    task automatic build_col_matrix(
        input int stride,
        input int pad
    );
        logic signed [31:0] padded [8][8];
        int img_h, img_w, out_h, out_w;
        int pr, pc, kr, kc, idx;

        img_h = 4; img_w = 4;

        for (pr=0; pr<8; pr++) for (pc=0; pc<8; pc++)
            padded[pr][pc] = 32'd0;

        for (pr=0; pr<img_h; pr++) for (pc=0; pc<img_w; pc++)
            padded[pr+pad][pc+pad] = img[pr][pc];

        out_h = (img_h + 2*pad - 3) / stride + 1;
        out_w = (img_w + 2*pad - 3) / stride + 1;

        idx = 0;
        for (pr=0; pr<out_h; pr++) begin
            for (pc=0; pc<out_w; pc++) begin
                for (kr=0; kr<3; kr++) for (kc=0; kc<3; kc++)
                    col_matrix[idx][kr*3+kc] =
                        padded[pr*stride+kr][pc*stride+kc];
                idx++;
            end
        end

        for (pr=idx; pr<REAL_K; pr++)
            for (pc=0; pc<REAL_M; pc++)
                col_matrix[pr][pc] = 32'd0;
    endtask

    // -----------------------------------------------------------------------
    // Task: load_kernel
    // -----------------------------------------------------------------------
    task automatic load_kernel(
        input logic signed [31:0] k00, k01, k02,
        input logic signed [31:0] k10, k11, k12,
        input logic signed [31:0] k20, k21, k22
    );
        kern_flat[0][0]=k00; kern_flat[1][0]=k01; kern_flat[2][0]=k02;
        kern_flat[3][0]=k10; kern_flat[4][0]=k11; kern_flat[5][0]=k12;
        kern_flat[6][0]=k20; kern_flat[7][0]=k21; kern_flat[8][0]=k22;
        kern_flat[9][0]=32'd0;
        kern_flat[10][0]=32'd0;
        kern_flat[11][0]=32'd0;
    endtask

    // -----------------------------------------------------------------------
    // Task: stream_to_systolic
    // a_feed[0] = y_mem[step][0]        (kernel, no skew, col 0 only)
    // b_feed[i] = x_mem[i][step - i]    (image row i, skewed by row index)
    // -----------------------------------------------------------------------
    task automatic stream_to_systolic();
        int b_idx;

        for (feed_step = 0; feed_step < PAD_M + TILE - 1; feed_step++) begin
            if (feed_step < PAD_M)
                a_feed[0] = y_mem[feed_step][0];
            else
                a_feed[0] = '0;
            for (int j = 1; j < TILE; j++)
                a_feed[j] = '0;

            for (int i = 0; i < TILE; i++) begin
                b_idx = feed_step - i;
                if (b_idx >= 0 && b_idx < PAD_M)
                    b_feed[i] = x_mem[i][b_idx];
                else
                    b_feed[i] = '0;
            end
            @(posedge clk); #1;
        end

        // Drain pipeline
        for (int d = 0; d < TILE + 2; d++) begin
            for (int i = 0; i < TILE; i++) begin
                a_feed[i] = '0;
                b_feed[i] = '0;
            end
            @(posedge clk); #1;
        end
    endtask

    // -----------------------------------------------------------------------
    // Task: check_result
    // -----------------------------------------------------------------------
    task automatic check_result(
        input string              label,
        input logic signed [63:0] got,
        input logic signed [63:0] expected
    );
        if (got === expected) begin
            $display("  PASS  %s : got=%0d", label, got);
            pass_count++;
        end else begin
            $display("  FAIL  %s : got=%0d  expected=%0d",
                     label, got, expected);
            fail_count++;
        end
    endtask

    task automatic print_example_header(input string title);
        $display("\n========================================");
        $display(" %s", title);
        $display("========================================");
    endtask

    // -----------------------------------------------------------------------
    // Manual dot-product verification for sharpen kernel, stride=1, pad=0
    // Image patches (col_matrix rows):
    //   row 0: [1,2,3,5,6,7,9,10,11]
    //   row 1: [2,3,4,6,7,8,10,11,12]
    //   row 2: [5,6,7,9,10,11,13,14,15]
    //   row 3: [6,7,8,10,11,12,14,15,16]
    // Sharpen kernel flat: [0,-1,0,-1,5,-1,0,-1,0]
    //   row 0: 0*1 +(-1)*2 +0*3 +(-1)*5 +5*6 +(-1)*7 +0*9 +(-1)*10+0*11
    //        = 0 - 2 + 0 - 5 + 30 - 7 + 0 - 10 + 0 = 6
    //   row 1: 0*2 +(-1)*3 +0*4 +(-1)*6 +5*7 +(-1)*8 +0*10+(-1)*11+0*12
    //        = 0 - 3 + 0 - 6 + 35 - 8 + 0 - 11 + 0 = 7
    //   row 2: 0*5 +(-1)*6 +0*7 +(-1)*9 +5*10+(-1)*11+0*13+(-1)*14+0*15
    //        = 0 - 6 + 0 - 9 + 50 - 11 + 0 - 14 + 0 = 10
    //   row 3: 0*6 +(-1)*7 +0*8 +(-1)*10+5*11+(-1)*12+0*14+(-1)*15+0*16
    //        = 0 - 7 + 0 - 10 + 55 - 12 + 0 - 15 + 0 = 11
    // So correct expected values are 6, 7, 10, 11 ? NOT all 6.
    // The image is a ramp (1..16), not uniform, so the sharpen filter
    // produces different results at each output position.
    // -----------------------------------------------------------------------

    initial begin
        pass_count = 0;
        fail_count = 0;

        // Cold start
        rst    = 1;
        sa_rst = 1;
        for (int i = 0; i < TILE; i++) begin
            a_feed[i] = '0;
            b_feed[i] = '0;
        end
        repeat(10) @(posedge clk); #1;
        rst    = 0;
        repeat(5)  @(posedge clk); #1;
        sa_rst = 0;
        repeat(5)  @(posedge clk); #1;

        // Shared image
        img[0][0]=32'd1;  img[0][1]=32'd2;  img[0][2]=32'd3;  img[0][3]=32'd4;
        img[1][0]=32'd5;  img[1][1]=32'd6;  img[1][2]=32'd7;  img[1][3]=32'd8;
        img[2][0]=32'd9;  img[2][1]=32'd10; img[2][2]=32'd11; img[2][3]=32'd12;
        img[3][0]=32'd13; img[3][1]=32'd14; img[3][2]=32'd15; img[3][3]=32'd16;

        // ==================================================================
        // Example 1 ? Sobel-X   expected: all -8
        // ==================================================================
        print_example_header("Example 1 - Sobel-X (horizontal edges)");
        build_col_matrix(1, 0);
        load_kernel(
             32'd1,  32'd0, -32'd1,
             32'd2,  32'd0, -32'd2,
             32'd1,  32'd0, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd8);
        check_result("out[0,1]", c_out[1][0], -64'd8);
        check_result("out[1,0]", c_out[2][0], -64'd8);
        check_result("out[1,1]", c_out[3][0], -64'd8);

        // ==================================================================
        // Example 2 ? Sobel-Y   expected: all -32
        // ==================================================================
        print_example_header("Example 2 - Sobel-Y (vertical edges)");
        build_col_matrix(1, 0);
        load_kernel(
             32'd1,  32'd2,  32'd1,
             32'd0,  32'd0,  32'd0,
            -32'd1, -32'd2, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd32);
        check_result("out[0,1]", c_out[1][0], -64'd32);
        check_result("out[1,0]", c_out[2][0], -64'd32);
        check_result("out[1,1]", c_out[3][0], -64'd32);

        // ==================================================================
        // Example 3 ? Laplacian   expected: all 0
        // ==================================================================
        print_example_header("Example 3 - Laplacian (all-direction edges)");
        build_col_matrix(1, 0);
        load_kernel(
            32'd0,  32'd1,  32'd0,
            32'd1, -32'd4,  32'd1,
            32'd0,  32'd1,  32'd0
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], 64'd0);
        check_result("out[0,1]", c_out[1][0], 64'd0);
        check_result("out[1,0]", c_out[2][0], 64'd0);
        check_result("out[1,1]", c_out[3][0], 64'd0);

        // ==================================================================
        // Example 4 ? Box blur   expected: 54, 63, 90, 99
        // ==================================================================
        print_example_header("Example 4 - Box blur (3x3 sum)");
        build_col_matrix(1, 0);
        load_kernel(
            32'd1, 32'd1, 32'd1,
            32'd1, 32'd1, 32'd1,
            32'd1, 32'd1, 32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], 64'd54);
        check_result("out[0,1]", c_out[1][0], 64'd63);
        check_result("out[1,0]", c_out[2][0], 64'd90);
        check_result("out[1,1]", c_out[3][0], 64'd99);

        // ==================================================================
        // Example 5 ? Sharpen
        // FIX: expected values corrected by manual dot-product calculation.
        // The image is a 1..16 ramp so each output pixel is different.
        //   pixel 0 (top-left):     6
        //   pixel 1 (top-right):    7
        //   pixel 2 (bottom-left): 10
        //   pixel 3 (bottom-right):11
        // ==================================================================
        print_example_header("Example 5 - Sharpen filter");
        build_col_matrix(1, 0);
        load_kernel(
             32'd0, -32'd1,  32'd0,
            -32'd1,  32'd5, -32'd1,
             32'd0, -32'd1,  32'd0
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], 64'd6);
        check_result("out[0,1]", c_out[1][0], 64'd7);   // was wrongly 6
        check_result("out[1,0]", c_out[2][0], 64'd10);  // was wrongly 6
        check_result("out[1,1]", c_out[3][0], 64'd11);  // was wrongly 6

        // ==================================================================
        // Example 6 ? Same padding (pad=1, Sobel-X)
        // expected: -10, -6, -6, 13
        // ==================================================================
        print_example_header("Example 6 - Same padding (pad=1, Sobel-X)");
        build_col_matrix(1, 1);
        load_kernel(
             32'd1,  32'd0, -32'd1,
             32'd2,  32'd0, -32'd2,
             32'd1,  32'd0, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd10);
        check_result("out[0,1]", c_out[1][0], -64'd6);
        check_result("out[0,2]", c_out[2][0], -64'd6);
        check_result("out[0,3]", c_out[3][0],  64'd13);

        // ==================================================================
        // Final summary
        // ==================================================================
        $display("\n========================================");
        $display(" FINAL RESULTS: %0d PASSED, %0d FAILED",
                 pass_count, fail_count);
        $display("========================================\n");
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED - check waveform");

        $finish;
    end

    initial begin
        $dumpfile("tb_all.vcd");
        $dumpvars(0, tb_all_examples);
    end

    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
