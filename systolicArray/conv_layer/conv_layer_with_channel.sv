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
// PAD_M = ceil(REAL_M / TILE) * TILE
// With C_in=3: REAL_M = K_h * K_w * C_in = 3*3*3 = 27
//              PAD_M  = ceil(27/4)*4 = 28
// =============================================================================
module padding_engine #(
    parameter TILE   = 4,
    parameter REAL_K = 4,
    parameter REAL_M = 27,   // K_h*K_w*C_in = 3*3*3
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

module tb_multichannel;

    // -----------------------------------------------------------------------
    // Parameters
    // C_IN  = 3  (RGB)
    // K_H   = K_W = 3
    // REAL_M = K_H * K_W * C_IN = 27
    // REAL_K = number of output pixels = out_h * out_w
    //          for 4x4 image, stride=1, pad=0: out_h=out_w=2 -> REAL_K=4
    // -----------------------------------------------------------------------
    localparam TILE   = 4;
    localparam C_IN   = 3;
    localparam K_H    = 3;
    localparam K_W    = 3;
    localparam REAL_K = 4;                    // output pixels per example
    localparam REAL_M = K_H * K_W * C_IN;    // 27
    localparam REAL_N = 1;
    localparam PAD_K  = ((REAL_K + TILE - 1) / TILE) * TILE;  // 4
    localparam PAD_M  = ((REAL_M + TILE - 1) / TILE) * TILE;  // 28
    localparam PAD_N  = ((REAL_N + TILE - 1) / TILE) * TILE;  // 4

    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    // col_matrix: one row per output pixel, columns = flattened patch+channels
    // Layout per row: [p[0,0,R], p[0,0,G], p[0,0,B], p[0,1,R], ... p[2,2,B]]
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

    // RGB image arrays ? separate channels for readability
    logic signed [31:0] img_R [4][4];
    logic signed [31:0] img_G [4][4];
    logic signed [31:0] img_B [4][4];

    // -----------------------------------------------------------------------
    // Task: reset_dut ? called AFTER build_col_matrix + load_kernel
    // -----------------------------------------------------------------------
    task automatic reset_dut();
        rst    = 1;
        sa_rst = 1;
        for (int i = 0; i < TILE; i++) begin
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
    // Task: build_col_matrix_rgb
    // Builds im2col matrix with interleaved channels.
    // For each output pixel, the patch is flattened as:
    //   [p[kr,kc,R], p[kr,kc,G], p[kr,kc,B]] for kr=0..2, kc=0..2
    // This matches the kernel_flat layout so the dot product is correct.
    // -----------------------------------------------------------------------
    task automatic build_col_matrix_rgb(
        input int stride,
        input int pad
    );
        logic signed [31:0] padR [8][8], padG [8][8], padB [8][8];
        int img_h, img_w, out_h, out_w;
        int pr, pc, kr, kc, idx, col_idx;

        img_h = 4; img_w = 4;

        // Zero-pad all channels
        for (pr=0; pr<8; pr++) for (pc=0; pc<8; pc++) begin
            padR[pr][pc] = 32'd0;
            padG[pr][pc] = 32'd0;
            padB[pr][pc] = 32'd0;
        end
        for (pr=0; pr<img_h; pr++) for (pc=0; pc<img_w; pc++) begin
            padR[pr+pad][pc+pad] = img_R[pr][pc];
            padG[pr+pad][pc+pad] = img_G[pr][pc];
            padB[pr+pad][pc+pad] = img_B[pr][pc];
        end

        out_h = (img_h + 2*pad - K_H) / stride + 1;
        out_w = (img_w + 2*pad - K_W) / stride + 1;

        idx = 0;
        for (pr=0; pr<out_h; pr++) begin
            for (pc=0; pc<out_w; pc++) begin
                col_idx = 0;
                // Interleave channels at each kernel position
                for (kr=0; kr<K_H; kr++) begin
                    for (kc=0; kc<K_W; kc++) begin
                        col_matrix[idx][col_idx]   = padR[pr*stride+kr][pc*stride+kc];
                        col_matrix[idx][col_idx+1] = padG[pr*stride+kr][pc*stride+kc];
                        col_matrix[idx][col_idx+2] = padB[pr*stride+kr][pc*stride+kc];
                        col_idx += C_IN;
                    end
                end
                idx++;
            end
        end

        // Zero-fill unused rows
        for (pr=idx; pr<REAL_K; pr++)
            for (pc=0; pc<REAL_M; pc++)
                col_matrix[pr][pc] = 32'd0;
    endtask

    // -----------------------------------------------------------------------
    // Task: load_kernel_rgb
    // Loads a 3x3 kernel with per-channel weights.
    // Flat layout: [k[0,0,R], k[0,0,G], k[0,0,B], k[0,1,R], ... k[2,2,B]]
    // kXY_R/G/B are the kernel weights for row X, col Y, channel R/G/B.
    // -----------------------------------------------------------------------
    task automatic load_kernel_rgb(
        // Row 0
        input logic signed [31:0] k00R, k00G, k00B,
        input logic signed [31:0] k01R, k01G, k01B,
        input logic signed [31:0] k02R, k02G, k02B,
        // Row 1
        input logic signed [31:0] k10R, k10G, k10B,
        input logic signed [31:0] k11R, k11G, k11B,
        input logic signed [31:0] k12R, k12G, k12B,
        // Row 2
        input logic signed [31:0] k20R, k20G, k20B,
        input logic signed [31:0] k21R, k21G, k21B,
        input logic signed [31:0] k22R, k22G, k22B
    );
        // Row 0
        kern_flat[0][0]=k00R; kern_flat[1][0]=k00G; kern_flat[2][0]=k00B;
        kern_flat[3][0]=k01R; kern_flat[4][0]=k01G; kern_flat[5][0]=k01B;
        kern_flat[6][0]=k02R; kern_flat[7][0]=k02G; kern_flat[8][0]=k02B;
        // Row 1
        kern_flat[9][0] =k10R; kern_flat[10][0]=k10G; kern_flat[11][0]=k10B;
        kern_flat[12][0]=k11R; kern_flat[13][0]=k11G; kern_flat[14][0]=k11B;
        kern_flat[15][0]=k12R; kern_flat[16][0]=k12G; kern_flat[17][0]=k12B;
        // Row 2
        kern_flat[18][0]=k20R; kern_flat[19][0]=k20G; kern_flat[20][0]=k20B;
        kern_flat[21][0]=k21R; kern_flat[22][0]=k21G; kern_flat[23][0]=k21B;
        kern_flat[24][0]=k22R; kern_flat[25][0]=k22G; kern_flat[26][0]=k22B;
        // Padding to PAD_M=28
        kern_flat[27][0] = 32'd0;
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
    // Main sequence
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

        // ---------------------------------------------------------------
        // RGB image: R=1..16, G=17..32, B=33..48
        // ---------------------------------------------------------------
        // R channel
        img_R[0][0]=32'd1;  img_R[0][1]=32'd2;  img_R[0][2]=32'd3;  img_R[0][3]=32'd4;
        img_R[1][0]=32'd5;  img_R[1][1]=32'd6;  img_R[1][2]=32'd7;  img_R[1][3]=32'd8;
        img_R[2][0]=32'd9;  img_R[2][1]=32'd10; img_R[2][2]=32'd11; img_R[2][3]=32'd12;
        img_R[3][0]=32'd13; img_R[3][1]=32'd14; img_R[3][2]=32'd15; img_R[3][3]=32'd16;
        // G channel
        img_G[0][0]=32'd17; img_G[0][1]=32'd18; img_G[0][2]=32'd19; img_G[0][3]=32'd20;
        img_G[1][0]=32'd21; img_G[1][1]=32'd22; img_G[1][2]=32'd23; img_G[1][3]=32'd24;
        img_G[2][0]=32'd25; img_G[2][1]=32'd26; img_G[2][2]=32'd27; img_G[2][3]=32'd28;
        img_G[3][0]=32'd29; img_G[3][1]=32'd30; img_G[3][2]=32'd31; img_G[3][3]=32'd32;
        // B channel
        img_B[0][0]=32'd33; img_B[0][1]=32'd34; img_B[0][2]=32'd35; img_B[0][3]=32'd36;
        img_B[1][0]=32'd37; img_B[1][1]=32'd38; img_B[1][2]=32'd39; img_B[1][3]=32'd40;
        img_B[2][0]=32'd41; img_B[2][1]=32'd42; img_B[2][2]=32'd43; img_B[2][3]=32'd44;
        img_B[3][0]=32'd45; img_B[3][1]=32'd46; img_B[3][2]=32'd47; img_B[3][3]=32'd48;

        // ==================================================================
        // Example 1 ? Sobel-X RGB  expected: all -24
        // Each channel contributes -8, total = -8*3 = -24
        // ==================================================================
        print_example_header("Example 1 - Sobel-X RGB (all channels equal)");
        build_col_matrix_rgb(1, 0);
        load_kernel_rgb(
            // Row 0: [1,0,-1] same for R,G,B
             32'd1,  32'd1,  32'd1,   32'd0, 32'd0, 32'd0,  -32'd1, -32'd1, -32'd1,
            // Row 1: [2,0,-2]
             32'd2,  32'd2,  32'd2,   32'd0, 32'd0, 32'd0,  -32'd2, -32'd2, -32'd2,
            // Row 2: [1,0,-1]
             32'd1,  32'd1,  32'd1,   32'd0, 32'd0, 32'd0,  -32'd1, -32'd1, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd24);
        check_result("out[0,1]", c_out[1][0], -64'd24);
        check_result("out[1,0]", c_out[2][0], -64'd24);
        check_result("out[1,1]", c_out[3][0], -64'd24);

        // ==================================================================
        // Example 2 ? Sobel-Y RGB  expected: all -96
        // Each channel contributes -32, total = -32*3 = -96
        // ==================================================================
        print_example_header("Example 2 - Sobel-Y RGB (all channels equal)");
        build_col_matrix_rgb(1, 0);
        load_kernel_rgb(
            // Row 0: [1,2,1]
             32'd1,  32'd1,  32'd1,   32'd2, 32'd2, 32'd2,   32'd1,  32'd1,  32'd1,
            // Row 1: [0,0,0]
             32'd0,  32'd0,  32'd0,   32'd0, 32'd0, 32'd0,   32'd0,  32'd0,  32'd0,
            // Row 2: [-1,-2,-1]
            -32'd1, -32'd1, -32'd1,  -32'd2,-32'd2,-32'd2,  -32'd1, -32'd1, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd96);
        check_result("out[0,1]", c_out[1][0], -64'd96);
        check_result("out[1,0]", c_out[2][0], -64'd96);
        check_result("out[1,1]", c_out[3][0], -64'd96);

        // ==================================================================
        // Example 3 ? Laplacian RGB  expected: all 0
        // Each channel is a uniform ramp -> Laplacian = 0 per channel
        // ==================================================================
        print_example_header("Example 3 - Laplacian RGB");
        build_col_matrix_rgb(1, 0);
        load_kernel_rgb(
            // Row 0: [0,1,0]
             32'd0,  32'd0,  32'd0,   32'd1, 32'd1, 32'd1,   32'd0,  32'd0,  32'd0,
            // Row 1: [1,-4,1]
             32'd1,  32'd1,  32'd1,  -32'd4,-32'd4,-32'd4,   32'd1,  32'd1,  32'd1,
            // Row 2: [0,1,0]
             32'd0,  32'd0,  32'd0,   32'd1, 32'd1, 32'd1,   32'd0,  32'd0,  32'd0
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], 64'd0);
        check_result("out[0,1]", c_out[1][0], 64'd0);
        check_result("out[1,0]", c_out[2][0], 64'd0);
        check_result("out[1,1]", c_out[3][0], 64'd0);

        // ==================================================================
        // Example 4 ? Box blur RGB  expected: 594, 621, 702, 729
        // Python verified: patch sums across all 3 channels
        // ==================================================================
        print_example_header("Example 4 - Box blur RGB");
        build_col_matrix_rgb(1, 0);
        load_kernel_rgb(
            // All ones for all channels
             32'd1, 32'd1, 32'd1,   32'd1, 32'd1, 32'd1,   32'd1, 32'd1, 32'd1,
             32'd1, 32'd1, 32'd1,   32'd1, 32'd1, 32'd1,   32'd1, 32'd1, 32'd1,
             32'd1, 32'd1, 32'd1,   32'd1, 32'd1, 32'd1,   32'd1, 32'd1, 32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], 64'd594);
        check_result("out[0,1]", c_out[1][0], 64'd621);
        check_result("out[1,0]", c_out[2][0], 64'd702);
        check_result("out[1,1]", c_out[3][0], 64'd729);

        // ==================================================================
        // Example 5 ? Sharpen RGB  expected: 66, 69, 78, 81
        // Python verified manually per patch
        // ==================================================================
        print_example_header("Example 5 - Sharpen RGB");
        build_col_matrix_rgb(1, 0);
        load_kernel_rgb(
            // Row 0: [0,-1,0]
             32'd0,  32'd0,  32'd0,  -32'd1,-32'd1,-32'd1,   32'd0,  32'd0,  32'd0,
            // Row 1: [-1,5,-1]
            -32'd1, -32'd1, -32'd1,   32'd5, 32'd5, 32'd5,  -32'd1, -32'd1, -32'd1,
            // Row 2: [0,-1,0]
             32'd0,  32'd0,  32'd0,  -32'd1,-32'd1,-32'd1,   32'd0,  32'd0,  32'd0
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], 64'd66);
        check_result("out[0,1]", c_out[1][0], 64'd69);
        check_result("out[1,0]", c_out[2][0], 64'd78);
        check_result("out[1,1]", c_out[3][0], 64'd81);

        // ==================================================================
        // Example 6 ? Same padding (pad=1, Sobel-X, RGB)
        // expected top row: -174, -18, -18, 183  (Python verified)
        // ==================================================================
        print_example_header("Example 6 - Same padding RGB (pad=1, Sobel-X)");
        build_col_matrix_rgb(1, 1);
        load_kernel_rgb(
             32'd1,  32'd1,  32'd1,   32'd0, 32'd0, 32'd0,  -32'd1, -32'd1, -32'd1,
             32'd2,  32'd2,  32'd2,   32'd0, 32'd0, 32'd0,  -32'd2, -32'd2, -32'd2,
             32'd1,  32'd1,  32'd1,   32'd0, 32'd0, 32'd0,  -32'd1, -32'd1, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd174);
        check_result("out[0,1]", c_out[1][0], -64'd18);
        check_result("out[0,2]", c_out[2][0], -64'd18);
        check_result("out[0,3]", c_out[3][0],  64'd183);

        // ==================================================================
        // Example 7 ? Per-channel luminance-weighted Sobel-X
        // R*3, G*6, B*1  expected: all -80  (Python verified)
        // ==================================================================
        print_example_header("Example 7 - Luminance-weighted Sobel-X (R*3, G*6, B*1)");
        build_col_matrix_rgb(1, 0);
        load_kernel_rgb(
            // Row 0: R*3, G*6, B*1 per Sobel-X row
             32'd3,  32'd6,  32'd1,   32'd0, 32'd0, 32'd0,  -32'd3, -32'd6, -32'd1,
            // Row 1: [2,0,-2] weighted
             32'd6,  32'd12, 32'd2,   32'd0, 32'd0, 32'd0,  -32'd6, -32'd12,-32'd2,
            // Row 2: [1,0,-1] weighted
             32'd3,  32'd6,  32'd1,   32'd0, 32'd0, 32'd0,  -32'd3, -32'd6, -32'd1
        );
        reset_dut();
        wait_pad_done();
        stream_to_systolic();
        check_result("out[0,0]", c_out[0][0], -64'd80);
        check_result("out[0,1]", c_out[1][0], -64'd80);
        check_result("out[1,0]", c_out[2][0], -64'd80);
        check_result("out[1,1]", c_out[3][0], -64'd80);

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
        $dumpfile("tb_multichannel.vcd");
        $dumpvars(0, tb_multichannel);
    end

    initial begin
        #1000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule

