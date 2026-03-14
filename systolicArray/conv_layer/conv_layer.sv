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
// Systolic tile  (TILE x TILE MACs)
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
    parameter REAL_K = 5,
    parameter REAL_M = 6,
    parameter REAL_N = 7,
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
                    if (rk < PAD_M - 1) begin
                        rk <= rk + 1;
                    end else begin
                        rk <= 0;
                        if (ri < PAD_K - 1) begin
                            ri <= ri + 1;
                        end else begin
                            ri <= 0; rk <= 0;
                            state <= FILL_Y;
                        end
                    end
                end

                FILL_Y: begin
                    y_mem[ri][rj] <= (ri < REAL_M && rj < REAL_N)
                                     ? raw_y[ri][rj] : 32'd0;
                    if (rj < PAD_N - 1) begin
                        rj <= rj + 1;
                    end else begin
                        rj <= 0;
                        if (ri < PAD_M - 1) begin
                            ri <= ri + 1;
                        end else begin
                            ri <= 0; rj <= 0;
                            state <= DONE;
                        end
                    end
                end

                DONE: begin
                    pad_done <= 1;
                end

                default: state <= FILL_X;
            endcase
        end
    end
endmodule


// =============================================================================
// Testbench
// =============================================================================
`timescale 1ns / 1ps

module tb_conv;

    localparam TILE   = 4;
    localparam REAL_K = 4;
    localparam REAL_M = 9;
    localparam REAL_N = 1;

    localparam PAD_K  = ((REAL_K + TILE - 1) / TILE) * TILE;  // = 4
    localparam PAD_M  = ((REAL_M + TILE - 1) / TILE) * TILE;  // = 12
    localparam PAD_N  = ((REAL_N + TILE - 1) / TILE) * TILE;  // = 4

    logic clk = 0, rst = 1;
    always #5 clk = ~clk;

    logic signed [31:0] col_matrix [REAL_K][REAL_M];
    logic signed [31:0] kern_flat  [REAL_M][REAL_N];

    logic signed [31:0] x_mem [PAD_K][PAD_M];
    logic signed [31:0] y_mem [PAD_M][PAD_N];
    logic pad_done;

    padding_engine #(
        .TILE   (TILE),
        .REAL_K (REAL_K),
        .REAL_M (REAL_M),
        .REAL_N (REAL_N)
    ) pe (
        .raw_x   (col_matrix),
        .raw_y   (kern_flat),
        .x_mem   (x_mem),
        .y_mem   (y_mem),
        .pad_done(pad_done),
        .clk     (clk),
        .rst     (rst)
    );

    logic signed [31:0] a_feed [TILE];
    logic signed [31:0] b_feed [TILE];
    logic signed [63:0] c_out  [TILE][TILE];
    logic sa_rst = 1;

    systolic_tiled #(.TILE(TILE)) sa (
        .clk(clk), .rst(sa_rst),
        .a(a_feed), .b(b_feed),
        .c(c_out)
    );

    int feed_step;

    localparam signed [63:0] EXP_00 = -64'd8;
    localparam signed [63:0] EXP_01 = -64'd8;
    localparam signed [63:0] EXP_10 = -64'd8;
    localparam signed [63:0] EXP_11 = -64'd8;

    // -----------------------------------------------------------------------
    // Task: load_matrices
    // -----------------------------------------------------------------------
    task automatic load_matrices();
        logic signed [31:0] img [4][4];
        logic signed [31:0] k   [3][3];
        int kr, kc;

        img[0][0]=32'd1;  img[0][1]=32'd2;  img[0][2]=32'd3;  img[0][3]=32'd4;
        img[1][0]=32'd5;  img[1][1]=32'd6;  img[1][2]=32'd7;  img[1][3]=32'd8;
        img[2][0]=32'd9;  img[2][1]=32'd10; img[2][2]=32'd11; img[2][3]=32'd12;
        img[3][0]=32'd13; img[3][1]=32'd14; img[3][2]=32'd15; img[3][3]=32'd16;

        k[0][0]=32'd1;  k[0][1]=32'd0;  k[0][2]=-32'd1;
        k[1][0]=32'd2;  k[1][1]=32'd0;  k[1][2]=-32'd2;
        k[2][0]=32'd1;  k[2][1]=32'd0;  k[2][2]=-32'd1;

        for (kr=0; kr<3; kr++) for (kc=0; kc<3; kc++)
            col_matrix[0][kr*3+kc] = img[kr][kc];
        for (kr=0; kr<3; kr++) for (kc=0; kc<3; kc++)
            col_matrix[1][kr*3+kc] = img[kr][kc+1];
        for (kr=0; kr<3; kr++) for (kc=0; kc<3; kc++)
            col_matrix[2][kr*3+kc] = img[kr+1][kc];
        for (kr=0; kr<3; kr++) for (kc=0; kc<3; kc++)
            col_matrix[3][kr*3+kc] = img[kr+1][kc+1];

        for (kr=0; kr<3; kr++) for (kc=0; kc<3; kc++)
            kern_flat[kr*3+kc][0] = k[kr][kc];
    endtask

    // -----------------------------------------------------------------------
    // Task: debug_dump ? print padded matrices so we can verify contents
    // -----------------------------------------------------------------------
    task automatic debug_dump();
        int r, c;
        $display("\n--- x_mem (col_matrix padded) ---");
        for (r=0; r<PAD_K; r++) begin
            for (c=0; c<PAD_M; c++)
                $write("%4d ", x_mem[r][c]);
            $write("\n");
        end
        $display("\n--- y_mem (kern_flat padded) ---");
        for (r=0; r<PAD_M; r++) begin
            for (c=0; c<PAD_N; c++)
                $write("%4d ", y_mem[r][c]);
            $write("\n");
        end
    endtask

    // -----------------------------------------------------------------------
    // Task: stream_to_systolic
    // a[j] feeds column j from the top  (north input)
    // b[i] feeds row    i from the left (west  input)
    //
    // For C = X * Y  where X is PAD_K x PAD_M and Y is PAD_M x PAD_N:
    //   a[j] = column j of Y  (flows north->south, into column j of PEs)
    //   b[i] = row    i of X  (flows west->east,   into row    i of PEs)
    //
    // Both are skewed: input t is delayed by t cycles so the diagonal
    // wavefront lines up correctly across the PE grid.
    // -----------------------------------------------------------------------
    task automatic stream_to_systolic();
        int t;

        sa_rst = 1;
        @(posedge clk); #1;
        sa_rst = 0;

        for (feed_step = 0; feed_step < PAD_M + TILE; feed_step++) begin
            for (t = 0; t < TILE; t++) begin
                if ((feed_step - t) >= 0 && (feed_step - t) < PAD_M) begin
                    // b[i] = row i of X  ? x_mem[i][feed_step-i]
                    b_feed[t] = x_mem[t][feed_step - t];
                    // a[j] = col j of Y  ? y_mem[feed_step-j][j]
                    a_feed[t] = y_mem[feed_step - t][t];
                end else begin
                    a_feed[t] = '0;
                    b_feed[t] = '0;
                end
            end
            @(posedge clk); #1;
        end

        // Drain
        for (t = 0; t < TILE + 2; t++) begin
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
    int pass_count, fail_count;

    task automatic check_result(
        input string              label,
        input logic signed [63:0] got,
        input logic signed [63:0] expected
    );
        if (got === expected) begin
            $display("  PASS  %s : got=%0d", label, got);
            pass_count++;
        end else begin
            $display("  FAIL  %s : got=%0d  expected=%0d", label, got, expected);
            fail_count++;
        end
    endtask

    // -----------------------------------------------------------------------
    // Main test sequence
    // -----------------------------------------------------------------------
    initial begin
        int i;

        pass_count = 0;
        fail_count = 0;

        for (i = 0; i < TILE; i++) begin
            a_feed[i] = '0;
            b_feed[i] = '0;
        end

        load_matrices();

        @(posedge clk); #1;
        rst = 0;

        wait(pad_done === 1'b1);
        @(posedge clk); #1;

        // Print padded matrices so we can verify data loaded correctly
        debug_dump();

        $display("\n========================================");
        $display(" Convolution Testbench (im2col path)");
        $display("========================================");
        $display(" Input  : 4x4 image, 3x3 Sobel-X kernel");
        $display(" Expect : all four outputs = -8");
        $display("----------------------------------------");

        stream_to_systolic();

        // Print full c_out array so we can see ALL PE results
        $display("\n--- full c_out grid ---");
        for (i = 0; i < TILE; i++) begin
            for (int j = 0; j < TILE; j++)
                $write("%6d ", c_out[i][j]);
            $write("\n");
        end

        $display("\n--- Result check ---");
        check_result("out[0,0]", c_out[0][0], EXP_00);
        check_result("out[0,1]", c_out[1][0], EXP_01);
        check_result("out[1,0]", c_out[2][0], EXP_10);
        check_result("out[1,1]", c_out[3][0], EXP_11);

        $display("\n========================================");
        $display(" Results: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("========================================\n");

        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("SOME TESTS FAILED ? check waveform");

        $finish;
    end

    // -----------------------------------------------------------------------
    // Waveform dump
    // -----------------------------------------------------------------------
    initial begin
        $dumpfile("tb_conv.vcd");
        $dumpvars(0, tb_conv);
    end

    // -----------------------------------------------------------------------
    // Timeout watchdog
    // -----------------------------------------------------------------------
    initial begin
        #100000;
        $display("TIMEOUT ? simulation did not finish in time");
        $finish;
    end

endmodule
