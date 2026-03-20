module systolic_square #(parameter N = 2) (
    input  logic clk,
    input  logic rst,

    input  logic signed [31:0] a[N],   // north inputs
    input  logic signed [31:0] b[N],   // west inputs

    output logic signed [63:0] c[N][N]
);

    logic signed [31:0] south[N][N];
    logic signed [31:0] east[N][N];

    genvar i, j;

    generate
        for (i = 0; i < N; i++) begin : ROW
            for (j = 0; j < N; j++) begin : COL

                mac PE(
                    .clk(clk),
                    .rst(rst),

                    .north( (i == 0) ? a[j] : south[i-1][j] ),
                    .west ( (j == 0) ? b[i] : east[i][j-1] ),

                    .south(south[i][j]),
                    .east (east[i][j]),

                    .result(c[i][j])
                );

            end
        end
    endgenerate

endmodule

module sys_data_square #(parameter N = 2) (
   output logic signed [63:0] z[N][N],
    input  logic signed [31:0] x[N][N],
    input  logic signed [31:0] y[N][N],
    input  logic clk,
    input  logic rst
);

    logic signed [31:0] a[N], b[N];
    integer t, i;

    systolic_square #(N) dut (
        .clk(clk),
        .rst(rst),
        .a(a),
        .b(b),
        .c(z)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            t <= 0;

            for (i = 0; i < N; i++) begin
                a[i] <= 0;
                b[i] <= 0;
            end
        end
        else begin

            for (i = 0; i < N; i++) begin
                if (t-i >= 0 && t-i < N)
                    a[i] <= y[t-i][i];
                else
                    a[i] <= 0;

                if (t-i >= 0 && t-i < N)
                    b[i] <= x[i][t-i];
                else
                    b[i] <= 0;
            end

            t <= t + 1;

        end
    end

endmodule

module tb_sys3x3;

    logic clk, rst;
    logic signed [31:0] x[3][3], y[3][3];
    logic signed [63:0] z[3][3];

    sys_data_sqaure #(.N(3)) dut(
        .clk(clk),
        .rst(rst),
        .x(x),
        .y(y),
        .z(z)
    );

    always #5 clk = ~clk;

    initial begin

        clk = 0;
        rst = 1;

        x[0][0] = 1;  x[0][1] = 1; x[0][2] = 1;
        x[1][0] = 1;  x[1][1] = 1; x[1][2] = 1;
        x[2][0] = 1;  x[2][1] = 1; x[2][2] = 1;

        y[0][0] = 1;  y[0][1] = 1; y[0][2] = 1;
        y[1][0] = 1;  y[1][1] = 1; y[1][2] = 1;
        y[2][0] = 1;  y[2][1] = 1; y[2][2] = 1;

        $display("---- Watching wavefront feed ----");

        $monitor("t=%0t | a0=%0d a1=%0d | b0=%0d b1=%0d || z00=%0d z01=%0d z10=%0d z11=%0d",
                  $time,
                  dut.a[0], dut.a[1],
                  dut.b[0], dut.b[1],
                  z[0][0], z[0][1], z[0][2], z[1][0], z[1][1], z[1][2], z[2][0], z[2][1], z[2][2]);

        repeat(2) @(posedge clk);
        rst = 0;

        repeat(8) @(posedge clk);

        $display("\nFinal Result Matrix:");
        $display("%0d %0d %0d", z[0][0], z[0][1], z[0][2]);
        $display("%0d %0d %0d", z[1][0], z[1][1], z[1][2]);
        $display("%0d %0d %0d", z[2][0], z[2][1], z[2][2]);

        $stop;

    end

endmodule
