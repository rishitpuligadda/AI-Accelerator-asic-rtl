module systolic #(parameter s1 = 2, s2 = 2) (c, a, b, clk, rst);
    output logic signed [63:0] c[s1][s2];
    input logic signed [31:0] a[s2], b[s1];
    input logic clk, rst;
    logic signed [31:0] south[s1][s2], east[s1][s2];
    genvar i, j;

    generate
        for (i = 0; i < s1; i++) begin: ROW
            for (j = 0; j < s2; j++) begin: COL
                mac PE(
                    .clk(clk),
                    .rst(rst),
                    .north( (i == 0) ? a[j] : south[i-1][j]),
                    .west( (j == 0) ? b[i] : east[i][j-1]),
                    .east(east[i][j]),
                    .south(south[i][j]),
                    .result(c[i][j])
                );
            end
        end
    endgenerate
endmodule

module sys_data #(parameter s1 = 2, M = 2, s2 = 2) (z, x, y, clk, rst, done);
    output logic signed [63:0] z[s1][s2];
    input logic signed [31:0] x[s1][M], y[M][s2];
    input logic clk, rst, done;
    logic signed [31:0] a[s2], b[s1];
    integer t, i;
    localparam TOTAL_CYCLES = M + s1 + s2 -2;

    systolic #(.s1(s1), .s2(s2)) array (
        .clk(clk),
        .rst(rst),
        .c(z),
        .a(a),
        .b(b)
    );

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin 
            t <= 0;
            done <= 0;

            for (i = 0; i < s2; i++)
                a[i] <= 0;
            for (i = 0; i < s1; i++) 
                b[i] <= 0;
        end
        else begin
            if (!done) begin
                for (i = 0; i < s2; i++) begin
                    if (t-i >= 0 && t-i < M)
                        a[i] <= y[t-i][i];
                    else
                        a[i] <= 0;
                end 
                
                for (i = 0; i < s1; i++) begin
                    if (t-i >= 0 && t-i < M)
                        b[i] <= x[i][t-i];
                    else
                        b[i] <= 0;
                end
                
                t <= t + 1;
                
                if (t == TOTAL_CYCLES)
                    done <= 1;
            end
        end
    end
endmodule
