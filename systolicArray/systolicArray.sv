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
