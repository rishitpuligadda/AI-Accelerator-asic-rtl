module systolic_tiled #(parameter TILE_MAX = 4) (
    output logic signed [31:0] c    [TILE_MAX][TILE_MAX],
    input  logic signed  [7:0] a    [TILE_MAX],
    input  logic signed  [7:0] b    [TILE_MAX],
    input  logic clk, rst
);
    logic signed [7:0] south [TILE_MAX][TILE_MAX];
    logic signed [7:0] east  [TILE_MAX][TILE_MAX];

    genvar i, j;
    generate
        for (i=0; i<TILE_MAX; i++) begin : ROW
            for (j=0; j<TILE_MAX; j++) begin : COL
                mac PE (
                    .clk(clk), .rst(rst),
                    .north((i==0) ? a[j] : south[i-1][j]),
                    .west ((j==0) ? b[i] : east [i][j-1]),
                    .east (east [i][j]),
                    .south(south[i][j]),
                    .result(c[i][j])
                );
            end
        end
    endgenerate
endmodule
