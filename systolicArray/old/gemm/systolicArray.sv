module systolic_tiled #(parameter TILE_MAX = 4) (
    output logic signed [31:0] c    [TILE_MAX][TILE_MAX],  // 32-bit accumulator output
    input  logic signed  [7:0] a    [TILE_MAX],            // 8-bit column feeds (north)
    input  logic signed  [7:0] b    [TILE_MAX],            // 8-bit row feeds    (west)
    input  logic clk, rst
);
    logic signed [7:0] south [TILE_MAX][TILE_MAX];  // 8-bit pass-through
    logic signed [7:0] east  [TILE_MAX][TILE_MAX];  // 8-bit pass-through

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


// ============================================================================
// SIMD Requantize ? 4 lanes
// ============================================================================
module simd_requant (
    input  logic             clk,
    input  logic             rst_n,
    input  logic [3:0][31:0] acc_in,
    input  logic [3:0][31:0] bias_in,
    input  logic [15:0]      M_int,
    input  logic  [4:0]      shift,
    output logic [3:0] [7:0] data_out
);
    logic [3:0][7:0] result;
    genvar i;
    generate
        for (i=0; i<4; i++) begin : lane
            logic signed [31:0] biased;
            logic signed [47:0] scaled;
            logic signed [31:0] shifted;
            logic signed  [7:0] clipped;
            assign biased  = acc_in[i] + bias_in[i];
            assign scaled  = (biased <= 0) ? 48'sd0
                           : ($signed(48'(biased)) * $signed(16'(M_int)));
            assign shifted = ($signed(scaled) + $signed(48'(1) << (shift - 1))) >>> shift;
            assign clipped = (shifted >  127) ?  8'sd127 :
                             (shifted < -128) ? -8'sd128 :
                              shifted[7:0];
            assign result[i] = clipped;
        end
    endgenerate
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) data_out <= '0;
        else        data_out <= result;
    end
endmodule

