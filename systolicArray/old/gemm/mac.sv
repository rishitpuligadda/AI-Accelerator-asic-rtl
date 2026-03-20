module mac(south, east, result, north, west, clk, rst);
    output logic signed  [7:0] south, east;   // pass-through registers
    output logic signed [31:0] result;        // accumulator (32-bit is plenty for INT8 x INT8 x depth)
    input  logic signed  [7:0] north, west;   // 8-bit activation and weight inputs
    input  logic clk, rst;

    logic signed [15:0] multi;  // 8x8 product fits in 16 bits

    assign multi = north * west;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 32'sd0;
            south  <= 8'sd0;
            east   <= 8'sd0;
        end else begin
            result <= result + 32'(signed'(multi));  // sign-extend product before accumulating
            south  <= north;
            east   <= west;
        end
    end
endmodule
