module mac(south, east, result, north, west, clk, rst);
    output logic signed  [7:0] south, east;
    output logic signed [31:0] result;
    input  logic signed  [7:0] north, west;
    input  logic clk, rst;

    logic signed [15:0] multi;
    assign multi = north * west;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 32'sd0;
            south  <= 8'sd0;
            east   <= 8'sd0;
        end else begin
            result <= result + 32'(signed'(multi));
            south  <= north;
            east   <= west;
        end
    end
endmodule
