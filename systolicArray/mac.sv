module mac(south, east, result, north, west, clk, rst);
    output logic [31:0] south, east;
    output logic [63:0] result;
    input logic [31:0] north, west;
    input logic clk, rst;
    logic [63:0] multi;
        
    assign multi = north * west;
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            result <= 0;
            south <= 0;
            east <= 0;
        end
        else begin 
            result <= result + multi;
            south <= north;
            east <= west;
        end
    end
endmodule
