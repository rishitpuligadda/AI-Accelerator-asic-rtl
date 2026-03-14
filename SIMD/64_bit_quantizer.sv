module ai_quantizer #(
    parameter int IN_WIDTH  = 64,
    parameter int OUT_WIDTH = 32,
    parameter int FRAC_BITS = 16  // Fractional bits in the output
)(
    input  logic [IN_WIDTH-1:0]  data_in,
    output logic [OUT_WIDTH-1:0] data_out
);

    // Calculate shift amount based on typical Q32.32 input format
    localparam int SHIFT_AMT = (IN_WIDTH / 2) - FRAC_BITS; 
    
    logic [IN_WIDTH:0] rounded_tmp;
    logic signed [IN_WIDTH-1:0] max_limit;
    logic signed [IN_WIDTH-1:0] min_limit;

    assign max_limit = (64'sh1 << (OUT_WIDTH - 1)) - 1;
    assign min_limit = -(64'sh1 << (OUT_WIDTH - 1));

    always_comb begin
        // DECLARATIONS FIRST - Fixed the "Illegal declaration" error
        automatic logic signed [IN_WIDTH-1:0] scaled_val;

        // 1. Rounding (Round to Nearest Half Up)
        rounded_tmp = $signed(data_in) + (64'sh1 << (SHIFT_AMT - 1));

        // 2. Alignment
        scaled_val = $signed(rounded_tmp) >>> SHIFT_AMT;

        // 3. Saturation Logic
        if (scaled_val > max_limit) begin
            data_out = {1'b0, {(OUT_WIDTH-1){1'b1}}}; // Clamp to Max Positive
        end 
        else if (scaled_val < min_limit) begin
            data_out = {1'b1, {(OUT_WIDTH-1){1'b0}}}; // Clamp to Max Negative
        end 
        else begin
            data_out = scaled_val[OUT_WIDTH-1:0];
        end
    end

endmodule

module tb_ai_quantizer();

    // Testbench signals
    logic [63:0] data_in;
    logic [31:0] data_out;

    // Instantiate the Unit Under Test (UUT)
    ai_quantizer #(
        .IN_WIDTH(64),
        .OUT_WIDTH(32),
        .FRAC_BITS(16)
    ) uut (
        .data_in(data_in),
        .data_out(data_out)
    );

    initial begin
        $display("Starting AI Quantizer Test...");
        $monitor("Time=%0t | In=%h | Out=%h", $time, data_in, data_out);

        // Case 1: Normal Value (No Saturation)
        // Let's pass a value that fits easily
        data_in = 64'h0000_0000_1234_5678; 
        #10;

        // Case 2: Positive Overflow (Should Saturate)
        // A very large 64-bit positive number
        data_in = 64'h0FFF_FFFF_FFFF_FFFF;
        #10;

        // Case 3: Negative Overflow (Should Saturate)
        // A very large 64-bit negative number
        data_in = 64'hF000_0000_0000_0000;
        #10;

        // Case 4: Testing Rounding
        // Input with a value that should round up
        data_in = 64'h0000_0000_0000_8000; // Bit below SHIFT_AMT is 1
        #10;

        $display("Test Complete.");
        $stop; // Pauses simulation in ModelSim
    end

endmodule
