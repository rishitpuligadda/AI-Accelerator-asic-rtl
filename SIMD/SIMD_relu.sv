module simd_relu_top (
    input  logic              clk,
    input  logic              rst_n,
    // Four 64-bit values input simultaneously (Total 256 bits)
    input  logic [3:0][63:0]  data_in, 
    // Four 64-bit values output simultaneously
    output logic [3:0][63:0]  data_out
);

    // Internal wire to hold the result of the ReLU operation
    logic [3:0][63:0] relu_results;

    // --- SIMD Parallel Logic ---
    // The 'generate' block ensures 4 parallel 'lanes' of logic are created.
    // Each lane operates independently and simultaneously.
    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : relu_lane
            // Combinational ReLU: 
            // If bit [63] is 1 (negative), output 64 bits of 0.
            // Else, output the original 64-bit value.
            assign relu_results[i] = (data_in[i][63] == 1'b1) ? 64'h0 : data_in[i];
        end
    endgenerate

    // --- Pipeline Register ---
    // This captures the simultaneous results and presents them 
    // on the output bus at the next clock edge.
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= '0;
        end else begin
            data_out <= relu_results;
        end
    end

endmodule

`timescale 1ns/1ps

module tb_simd_relu;

    // Parameters and Signals
    logic              clk;
    logic              rst_n;
    logic [3:0][63:0]  data_in;
    logic [3:0][63:0]  data_out;

    // Instantiate the Unit Under Test (UUT)
    simd_relu_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .data_out(data_out)
    );

    // Clock Generation: 10ns period (100 MHz)
    always #5 clk = ~clk;

    // Test Procedure
    initial begin
        // Initialize signals
        clk = 0;
        rst_n = 0;
        data_in = '0;

        // Reset the system
        $display("--- Starting SIMD ReLU Verification ---");
        #15 rst_n = 1;

        // --- Test Case 1: Mixed Positive and Negative ---
        // Lane 0: 5 (Positive)
        // Lane 1: -1 (Negative, All F's)
        // Lane 2: 100 (Positive)
        // Lane 3: -50 (Negative)
        @(posedge clk);
        data_in[0] = 64'd5;
        data_in[1] = 64'hFFFF_FFFF_FFFF_FFFF; 
        data_in[2] = 64'd100;
        data_in[3] = 64'hFFFF_FFFF_FFFF_FFCE; 
        
        // Wait for one clock cycle to see the latched output
        @(posedge clk);
        #1; // Small delay to allow signals to settle in simulation
        $display("Time: %0t | In: %h | Out: %h", $time, data_in, data_out);
        
        // --- Test Case 2: Boundary Values ---
        // Lane 0: Max Positive
        // Lane 1: Min Negative
        // Lane 2: Zero
        // Lane 3: -2
        @(posedge clk);
        data_in[0] = 64'h7FFF_FFFF_FFFF_FFFF; 
        data_in[1] = 64'h8000_0000_0000_0000;
        data_in[2] = 64'h0000_0000_0000_0000;
        data_in[3] = 64'hFFFF_FFFF_FFFF_FFFE;

        @(posedge clk);
        #1;
        $display("Time: %0t | In: %h | Out: %h", $time, data_in, data_out);

        #20;
        $display("--- Verification Complete ---");
        $finish;
    end

endmodule
