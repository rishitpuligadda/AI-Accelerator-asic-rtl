module simd_relu_2cycle (
    input  logic              clk,
    input  logic              rst_n,
    input  logic [3:0][63:0]  data_in, 
    output logic [3:0][63:0]  data_out
);

    // Stage 1 Register: Simply capturing the input
    logic [3:0][63:0] pipe_reg_1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) pipe_reg_1 <= '0;
        else        pipe_reg_1 <= data_in;
    end

    // Stage 2: ReLU Logic + Output Register
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_out <= '0;
        end else begin
            // We perform the ReLU on the data that was captured in the PREVIOUS cycle
            for (int i = 0; i < 4; i++) begin
                data_out[i] <= (pipe_reg_1[i][63] == 1'b1) ? 64'h0 : pipe_reg_1[i];
            end
        end
    end

endmodule

`timescale 1ns/1ps

module tb_simd_relu_2cycle;

    logic              clk;
    logic              rst_n;
    logic [3:0][63:0]  data_in;
    logic [3:0][63:0]  data_out;

    // Instantiate the 2-cycle Unit Under Test
    simd_relu_2cycle uut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .data_out(data_out)
    );

    // Clock: 100MHz (10ns period)
    always #5 clk = (clk === 1'b0) ? 1'b1 : 1'b0;

    initial begin
        // Initialize
        clk = 0;
        rst_n = 0;
        data_in = '0;

        $display("--- Starting 2-Cycle SIMD Verification ---");
        #15 rst_n = 1; // Release reset

        // Cycle 1: Feed Input A
        @(posedge clk);
        data_in[0] = 64'd100;                 // Positive
        data_in[1] = 64'hFFFF_FFFF_FFFF_FFFF; // -1 (Negative)
        data_in[2] = 64'd50;                  // Positive
        data_in[3] = 64'h8000_0000_0000_0000; // Min Negative
        $display("T=%0t | Input A fed.", $time);

        // Cycle 2: Feed Input B (while Input A is in Stage 1)
        @(posedge clk);
        data_in[0] = 64'hFFFF_FFFF_FFFF_FFFE; // -2 (Negative)
        data_in[1] = 64'd10;                  // Positive
        data_in[2] = 64'hFFFF_FFFF_FFFF_0000; // Negative
        data_in[3] = 64'd99;                  // Positive
        $display("T=%0t | Input B fed. (Output for A not ready yet)", $time);

        // Cycle 3: Output for A should now be visible!
        @(posedge clk);
        #1; // Wait for logic to settle
        $display("T=%0t | Output for A: %d, %d, %d, %d", $time, 
                 $signed(data_out[0]), $signed(data_out[1]), 
                 $signed(data_out[2]), $signed(data_out[3]));

        // Cycle 4: Output for B should now be visible!
        @(posedge clk);
        #1;
        $display("T=%0t | Output for B: %d, %d, %d, %d", $time, 
                 $signed(data_out[0]), $signed(data_out[1]), 
                 $signed(data_out[2]), $signed(data_out[3]));

        #50;
        $display("--- Verification Complete ---");
        $finish;
    end

endmodule
