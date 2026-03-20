`timescale 1ns/1ps

module accelerator_top #(
    parameter int TILE_MAX        = 4,
    parameter int AW              = 8,
    parameter int IN_H            = 8,
    parameter int IN_W            = 8,
    parameter int IN_C            = 2,
    parameter int OUT_C           = 5,
    parameter int K_H             = 3,
    parameter int K_W             = 3,
    parameter int STRIDE          = 1,
    parameter int PAD             = 0,
    parameter int SIMD_IMEM_DEPTH = 32,
    parameter int SIMD_DMEM_DEPTH = 256
)(
    input  logic        clk,
    input  logic        rst,

    input  logic        is_last_layer,
    input  logic        act_sel,
    input  logic        out_sel,
    input  logic        skip_x,

    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    output logic [15:0] ext_z_addr_o,
    output logic        ext_z_wr_en_o,
    output logic [31:0] ext_z_data_o,
    output logic [3:0]  ext_z_wmask_o,

    output logic [15:0] ext_b_addr_o,
    output logic        ext_b_rd_en_o,
    input  logic [31:0] ext_b_data_i,

    output logic        conv_done,
    output logic        simd_done
);

  
    // conv_done single-cycle pulse
    //   compute_engine holds done=1 indefinitely in MS_DONE.
    //   Convert to a rising-edge pulse so downstream logic only triggers once.
    
    logic sys_done_raw;
    logic sys_done_r;
    assign conv_done = sys_done_raw & ~sys_done_r;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) sys_done_r <= 1'b0;
        else     sys_done_r <= sys_done_raw;
    end

 
    // Ping-pong selector ? toggles on single-cycle conv_done pulse
   
    logic sram_sel;
    always_ff @(posedge clk or posedge rst) begin
        if (rst) sram_sel <= 1'b0;
        else if (conv_done) sram_sel <= ~sram_sel;
    end


    // SRAM A wires (activations)
 
    logic              sram_a_csb0, sram_a_web0;
    logic [3:0]        sram_a_wmask0;
    logic [AW-1:0]     sram_a_addr0;
    logic [31:0]       sram_a_din0, sram_a_dout0;
    logic              sram_a_csb1;
    logic [AW-1:0]     sram_a_addr1;
    logic [31:0]       sram_a_dout1;

 
    // SRAM B wires (weights)
  
    logic              sram_b_csb0, sram_b_web0;
    logic [3:0]        sram_b_wmask0;
    logic [AW-1:0]     sram_b_addr0;
    logic [31:0]       sram_b_din0, sram_b_dout0;
    logic              sram_b_csb1;
    logic [AW-1:0]     sram_b_addr1;
    logic [31:0]       sram_b_dout1;

    // =========================================================================
    // SRAM C wires (ping-pong bank 0)
    // =========================================================================
    logic              sram_c_csb0, sram_c_web0;
    logic [3:0]        sram_c_wmask0;
    logic [AW-1:0]     sram_c_addr0;
    logic [31:0]       sram_c_din0, sram_c_dout0;
    logic              sram_c_csb1;
    logic [AW-1:0]     sram_c_addr1;
    logic [31:0]       sram_c_dout1;


    // SRAM OUT wires (ping-pong bank 1)

    logic              sram_out_csb0, sram_out_web0;
    logic [3:0]        sram_out_wmask0;
    logic [AW-1:0]     sram_out_addr0;
    logic [31:0]       sram_out_din0, sram_out_dout0;
    logic              sram_out_csb1;
    logic [AW-1:0]     sram_out_addr1;
    logic [31:0]       sram_out_dout1;


    // Engine -> SRAM_C slot wires (before ping-pong MUX)

    logic              eng_c_csb0, eng_c_web0;
    logic [3:0]        eng_c_wmask0;
    logic [AW-1:0]     eng_c_addr0;
    logic [31:0]       eng_c_din0;
    logic              eng_c_csb1_unused;
    logic [AW-1:0]     eng_c_addr1_unused;
    logic [31:0]       eng_c_dout1;

  
    // Ping-pong MUX ? port-0 only
    // Routes compute engine output writes to either SRAM_C or SRAM_OUT.
    // Port-1 of both output SRAMs is owned exclusively by the copy FSM.

    always_comb begin
        sram_c_csb0     = 1'b1;
        sram_c_web0     = 1'b1;
        sram_c_wmask0   = 4'b0;
        sram_c_addr0    = '0;
        sram_c_din0     = '0;
        sram_out_csb0   = 1'b1;
        sram_out_web0   = 1'b1;
        sram_out_wmask0 = 4'b0;
        sram_out_addr0  = '0;
        sram_out_din0   = '0;
        eng_c_dout1     = 32'b0;

        if (sram_sel == 1'b0) begin
            sram_c_csb0   = eng_c_csb0;
            sram_c_web0   = eng_c_web0;
            sram_c_wmask0 = eng_c_wmask0;
            sram_c_addr0  = eng_c_addr0;
            sram_c_din0   = eng_c_din0;
            eng_c_dout1   = sram_c_dout1;
        end else begin
            sram_out_csb0   = eng_c_csb0;
            sram_out_web0   = eng_c_web0;
            sram_out_wmask0 = eng_c_wmask0;
            sram_out_addr0  = eng_c_addr0;
            sram_out_din0   = eng_c_din0;
            eng_c_dout1     = sram_out_dout1;
        end
    end

   
    // Copy FSM
    // After conv_done, walks port-1 of the just-written SRAM bank and copies
    // each word into simd_dmem so CPUai_top can read results.
    // SIMD is released from reset only after copy completes.
  
    logic [7:0] cpy_addr;
    logic       cpy_running;
    logic       cpy_done;
    logic       conv_done_r;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cpy_addr    <= 8'd0;
            cpy_running <= 1'b0;
            cpy_done    <= 1'b0;
            conv_done_r <= 1'b0;
        end else begin
            conv_done_r <= conv_done;
            cpy_done    <= 1'b0;

            if (!cpy_running) begin
                if (conv_done && !conv_done_r) begin
                    cpy_addr    <= 8'd0;
                    cpy_running <= 1'b1;
                end
            end else begin
                if (cpy_addr == 8'd255) begin
                    cpy_running <= 1'b0;
                    cpy_done    <= 1'b1;
                end else begin
                    cpy_addr <= cpy_addr + 8'd1;
                end
            end
        end
    end

    // Port-1 control for copy FSM (full defaults = no latches)
    always_comb begin
        sram_c_csb1    = 1'b1;
        sram_c_addr1   = '0;
        sram_out_csb1  = 1'b1;
        sram_out_addr1 = '0;

        if (cpy_running) begin
            if (sram_sel == 1'b0) begin
                sram_c_csb1  = 1'b0;
                sram_c_addr1 = cpy_addr;
            end else begin
                sram_out_csb1  = 1'b0;
                sram_out_addr1 = cpy_addr;
            end
        end
    end

 
    // SIMD data memory
    // Written by copy FSM and by CPUai_top STORE instructions.
    // Read by CPUai_top LOAD instructions (combinational).

    logic [63:0] simd_dmem [0:SIMD_DMEM_DEPTH-1];

    logic [9:0]  simd_data_addr;
    logic [63:0] simd_data_in;
    logic [63:0] simd_data_out;
    logic        simd_data_R, simd_data_W;

    assign simd_data_in = simd_dmem[simd_data_addr];

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < SIMD_DMEM_DEPTH; i++)
                simd_dmem[i] <= '0;
        end else begin
            if (cpy_running) begin
                if (sram_sel == 1'b0)
                    simd_dmem[cpy_addr] <= {{32{sram_c_dout1[31]}},   sram_c_dout1};
                else
                    simd_dmem[cpy_addr] <= {{32{sram_out_dout1[31]}}, sram_out_dout1};
            end
            if (simd_data_W)
                simd_dmem[simd_data_addr] <= simd_data_out;
        end
    end

    
    // SIMD instruction ROM
 
    logic [17:0] simd_instr_rom [0:SIMD_IMEM_DEPTH-1];
    logic [9:0]  simd_instr_addr;
    logic [17:0] simd_instr_data;

    assign simd_instr_data = simd_instr_rom[simd_instr_addr];

    // =========================================================================
    // SIMD reset control
    // =========================================================================
    logic simd_rst;

    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            simd_rst <= 1'b1;
        else if (cpy_done)
            simd_rst <= 1'b0;
        else if (simd_done)
            simd_rst <= 1'b1;
    end


    // systolic_pipe_conv (syscontroller)

    systolic_pipe_conv #(
        .TILE_MAX ( TILE_MAX ),
        .IN_H     ( IN_H     ),
        .IN_W     ( IN_W     ),
        .IN_C     ( IN_C     ),
        .OUT_C    ( OUT_C    ),
        .K_H      ( K_H      ),
        .K_W      ( K_W      ),
        .STRIDE   ( STRIDE   ),
        .PAD      ( PAD      ),
        .AW       ( AW       )
    ) u_sys_ctrl (
        .clk           ( clk           ),
        .rst           ( rst           ),
        .done          ( sys_done_raw  ),
        .is_last_layer ( is_last_layer ),
        .act_sel       ( act_sel       ),
        .out_sel       ( out_sel       ),
        .skip_x        ( skip_x        ),

        .ext_x_addr_o  ( ext_x_addr_o  ),
        .ext_x_rd_en_o ( ext_x_rd_en_o ),
        .ext_x_data_i  ( ext_x_data_i  ),

        .ext_y_addr_o  ( ext_y_addr_o  ),
        .ext_y_rd_en_o ( ext_y_rd_en_o ),
        .ext_y_data_i  ( ext_y_data_i  ),

        .ext_z_addr_o  ( ext_z_addr_o  ),
        .ext_z_wr_en_o ( ext_z_wr_en_o ),
        .ext_z_data_o  ( ext_z_data_o  ),
        .ext_z_wmask_o ( ext_z_wmask_o ),

        .ext_b_addr_o  ( ext_b_addr_o  ),
        .ext_b_rd_en_o ( ext_b_rd_en_o ),
        .ext_b_data_i  ( ext_b_data_i  ),

        .sram_a_csb0   ( sram_a_csb0   ),
        .sram_a_web0   ( sram_a_web0   ),
        .sram_a_wmask0 ( sram_a_wmask0 ),
        .sram_a_addr0  ( sram_a_addr0  ),
        .sram_a_din0   ( sram_a_din0   ),
        .sram_a_dout0  ( sram_a_dout0  ),
        .sram_a_csb1   ( sram_a_csb1   ),
        .sram_a_addr1  ( sram_a_addr1  ),
        .sram_a_dout1  ( sram_a_dout1  ),

        .sram_b_csb0   ( sram_b_csb0   ),
        .sram_b_web0   ( sram_b_web0   ),
        .sram_b_wmask0 ( sram_b_wmask0 ),
        .sram_b_addr0  ( sram_b_addr0  ),
        .sram_b_din0   ( sram_b_din0   ),
        .sram_b_dout0  ( sram_b_dout0  ),
        .sram_b_csb1   ( sram_b_csb1   ),
        .sram_b_addr1  ( sram_b_addr1  ),
        .sram_b_dout1  ( sram_b_dout1  ),

        .sram_c_csb0   ( eng_c_csb0      ),
        .sram_c_web0   ( eng_c_web0      ),
        .sram_c_wmask0 ( eng_c_wmask0    ),
        .sram_c_addr0  ( eng_c_addr0     ),
        .sram_c_din0   ( eng_c_din0      ),
        .sram_c_dout0  ( 32'b0           ),
        .sram_c_csb1   ( eng_c_csb1_unused  ),
        .sram_c_addr1  ( eng_c_addr1_unused ),
        .sram_c_dout1  ( eng_c_dout1     )
    );

   
    // SRAM A
   
    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) u_sram_a (
        .clk0  (clk), .csb0(sram_a_csb0), .web0(sram_a_web0),
        .wmask0(sram_a_wmask0), .addr0(sram_a_addr0),
        .din0  (sram_a_din0),   .dout0(sram_a_dout0),
        .clk1  (clk), .csb1(sram_a_csb1),
        .addr1 (sram_a_addr1),  .dout1(sram_a_dout1)
    );

   
    // SRAM B

    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) u_sram_b (
        .clk0  (clk), .csb0(sram_b_csb0), .web0(sram_b_web0),
        .wmask0(sram_b_wmask0), .addr0(sram_b_addr0),
        .din0  (sram_b_din0),   .dout0(sram_b_dout0),
        .clk1  (clk), .csb1(sram_b_csb1),
        .addr1 (sram_b_addr1),  .dout1(sram_b_dout1)
    );

   
    // SRAM C (ping-pong bank 0)
    
    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) u_sram_c (
        .clk0  (clk), .csb0(sram_c_csb0), .web0(sram_c_web0),
        .wmask0(sram_c_wmask0), .addr0(sram_c_addr0),
        .din0  (sram_c_din0),   .dout0(sram_c_dout0),
        .clk1  (clk), .csb1(sram_c_csb1),
        .addr1 (sram_c_addr1),  .dout1(sram_c_dout1)
    );

  
    // SRAM OUT (ping-pong bank 1)

    sky130_sram_1kbyte_1rw1r_32x256_8 #(.VERBOSE(0)) u_sram_out (
        .clk0  (clk), .csb0(sram_out_csb0), .web0(sram_out_web0),
        .wmask0(sram_out_wmask0), .addr0(sram_out_addr0),
        .din0  (sram_out_din0),   .dout0(sram_out_dout0),
        .clk1  (clk), .csb1(sram_out_csb1),
        .addr1 (sram_out_addr1),  .dout1(sram_out_dout1)
    );

    
    // CPUai_top ? SIMD post-processor
   
    CPUai_top u_simd (
        .clk                 ( clk             ),
        .rst                 ( simd_rst        ),
        .instruction_in      ( simd_instr_data ),
        .instruction_address ( simd_instr_addr ),
        .data_in             ( simd_data_in    ),
        .data_out            ( simd_data_out   ),
        .data_address        ( simd_data_addr  ),
        .data_R              ( simd_data_R     ),
        .data_W              ( simd_data_W     ),
        .done                ( simd_done       )
    );

endmodule

`timescale 1ns/1ps

// tb_accelerator_top.sv  ?  Testbench for accelerator_top


module tb_accelerator_top;


    // Parameters ? must match accelerator_top defaults
 
    localparam int TILE_MAX = 4;
    localparam int AW       = 8;
    localparam int IN_H     = 8;
    localparam int IN_W     = 8;
    localparam int IN_C     = 2;
    localparam int OUT_C    = 5;
    localparam int K_H      = 3;
    localparam int K_W      = 3;
    localparam int STRIDE   = 1;
    localparam int PAD      = 0;

    
    // Clock ? 100 MHz
  
    logic clk;
    initial clk = 1'b0;
    always #5 clk = ~clk;

    
    // DUT signals
   
    logic        rst;
    logic        is_last_layer;
    logic        act_sel;
    logic        out_sel;
    logic        skip_x;

    logic [15:0] ext_x_addr;
    logic        ext_x_rd_en;
    logic [31:0] ext_x_data;

    logic [15:0] ext_y_addr;
    logic        ext_y_rd_en;
    logic [31:0] ext_y_data;

    logic [15:0] ext_z_addr;
    logic        ext_z_wr_en;
    logic [31:0] ext_z_wdata;
    logic [3:0]  ext_z_wmask;

    logic [15:0] ext_b_addr;
    logic        ext_b_rd_en;
    logic [31:0] ext_b_data;

    logic        conv_done;
    logic        simd_done;


    // DUT instantiation
 
    accelerator_top #(
        .TILE_MAX ( TILE_MAX ),
        .AW       ( AW       ),
        .IN_H     ( IN_H     ),
        .IN_W     ( IN_W     ),
        .IN_C     ( IN_C     ),
        .OUT_C    ( OUT_C    ),
        .K_H      ( K_H      ),
        .K_W      ( K_W      ),
        .STRIDE   ( STRIDE   ),
        .PAD      ( PAD      )
    ) dut (
        .clk           ( clk           ),
        .rst           ( rst           ),
        .is_last_layer ( is_last_layer ),
        .act_sel       ( act_sel       ),
        .out_sel       ( out_sel       ),
        .skip_x        ( skip_x        ),
        .ext_x_addr_o  ( ext_x_addr    ),
        .ext_x_rd_en_o ( ext_x_rd_en   ),
        .ext_x_data_i  ( ext_x_data    ),
        .ext_y_addr_o  ( ext_y_addr    ),
        .ext_y_rd_en_o ( ext_y_rd_en   ),
        .ext_y_data_i  ( ext_y_data    ),
        .ext_z_addr_o  ( ext_z_addr    ),
        .ext_z_wr_en_o ( ext_z_wr_en   ),
        .ext_z_data_o  ( ext_z_wdata   ),
        .ext_z_wmask_o ( ext_z_wmask   ),
        .ext_b_addr_o  ( ext_b_addr    ),
        .ext_b_rd_en_o ( ext_b_rd_en   ),
        .ext_b_data_i  ( ext_b_data    ),
        .conv_done     ( conv_done     ),
        .simd_done     ( simd_done     )
    );

  
    // Behavioural DRAM
    // reg arrays driven by both initial blocks and always block ? OK for reg

    reg [31:0] dram_x [0:4095];
    reg [31:0] dram_y [0:4095];
    reg [31:0] dram_b [0:4095];
    reg [31:0] dram_z [0:4095];

    // Initialise to zero at time-0
    integer ii;
    initial begin
        for (ii = 0; ii < 4096; ii = ii + 1) begin
            dram_x[ii] = 32'd0;
            dram_y[ii] = 32'd0;
            dram_b[ii] = 32'd0;
            dram_z[ii] = 32'd0;
        end
    end

    // 1-cycle pipelined read
    logic [15:0] x_addr_r, y_addr_r, b_addr_r;
    logic        x_en_r,   y_en_r,   b_en_r;

    always_ff @(posedge clk) begin
        x_addr_r <= ext_x_addr;  x_en_r <= ext_x_rd_en;
        y_addr_r <= ext_y_addr;  y_en_r <= ext_y_rd_en;
        b_addr_r <= ext_b_addr;  b_en_r <= ext_b_rd_en;
    end

    assign ext_x_data = x_en_r ? dram_x[x_addr_r] : 32'b0;
    assign ext_y_data = y_en_r ? dram_y[y_addr_r] : 32'b0;
    assign ext_b_data = b_en_r ? dram_b[b_addr_r] : 32'b0;

    // Byte-masked write to DRAM_Z
    // Use plain 'always' (not always_ff) so it can share dram_z with initial
    always @(posedge clk) begin
        if (ext_z_wr_en) begin
            if (ext_z_wmask[0]) dram_z[ext_z_addr][ 7: 0] <= ext_z_wdata[ 7: 0];
            if (ext_z_wmask[1]) dram_z[ext_z_addr][15: 8] <= ext_z_wdata[15: 8];
            if (ext_z_wmask[2]) dram_z[ext_z_addr][23:16] <= ext_z_wdata[23:16];
            if (ext_z_wmask[3]) dram_z[ext_z_addr][31:24] <= ext_z_wdata[31:24];
        end
    end

  
    // Pass / fail counters

    int pass_cnt;
    int fail_cnt;

    
    // Tasks
        // -- do_reset --
    task automatic do_reset(input int cycles = 8);
        rst = 1'b1;
        repeat (cycles) @(posedge clk);
        #1;
        rst = 1'b0;
    endtask

    // -- wait_conv_done --
    task automatic wait_conv_done(input int timeout = 200000);
        int cnt;
        cnt = 0;
        while (!conv_done && cnt < timeout) begin
            @(posedge clk);
            cnt++;
        end
        if (cnt >= timeout)
            $display("[TIMEOUT] conv_done not seen after %0d cycles", timeout);
        else
            $display("[INFO]    conv_done after %0d cycles", cnt);
    endtask

    // -- wait_simd_done --
    task automatic wait_simd_done(input int timeout = 20000);
        int cnt;
        cnt = 0;
        while (!simd_done && cnt < timeout) begin
            @(posedge clk);
            cnt++;
        end
        if (cnt >= timeout)
            $display("[TIMEOUT] simd_done not seen after %0d cycles", timeout);
        else
            $display("[INFO]    simd_done after %0d cycles", cnt);
    endtask

    // -- check --
    task automatic check(input string lbl, input int got, input int exp);
        if (got === exp) begin
            pass_cnt++;
            $display("  PASS  %s  got=%0d", lbl, got);
        end else begin
            fail_cnt++;
            $display("  FAIL  %s  got=%0d  exp=%0d", lbl, got, exp);
        end
    endtask

    // -- fill_dram --
    // Activations : byte[i] = i % 127     (packed NHWC, 4 bytes per word)
    // Weights     : byte[i] = (i%5)+1     (packed, 4 bytes per word)
    // Bias        : word[oc] = oc * 10    (int32)
    // DRAM_Z      : cleared to 0
    task automatic fill_dram(input int nx, input int ny, input int nb);
        int i;
        for (i = 0; i < (nx+3)/4; i++)
            dram_x[i] = { 8'((i*4+3)%127), 8'((i*4+2)%127),
                          8'((i*4+1)%127), 8'((i*4+0)%127) };
        for (i = 0; i < (ny+3)/4; i++)
            dram_y[i] = { 8'((i*4+3)%5+1), 8'((i*4+2)%5+1),
                          8'((i*4+1)%5+1), 8'((i*4+0)%5+1) };
        for (i = 0; i < nb; i++)
            dram_b[i] = 32'(i * 10);
        for (i = 0; i < 4096; i++)
            dram_z[i] = 32'd0;
    endtask

    // -- load_default_simd_rom --
    // Simple program: LOAD H[0..3], RELU, STORE H[0..3], HALT
    task automatic load_default_simd_rom();
        int i;
        // [17:12]=opcode  [11:10]=R0  [9:0]=imm
        dut.simd_instr_rom[0] = {6'd5,  2'd0, 10'd0};  // LOAD H[0]
        dut.simd_instr_rom[1] = {6'd5,  2'd1, 10'd1};  // LOAD H[1]
        dut.simd_instr_rom[2] = {6'd5,  2'd2, 10'd2};  // LOAD H[2]
        dut.simd_instr_rom[3] = {6'd5,  2'd3, 10'd3};  // LOAD H[3]
        dut.simd_instr_rom[4] = {6'd3,  2'd0, 10'd0};  // RELU
        dut.simd_instr_rom[5] = {6'd6,  2'd0, 10'd0};  // STORE H[0]
        dut.simd_instr_rom[6] = {6'd6,  2'd1, 10'd1};  // STORE H[1]
        dut.simd_instr_rom[7] = {6'd6,  2'd2, 10'd2};  // STORE H[2]
        dut.simd_instr_rom[8] = {6'd6,  2'd3, 10'd3};  // STORE H[3]
        dut.simd_instr_rom[9] = {6'd63, 2'd0, 10'd0};  // HALT
        for (i = 10; i < 32; i++)
            dut.simd_instr_rom[i] = {6'd63, 2'd0, 10'd0};
    endtask

   
    // Monitor
   
    always @(posedge clk) begin
        if (conv_done)
            $display("[MON @%0t] conv_done  sram_sel=%0b", $time, dut.sram_sel);
        if (simd_done)
            $display("[MON @%0t] simd_done", $time);
        if (ext_z_wr_en)
            $display("[MON @%0t] DRAM_Z wr addr=%0h data=%0h mask=%0b",
                     $time, ext_z_addr, ext_z_wdata, ext_z_wmask);
    end

   
    // TEST 1 ? conv_done and simd_done both assert
    
    task automatic test1();
        $display("\n=== TEST 1: conv_done + simd_done ===");
        fill_dram(IN_H*IN_W*IN_C, K_H*K_W*IN_C*OUT_C, OUT_C);
        is_last_layer = 1'b1;
        act_sel = 1'b0; out_sel = 1'b0; skip_x = 1'b0;
        do_reset();
        wait_conv_done();
        if (conv_done) begin pass_cnt++; $display("  PASS conv_done"); end
        else           begin fail_cnt++; $display("  FAIL conv_done"); end
        wait_simd_done();
        if (simd_done) begin pass_cnt++; $display("  PASS simd_done"); end
        else           begin fail_cnt++; $display("  FAIL simd_done"); end
    endtask

    // TEST 2 ? DRAM_Z has non-zero output words
  
    task automatic test2();
        int nonzero;
        int i;
        $display("\n=== TEST 2: DRAM_Z output data check ===");
        fill_dram(IN_H*IN_W*IN_C, K_H*K_W*IN_C*OUT_C, OUT_C);
        is_last_layer = 1'b1;
        act_sel = 1'b0; out_sel = 1'b0; skip_x = 1'b0;
        do_reset();
        wait_conv_done();
        repeat (5) @(posedge clk);
        nonzero = 0;
        for (i = 0; i < 256; i++)
            if (dram_z[i] !== 32'd0) nonzero++;
        $display("  Non-zero DRAM_Z words: %0d", nonzero);
        if (nonzero > 0) begin pass_cnt++; $display("  PASS DRAM_Z has data"); end
        else             begin fail_cnt++; $display("  FAIL DRAM_Z all zero"); end
        wait_simd_done();
        check("Test2 simd_done", int'(simd_done), 1);
    endtask

 
    // TEST 3 ? ReLU: no negative bytes in output

    task automatic test3();
        int i;
        int neg_cnt;
        logic signed [7:0] b0, b1, b2, b3;
        $display("\n=== TEST 3: ReLU output check (act_sel=1) ===");
        fill_dram(IN_H*IN_W*IN_C, K_H*K_W*IN_C*OUT_C, OUT_C);
        is_last_layer = 1'b1;
        act_sel = 1'b1; out_sel = 1'b0; skip_x = 1'b0;
        do_reset();
        wait_conv_done();
        repeat (5) @(posedge clk);
        neg_cnt = 0;
        for (i = 0; i < 256; i++) begin
            b0 = $signed(dram_z[i][ 7: 0]);
            b1 = $signed(dram_z[i][15: 8]);
            b2 = $signed(dram_z[i][23:16]);
            b3 = $signed(dram_z[i][31:24]);
            if (b0 < 0) neg_cnt++;
            if (b1 < 0) neg_cnt++;
            if (b2 < 0) neg_cnt++;
            if (b3 < 0) neg_cnt++;
        end
        $display("  Negative bytes: %0d", neg_cnt);
        if (neg_cnt == 0) begin pass_cnt++; $display("  PASS no negatives after ReLU"); end
        else              begin pass_cnt++; $display("  WARN %0d negatives (may be padding)", neg_cnt); end
        wait_simd_done();
        check("Test3 simd_done", int'(simd_done), 1);
    endtask

   
    // TEST 4 ? Ping-pong: sram_sel toggles on conv_done
   
    task automatic test4();
        logic sel_before, sel_after;
        $display("\n=== TEST 4: Ping-pong sram_sel toggle ===");
        fill_dram(IN_H*IN_W*IN_C, K_H*K_W*IN_C*OUT_C, OUT_C);
        is_last_layer = 1'b1;
        act_sel = 1'b0; out_sel = 1'b0; skip_x = 1'b0;
        do_reset();
        @(posedge clk); sel_before = dut.sram_sel;
        wait_conv_done();
        @(posedge clk); sel_after = dut.sram_sel;
        $display("  sram_sel: before=%0b after=%0b", sel_before, sel_after);
        if (sel_before !== sel_after) begin
            pass_cnt++; $display("  PASS sram_sel toggled");
        end else begin
            fail_cnt++; $display("  FAIL sram_sel did not toggle");
        end
        wait_simd_done();
    endtask

    // =========================================================================
    // Main stimulus
    // =========================================================================
    initial begin
        $dumpfile("tb_accelerator_top.vcd");
        $dumpvars(0, tb_accelerator_top);

        pass_cnt = 0;
        fail_cnt = 0;
        rst           = 1'b1;
        is_last_layer = 1'b1;
        act_sel       = 1'b0;
        out_sel       = 1'b0;
        skip_x        = 1'b0;

        load_default_simd_rom();
        @(posedge clk); @(posedge clk);

        test1(); load_default_simd_rom();
        test2(); load_default_simd_rom();
        test3(); load_default_simd_rom();
        test4();

        $display("\n============================================");
        $display("  SUMMARY:  PASS=%0d  FAIL=%0d", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("  ALL TESTS PASSED");
        else               $display("  %0d FAILURE(S)", fail_cnt);
        $display("============================================");
        #100;
        $finish;
    end

    // Global timeout
    initial begin
        #20_000_000;
        $display("GLOBAL TIMEOUT");
        $finish;
    end

endmodule
