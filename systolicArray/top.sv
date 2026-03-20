// =============================================================================
// conv_top.sv  -  Top-level wrapper
//
// Instantiates:
//   - systolic_pipe_conv  (the DUT / sys-controller)
//   - sky130_sram_1kbyte_1rw1r_32x256_8  x3  (sram_a, sram_b, sram_c)
//
// All convolution parameters are exposed as overridable localparams so the
// module can be re-parameterised for Test E / F / G (or any future config)
// without touching the port list.
//
// DRAM interface is kept as raw addr/en/data ports ? identical to the signals
// the DUT drives in tbStress.sv ? so the same DRAM behavioural model in the
// testbench connects without any glue logic.
//
// Port naming follows the DUT convention used in tbStress.sv.
// =============================================================================

`timescale 1ns/1ps

module conv_top #(
    // -----------------------------------------------------------------
    // Tile / memory geometry
    // -----------------------------------------------------------------
    parameter int TILE_MAX = 4,   // max output-positions per tile (K-dim)
    parameter int AW       = 8,   // SRAM address width (256 entries @ 8b)

    // -----------------------------------------------------------------
    // Convolution shape  ? override per test
    // -----------------------------------------------------------------
    // Default parameterisation = Test E  (8x8x2 | 3x3 | stride=1 pad=0 | 5 filters)
    parameter int IN_H   = 8,    // input feature-map height
    parameter int IN_W   = 8,    // input feature-map width
    parameter int IN_C   = 2,    // input channels
    parameter int OUT_C  = 5,    // output channels (number of filters)
    parameter int K_H    = 3,    // kernel height
    parameter int K_W    = 3,    // kernel width
    parameter int STRIDE = 1,    // convolution stride
    parameter int PAD    = 0     // zero-padding (symmetric)
) (
    // -----------------------------------------------------------------
    // Global
    // -----------------------------------------------------------------
    input  logic        clk,
    input  logic        rst,      // active-high synchronous reset
    output logic        done,     // pulses high for one cycle when finished

    // -----------------------------------------------------------------
    // Control side-band (pass straight through to DUT)
    // -----------------------------------------------------------------
    input  logic        is_last_layer,
    input  logic        act_sel,   // activation select (0=none, 1=ReLU, ?)
    input  logic        out_sel,   // output precision select
    input  logic        skip_x,    // skip loading input activations

    // -----------------------------------------------------------------
    // DRAM  ?  X  (input activations, read-only)
    // -----------------------------------------------------------------
    output logic [15:0] ext_x_addr_o,
    output logic        ext_x_rd_en_o,
    input  logic [31:0] ext_x_data_i,

    // -----------------------------------------------------------------
    // DRAM  ?  Y  (weights / kernel, read-only)
    // -----------------------------------------------------------------
    output logic [15:0] ext_y_addr_o,
    output logic        ext_y_rd_en_o,
    input  logic [31:0] ext_y_data_i,

    // -----------------------------------------------------------------
    // DRAM  ?  Z  (output activations, write-only, byte-masked)
    // -----------------------------------------------------------------
    output logic [15:0] ext_z_addr_o,
    output logic        ext_z_wr_en_o,
    output logic [31:0] ext_z_data_o,
    output logic [3:0]  ext_z_wmask_o,

    // -----------------------------------------------------------------
    // DRAM  ?  B  (bias, read-only)
    // -----------------------------------------------------------------
    output logic [15:0] ext_b_addr_o,
    output logic        ext_b_rd_en_o,
    input  logic [31:0] ext_b_data_i
);

    // =================================================================
    // Internal SRAM wires  (DUT ? SRAM)
    // =================================================================

    // --- SRAM A ---
    logic              sram_a_csb0, sram_a_web0;
    logic [3:0]        sram_a_wmask0;
    logic [AW-1:0]     sram_a_addr0;
    logic [31:0]       sram_a_din0,  sram_a_dout0;
    logic              sram_a_csb1;
    logic [AW-1:0]     sram_a_addr1;
    logic [31:0]       sram_a_dout1;

    // --- SRAM B ---
    logic              sram_b_csb0, sram_b_web0;
    logic [3:0]        sram_b_wmask0;
    logic [AW-1:0]     sram_b_addr0;
    logic [31:0]       sram_b_din0,  sram_b_dout0;
    logic              sram_b_csb1;
    logic [AW-1:0]     sram_b_addr1;
    logic [31:0]       sram_b_dout1;

    // --- SRAM C ---
    logic              sram_c_csb0, sram_c_web0;
    logic [3:0]        sram_c_wmask0;
    logic [AW-1:0]     sram_c_addr0;
    logic [31:0]       sram_c_din0,  sram_c_dout0;
    logic              sram_c_csb1;
    logic [AW-1:0]     sram_c_addr1;
    logic [31:0]       sram_c_dout1;

    // =================================================================
    // systolic_pipe_conv  ?  the DUT / systolic controller
    // =================================================================
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
        // global
        .clk             ( clk             ),
        .rst             ( rst             ),
        .done            ( done            ),

        // side-band control
        .is_last_layer   ( is_last_layer   ),
        .act_sel         ( act_sel         ),
        .out_sel         ( out_sel         ),
        .skip_x          ( skip_x          ),

        // DRAM X
        .ext_x_addr_o    ( ext_x_addr_o    ),
        .ext_x_rd_en_o   ( ext_x_rd_en_o   ),
        .ext_x_data_i    ( ext_x_data_i    ),

        // DRAM Y
        .ext_y_addr_o    ( ext_y_addr_o    ),
        .ext_y_rd_en_o   ( ext_y_rd_en_o   ),
        .ext_y_data_i    ( ext_y_data_i    ),

        // DRAM Z
        .ext_z_addr_o    ( ext_z_addr_o    ),
        .ext_z_wr_en_o   ( ext_z_wr_en_o   ),
        .ext_z_data_o    ( ext_z_data_o    ),
        .ext_z_wmask_o   ( ext_z_wmask_o   ),

        // DRAM B
        .ext_b_addr_o    ( ext_b_addr_o    ),
        .ext_b_rd_en_o   ( ext_b_rd_en_o   ),
        .ext_b_data_i    ( ext_b_data_i    ),

        // SRAM A ? port 0 (rw)
        .sram_a_csb0     ( sram_a_csb0     ),
        .sram_a_web0     ( sram_a_web0     ),
        .sram_a_wmask0   ( sram_a_wmask0   ),
        .sram_a_addr0    ( sram_a_addr0    ),
        .sram_a_din0     ( sram_a_din0     ),
        .sram_a_dout0    ( sram_a_dout0    ),
        // SRAM A ? port 1 (ro)
        .sram_a_csb1     ( sram_a_csb1     ),
        .sram_a_addr1    ( sram_a_addr1    ),
        .sram_a_dout1    ( sram_a_dout1    ),

        // SRAM B ? port 0 (rw)
        .sram_b_csb0     ( sram_b_csb0     ),
        .sram_b_web0     ( sram_b_web0     ),
        .sram_b_wmask0   ( sram_b_wmask0   ),
        .sram_b_addr0    ( sram_b_addr0    ),
        .sram_b_din0     ( sram_b_din0     ),
        .sram_b_dout0    ( sram_b_dout0    ),
        // SRAM B ? port 1 (ro)
        .sram_b_csb1     ( sram_b_csb1     ),
        .sram_b_addr1    ( sram_b_addr1    ),
        .sram_b_dout1    ( sram_b_dout1    ),

        // SRAM C ? port 0 (rw)
        .sram_c_csb0     ( sram_c_csb0     ),
        .sram_c_web0     ( sram_c_web0     ),
        .sram_c_wmask0   ( sram_c_wmask0   ),
        .sram_c_addr0    ( sram_c_addr0    ),
        .sram_c_din0     ( sram_c_din0     ),
        .sram_c_dout0    ( sram_c_dout0    ),
        // SRAM C ? port 1 (ro)
        .sram_c_csb1     ( sram_c_csb1     ),
        .sram_c_addr1    ( sram_c_addr1    ),
        .sram_c_dout1    ( sram_c_dout1    )
    );

    // =================================================================
    // SRAM A
    // =================================================================
    sky130_sram_1kbyte_1rw1r_32x256_8 #(
        .VERBOSE ( 0 )
    ) u_sram_a (
        // port 0 ? read/write
        .clk0   ( clk           ),
        .csb0   ( sram_a_csb0   ),
        .web0   ( sram_a_web0   ),
        .wmask0 ( sram_a_wmask0 ),
        .addr0  ( sram_a_addr0  ),
        .din0   ( sram_a_din0   ),
        .dout0  ( sram_a_dout0  ),
        // port 1 ? read-only
        .clk1   ( clk           ),
        .csb1   ( sram_a_csb1   ),
        .addr1  ( sram_a_addr1  ),
        .dout1  ( sram_a_dout1  )
    );

    // =================================================================
    // SRAM B
    // =================================================================
    sky130_sram_1kbyte_1rw1r_32x256_8 #(
        .VERBOSE ( 0 )
    ) u_sram_b (
        .clk0   ( clk           ),
        .csb0   ( sram_b_csb0   ),
        .web0   ( sram_b_web0   ),
        .wmask0 ( sram_b_wmask0 ),
        .addr0  ( sram_b_addr0  ),
        .din0   ( sram_b_din0   ),
        .dout0  ( sram_b_dout0  ),
        .clk1   ( clk           ),
        .csb1   ( sram_b_csb1   ),
        .addr1  ( sram_b_addr1  ),
        .dout1  ( sram_b_dout1  )
    );

    // =================================================================
    // SRAM C
    // =================================================================
    sky130_sram_1kbyte_1rw1r_32x256_8 #(
        .VERBOSE ( 0 )
    ) u_sram_c (
        .clk0   ( clk           ),
        .csb0   ( sram_c_csb0   ),
        .web0   ( sram_c_web0   ),
        .wmask0 ( sram_c_wmask0 ),
        .addr0  ( sram_c_addr0  ),
        .din0   ( sram_c_din0   ),
        .dout0  ( sram_c_dout0  ),
        .clk1   ( clk           ),
        .csb1   ( sram_c_csb1   ),
        .addr1  ( sram_c_addr1  ),
        .dout1  ( sram_c_dout1  )
    );

endmodule
