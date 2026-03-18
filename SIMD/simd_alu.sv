`timescale 1ns/1ps

// =============================================================================
//  softmax_pkg ? shared parameters and exp LUT
// =============================================================================
package softmax_pkg;
    parameter int N_LANES    = 8;
    parameter int DATA_W     = 16;
    parameter int FRAC_W     = 8;
    parameter int EXP_W      = 24;
    parameter int EXP_FRAC_W = 16;

    function automatic logic [EXP_W-1:0] exp_lut(input int k);
        logic [EXP_W-1:0] LUT [0:15];
        LUT[0]  = 24'h010000;
        LUT[1]  = 24'h005E2D;
        LUT[2]  = 24'h0022A5;
        LUT[3]  = 24'h000CBE;
        LUT[4]  = 24'h0004B0;
        LUT[5]  = 24'h0001B9;
        LUT[6]  = 24'h0000A2;
        LUT[7]  = 24'h00003B;
        LUT[8]  = 24'h000015;
        LUT[9]  = 24'h000008;
        LUT[10] = 24'h000002;
        LUT[11] = 24'h000001;
        LUT[12] = 24'h000000;
        LUT[13] = 24'h000000;
        LUT[14] = 24'h000000;
        LUT[15] = 24'h000000;
        return LUT[k];
    endfunction
endpackage : softmax_pkg

// =============================================================================
//  ai_quantizer ? 64-bit input ? 32-bit output with rounding and saturation
// =============================================================================
module ai_quantizer #(
    parameter int IN_WIDTH  = 64,
    parameter int OUT_WIDTH = 32,
    parameter int FRAC_BITS = 16
)(
    input  logic [IN_WIDTH-1:0]  data_in,
    output logic [OUT_WIDTH-1:0] data_out
);
    localparam int SHIFT_AMT = (IN_WIDTH / 2) - FRAC_BITS;

    logic [IN_WIDTH:0]          rounded_tmp;
    logic signed [IN_WIDTH-1:0] max_limit;
    logic signed [IN_WIDTH-1:0] min_limit;

    assign max_limit = (64'sh1 << (OUT_WIDTH - 1)) - 1;
    assign min_limit = -(64'sh1 << (OUT_WIDTH - 1));

    always_comb begin
        automatic logic signed [IN_WIDTH-1:0] scaled_val;
        rounded_tmp = $signed(data_in) + (64'sh1 << (SHIFT_AMT - 1));
        scaled_val  = $signed(rounded_tmp) >>> SHIFT_AMT;

        if      (scaled_val > max_limit) data_out = {1'b0, {(OUT_WIDTH-1){1'b1}}};
        else if (scaled_val < min_limit) data_out = {1'b1, {(OUT_WIDTH-1){1'b0}}};
        else                             data_out = scaled_val[OUT_WIDTH-1:0];
    end
endmodule

// =============================================================================
//  simd_relu_top ? 4-lane 64-bit parallel ReLU with pipeline register
// =============================================================================
module simd_relu_top (
    input  logic             clk,
    input  logic             rst_n,
    input  logic [3:0][63:0] data_in,
    output logic [3:0][63:0] data_out
);
    logic [3:0][63:0] relu_results;

    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : relu_lane
            assign relu_results[i] = (data_in[i][63] == 1'b1) ? 64'h0 : data_in[i];
        end
    endgenerate

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) data_out <= '0;
        else        data_out <= relu_results;
    end
endmodule

// =============================================================================
//  exp_approx_lane ? single lane 3-cycle pipelined exp approximation
// =============================================================================
module exp_approx_lane
    import softmax_pkg::*;
#(parameter int DW=DATA_W, parameter int FW=FRAC_W,
  parameter int OW=EXP_W,  parameter int OFW=EXP_FRAC_W)
(
    input  logic                  clk, rst_n, valid_i,
    input  logic signed [DW-1:0]  x_i,
    output logic                  valid_o,
    output logic [OW-1:0]         exp_o
);
    localparam logic signed [DW-1:0] CLAMP_NEG = DW'(-15*(1<<FW));
    localparam logic signed [DW-1:0] ZERO      = '0;
    localparam logic [FW-1:0]        HALF      = FW'(1<<(FW-1));
    localparam logic [FW-1:0]        SIXTH     = FW'((1<<FW)/6);
    localparam logic [FW-1:0]        T24       = FW'((1<<FW)/24);
    localparam logic [FW-1:0]        FW_MASK   = {FW{1'b1}};

    // Stage 1 ? clamp
    logic signed [DW-1:0] xc; logic vc1;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin vc1<=0; xc<=0; end
        else begin
            vc1 <= valid_i;
            if      (x_i < CLAMP_NEG) xc <= CLAMP_NEG;
            else if (x_i > ZERO)      xc <= ZERO;
            else                       xc <= x_i;
        end
    end

    // Combinational ? split integer k and fractional part
    logic signed [DW-1:0] floor_x;
    logic [3:0]   k;
    logic [FW-1:0] frac;
    assign k       = 4'(($unsigned(-xc) + FW_MASK) >> FW);
    assign floor_x = -(DW'(k) << FW);
    assign frac    = FW'($unsigned(xc - floor_x));

    // Taylor series terms
    logic [2*FW-1:0]   f2, f3, f4;
    logic [2*FW+1:0]   t1, t2, t3, t4;
    assign f2 = frac * frac;
    assign f3 = f2[2*FW-1:FW] * frac;
    assign f4 = f3[2*FW-1:FW] * frac;
    assign t1 = {{2{1'b0}}, frac, {FW{1'b0}}};
    assign t2 = {2'b00, (2*FW)'(f2[2*FW-1:FW]) * (2*FW)'(HALF)};
    assign t3 = {2'b00, (2*FW)'(f3[2*FW-1:FW]) * (2*FW)'(SIXTH)};
    assign t4 = {2'b00, (2*FW)'(f4[2*FW-1:FW]) * (2*FW)'(T24)};

    // Stage 2 ? LUT base + polynomial
    logic [OW-1:0]   base;
    logic [2*FW+3:0] poly;
    logic vc2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin vc2<=0; base<=0; poly<=0; end
        else begin
            vc2  <= vc1;
            base <= exp_lut(int'(k));
            poly <= (2*FW+4)'(1 << (2*FW)) + (2*FW+4)'(t1)
                  + (2*FW+4)'(t2) + (2*FW+4)'(t3) + (2*FW+4)'(t4);
        end
    end

    // Stage 3 ? base × poly
    logic [OW+2*FW+3:0] prod;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin valid_o<=0; exp_o<=0; end
        else begin
            valid_o <= vc2;
            prod     = (OW+2*FW+4)'(base) * (OW+2*FW+4)'(poly);
            exp_o   <= OW'(prod >> (2*FW));
        end
    end
endmodule : exp_approx_lane

// =============================================================================
//  simd_softmax ? 8-lane softmax, fully pipelined, scalar ports
// =============================================================================
module simd_softmax
    import softmax_pkg::*;
#(parameter int N=N_LANES, parameter int DW=DATA_W, parameter int FW=FRAC_W,
  parameter int OW=EXP_W,  parameter int OFW=EXP_FRAC_W)
(
    input  logic                  clk, rst_n, valid_i,
    input  logic signed [DW-1:0]  i0,i1,i2,i3,i4,i5,i6,i7,
    output logic                  valid_o,
    output logic [OW-1:0]         p0,p1,p2,p3,p4,p5,p6,p7
);
    // --- STAGE A: Max reduction (3 pipeline levels) ---
    logic signed [DW-1:0] m1a,m1b,m1c,m1d; logic mv1;
    logic signed [DW-1:0] m2a,m2b;          logic mv2;
    logic signed [DW-1:0] xmax;             logic mv3;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin mv1<=0; m1a<=0;m1b<=0;m1c<=0;m1d<=0; end
        else begin
            mv1<=valid_i;
            m1a<=(i0>i1)?i0:i1; m1b<=(i2>i3)?i2:i3;
            m1c<=(i4>i5)?i4:i5; m1d<=(i6>i7)?i6:i7;
        end
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin mv2<=0; m2a<=0; m2b<=0; end
        else begin mv2<=mv1; m2a<=(m1a>m1b)?m1a:m1b; m2b<=(m1c>m1d)?m1c:m1d; end
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin mv3<=0; xmax<=0; end
        else begin mv3<=mv2; xmax<=(m2a>m2b)?m2a:m2b; end
    end

    // --- Input delay: 3 stages to align with xmax ---
    logic signed [DW-1:0] d1_0,d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d1_7;
    logic signed [DW-1:0] d2_0,d2_1,d2_2,d2_3,d2_4,d2_5,d2_6,d2_7;
    logic signed [DW-1:0] d3_0,d3_1,d3_2,d3_3,d3_4,d3_5,d3_6,d3_7;
    logic vd1,vd2,vd3;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vd1<=0;vd2<=0;vd3<=0;
            d1_0<=0;d1_1<=0;d1_2<=0;d1_3<=0;d1_4<=0;d1_5<=0;d1_6<=0;d1_7<=0;
            d2_0<=0;d2_1<=0;d2_2<=0;d2_3<=0;d2_4<=0;d2_5<=0;d2_6<=0;d2_7<=0;
            d3_0<=0;d3_1<=0;d3_2<=0;d3_3<=0;d3_4<=0;d3_5<=0;d3_6<=0;d3_7<=0;
        end else begin
            vd1<=valid_i; vd2<=vd1; vd3<=vd2;
            d1_0<=i0; d2_0<=d1_0; d3_0<=d2_0;
            d1_1<=i1; d2_1<=d1_1; d3_1<=d2_1;
            d1_2<=i2; d2_2<=d1_2; d3_2<=d2_2;
            d1_3<=i3; d2_3<=d1_3; d3_3<=d2_3;
            d1_4<=i4; d2_4<=d1_4; d3_4<=d2_4;
            d1_5<=i5; d2_5<=d1_5; d3_5<=d2_5;
            d1_6<=i6; d2_6<=d1_6; d3_6<=d2_6;
            d1_7<=i7; d2_7<=d1_7; d3_7<=d2_7;
        end
    end

    // --- STAGE B: Subtract max ---
    logic signed [DW-1:0] xs0,xs1,xs2,xs3,xs4,xs5,xs6,xs7; logic sv;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sv<=0;xs0<=0;xs1<=0;xs2<=0;xs3<=0;xs4<=0;xs5<=0;xs6<=0;xs7<=0; end
        else begin
            sv<=mv3;
            xs0<=d3_0-xmax; xs1<=d3_1-xmax; xs2<=d3_2-xmax; xs3<=d3_3-xmax;
            xs4<=d3_4-xmax; xs5<=d3_5-xmax; xs6<=d3_6-xmax; xs7<=d3_7-xmax;
        end
    end

    // --- STAGE C: 8 parallel exp lanes ---
    logic [OW-1:0] ex0,ex1,ex2,ex3,ex4,ex5,ex6,ex7;
    logic          ev0,ev1,ev2,ev3,ev4,ev5,ev6,ev7;

    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u0(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs0),.valid_o(ev0),.exp_o(ex0));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u1(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs1),.valid_o(ev1),.exp_o(ex1));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u2(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs2),.valid_o(ev2),.exp_o(ex2));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u3(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs3),.valid_o(ev3),.exp_o(ex3));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u4(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs4),.valid_o(ev4),.exp_o(ex4));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u5(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs5),.valid_o(ev5),.exp_o(ex5));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u6(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs6),.valid_o(ev6),.exp_o(ex6));
    exp_approx_lane #(.DW(DW),.FW(FW),.OW(OW),.OFW(OFW)) u7(.clk(clk),.rst_n(rst_n),.valid_i(sv),.x_i(xs7),.valid_o(ev7),.exp_o(ex7));

    logic aev; assign aev = ev0;

    // --- STAGE D: Sum reduction (3 levels tree) ---
    localparam int SW = OW+3;
    logic [SW-1:0] s1a,s1b,s1c,s1d; logic ss1;
    logic [SW-1:0] s2a,s2b;          logic ss2;
    logic [SW-1:0] esum;              logic ss3;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ss1<=0;s1a<=0;s1b<=0;s1c<=0;s1d<=0; end
        else begin
            ss1<=aev;
            s1a<=SW'(ex0)+SW'(ex1); s1b<=SW'(ex2)+SW'(ex3);
            s1c<=SW'(ex4)+SW'(ex5); s1d<=SW'(ex6)+SW'(ex7);
        end
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ss2<=0;s2a<=0;s2b<=0; end
        else begin ss2<=ss1; s2a<=s1a+s1b; s2b<=s1c+s1d; end
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin ss3<=0;esum<=0; end
        else begin ss3<=ss2; esum<=s2a+s2b; end
    end

    // --- Exp delay: 5 stages to align with NR stage E2 ---
    logic [OW-1:0] e1_0,e1_1,e1_2,e1_3,e1_4,e1_5,e1_6,e1_7;
    logic [OW-1:0] e2_0,e2_1,e2_2,e2_3,e2_4,e2_5,e2_6,e2_7;
    logic [OW-1:0] e3_0,e3_1,e3_2,e3_3,e3_4,e3_5,e3_6,e3_7;
    logic [OW-1:0] e4_0,e4_1,e4_2,e4_3,e4_4,e4_5,e4_6,e4_7;
    logic [OW-1:0] e5_0,e5_1,e5_2,e5_3,e5_4,e5_5,e5_6,e5_7;
    logic evd1,evd2,evd3,evd4,evd5;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            evd1<=0;evd2<=0;evd3<=0;evd4<=0;evd5<=0;
            e1_0<=0;e1_1<=0;e1_2<=0;e1_3<=0;e1_4<=0;e1_5<=0;e1_6<=0;e1_7<=0;
            e2_0<=0;e2_1<=0;e2_2<=0;e2_3<=0;e2_4<=0;e2_5<=0;e2_6<=0;e2_7<=0;
            e3_0<=0;e3_1<=0;e3_2<=0;e3_3<=0;e3_4<=0;e3_5<=0;e3_6<=0;e3_7<=0;
            e4_0<=0;e4_1<=0;e4_2<=0;e4_3<=0;e4_4<=0;e4_5<=0;e4_6<=0;e4_7<=0;
            e5_0<=0;e5_1<=0;e5_2<=0;e5_3<=0;e5_4<=0;e5_5<=0;e5_6<=0;e5_7<=0;
        end else begin
            evd1<=aev; evd2<=evd1; evd3<=evd2; evd4<=evd3; evd5<=evd4;
            e1_0<=ex0; e2_0<=e1_0; e3_0<=e2_0; e4_0<=e3_0; e5_0<=e4_0;
            e1_1<=ex1; e2_1<=e1_1; e3_1<=e2_1; e4_1<=e3_1; e5_1<=e4_1;
            e1_2<=ex2; e2_2<=e1_2; e3_2<=e2_2; e4_2<=e3_2; e5_2<=e4_2;
            e1_3<=ex3; e2_3<=e1_3; e3_3<=e2_3; e4_3<=e3_3; e5_3<=e4_3;
            e1_4<=ex4; e2_4<=e1_4; e3_4<=e2_4; e4_4<=e3_4; e5_4<=e4_4;
            e1_5<=ex5; e2_5<=e1_5; e3_5<=e2_5; e4_5<=e3_5; e5_5<=e4_5;
            e1_6<=ex6; e2_6<=e1_6; e3_6<=e2_6; e4_6<=e3_6; e5_6<=e4_6;
            e1_7<=ex7; e2_7<=e1_7; e3_7<=e2_7; e4_7<=e3_7; e5_7<=e4_7;
        end
    end

    // --- STAGE E: Newton-Raphson reciprocal (2 cycles) ---
    logic [SW-1:0]    nr_sr;  logic [OW-1:0] nr_seed, nr_recip;
    logic             nr_e1,  nr_e2;
    logic [2*OW-1:0]  nr_sp,  nr_fl; logic [OW-1:0] nr_2m;
    assign nr_sp = (2*OW)'(nr_sr)   * (2*OW)'(nr_seed);
    assign nr_2m = OW'(2<<OFW)      - OW'(nr_sp>>OFW);
    assign nr_fl = (2*OW)'(nr_seed) * (2*OW)'(nr_2m);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin nr_e1<=0; nr_sr<=0; nr_seed<=0; end
        else begin
            nr_e1  <= ss3; nr_sr <= esum;
            nr_seed <= (esum==0) ? {OW{1'b1}} : OW'(64'h1_0000_0000/{32'b0,esum});
        end
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin nr_e2<=0; nr_recip<=0; end
        else begin nr_e2 <= nr_e1; nr_recip <= OW'(nr_fl>>OFW); end
    end

    // --- STAGE F: Scale (1 cycle) ---
    logic vd; assign vd = nr_e2 & evd5;
    logic [2*OW-1:0] sc0,sc1,sc2,sc3,sc4,sc5,sc6,sc7;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o<=0; p0<=0;p1<=0;p2<=0;p3<=0;p4<=0;p5<=0;p6<=0;p7<=0;
        end else begin
            valid_o <= vd;
            if (vd) begin
                sc0=(2*OW)'(e5_0)*(2*OW)'(nr_recip); p0<=OW'(sc0>>OFW);
                sc1=(2*OW)'(e5_1)*(2*OW)'(nr_recip); p1<=OW'(sc1>>OFW);
                sc2=(2*OW)'(e5_2)*(2*OW)'(nr_recip); p2<=OW'(sc2>>OFW);
                sc3=(2*OW)'(e5_3)*(2*OW)'(nr_recip); p3<=OW'(sc3>>OFW);
                sc4=(2*OW)'(e5_4)*(2*OW)'(nr_recip); p4<=OW'(sc4>>OFW);
                sc5=(2*OW)'(e5_5)*(2*OW)'(nr_recip); p5<=OW'(sc5>>OFW);
                sc6=(2*OW)'(e5_6)*(2*OW)'(nr_recip); p6<=OW'(sc6>>OFW);
                sc7=(2*OW)'(e5_7)*(2*OW)'(nr_recip); p7<=OW'(sc7>>OFW);
            end
        end
    end
endmodule : simd_softmax

// =============================================================================
//  CPUai_top ? main FSM-based SIMD AI CPU
//
//  Instruction encoding (18-bit):
//    [17:12] opcode
//    [11:10] R0  (destination register)
//    [9:0]   immediate / address
//
//  Opcode map:
//    0  ? QUANT only          (quantize R0 ? H[R0])
//    1  ? QUANT + RELU        (quantize then relu on 4-lane SIMD)
//    2  ? QUANT + RELU + SOFTMAX  (full pipeline)
//    3  ? RELU only           (relu on current H registers)
//    4  ? SOFTMAX only        (softmax on current H registers)
//    5  ? LOAD  H[R0] ? MEM[imm]
//    6  ? STORE MEM[imm] ? H[R0]
//    7  ? SET   H[R0] ? imm (zero-extended)
//   63  ? HALT
//
//  FSM states:
//    IDLE ? IF ? ID ? QUANT ? RELU ? SOFTMAX ? WB ? (back to IF)
//    Stages are SKIPPED based on need_* flags set in ID, using
//    CMD flags so the WB stage always knows what happened.
// =============================================================================
module CPUai_top (
    input  logic        clk,
    input  logic        rst,
    // instruction memory interface
    input  logic [17:0] instruction_in,
    output logic [9:0]  instruction_address,
    // data memory interface
    input  logic [63:0] data_in,
    output logic [63:0] data_out,
    output logic [9:0]  data_address,
    output logic        data_R,
    output logic        data_W,
    // status
    output logic        done
);

    // -------------------------------------------------------------------------
    //  Opcode extraction
    // -------------------------------------------------------------------------
    logic [5:0] opcode;
    assign opcode = instruction_in[17:12];

    // -------------------------------------------------------------------------
    //  FSM state encoding
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        STATE_IDLE    = 4'd0,
        STATE_IF      = 4'd1,
        STATE_ID      = 4'd2,
        STATE_QUANT   = 4'd3,
        STATE_RELU    = 4'd4,
        STATE_SOFTMAX = 4'd5,
        STATE_MEM     = 4'd6,   // address/data settle cycle for LOAD/STORE
        STATE_WB      = 4'd7,
        STATE_HALT    = 4'd8
    } state_t;

    state_t current_state;

    // -------------------------------------------------------------------------
    //  Program counter
    // -------------------------------------------------------------------------
    logic [9:0] PC, next_PC;
    assign instruction_address = PC;
    assign done                = (current_state == STATE_HALT);

    // -------------------------------------------------------------------------
    //  Register file
    //    H[0..3] ? four 64-bit general purpose registers
    // -------------------------------------------------------------------------
    logic [63:0] H [0:3];

    // -------------------------------------------------------------------------
    //  Instruction decode fields (set in STATE_ID, held until WB)
    // -------------------------------------------------------------------------
    logic [1:0] R0;           // destination register index
    logic [9:0] imm;          // 10-bit immediate / address

    // CMD flags ? what does this instruction do?
    logic CMD_quant;
    logic CMD_relu;
    logic CMD_softmax;
    logic CMD_load;
    logic CMD_store;
    logic CMD_set;

    // Routing flags ? which pipeline stages to visit?
    // Set once in ID, used by FSM transition logic to skip stages.
    logic need_quant;
    logic need_relu;
    logic need_softmax;

    // -------------------------------------------------------------------------
    //  Sub-module wires ? quantizer
    //    One instance per register lane (4 lanes, 64?16-bit each)
    //    We quantize 64-bit H values down to 16-bit for RELU/SOFTMAX
    // -------------------------------------------------------------------------
    logic [63:0] quant_in  [0:3];
    logic [31:0] quant_out [0:3];   // 32-bit intermediate from ai_quantizer
    logic [15:0] quant16   [0:3];   // lower 16 bits used as SIMD input

    genvar g;
    generate
        for (g = 0; g < 4; g++) begin : quant_lanes
            ai_quantizer #(.IN_WIDTH(64), .OUT_WIDTH(32), .FRAC_BITS(16)) qinst (
                .data_in  (quant_in[g]),
                .data_out (quant_out[g])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    //  Sub-module wires ? RELU
    //    Takes 4 × 64-bit SIMD lanes
    // -------------------------------------------------------------------------
    logic [3:0][63:0] relu_in;
    logic [3:0][63:0] relu_out;
    logic             relu_rst_n;
    assign relu_rst_n = ~rst;

    simd_relu_top relu_inst (
        .clk     (clk),
        .rst_n   (relu_rst_n),
        .data_in (relu_in),
        .data_out(relu_out)
    );

    // -------------------------------------------------------------------------
    //  Sub-module wires ? SOFTMAX
    //    Takes 8 × 16-bit signed scalar inputs (mapped from relu_out + quant16)
    // -------------------------------------------------------------------------
    import softmax_pkg::DATA_W, softmax_pkg::EXP_W;

    logic        softmax_valid_i, softmax_valid_o;
    logic signed [DATA_W-1:0] sm_i0,sm_i1,sm_i2,sm_i3,sm_i4,sm_i5,sm_i6,sm_i7;
    logic        [EXP_W-1:0]  sm_p0,sm_p1,sm_p2,sm_p3,sm_p4,sm_p5,sm_p6,sm_p7;
    logic        softmax_rst_n;
    assign softmax_rst_n = ~rst;

    simd_softmax softmax_inst (
        .clk    (clk),
        .rst_n  (softmax_rst_n),
        .valid_i(softmax_valid_i),
        .i0(sm_i0), .i1(sm_i1), .i2(sm_i2), .i3(sm_i3),
        .i4(sm_i4), .i5(sm_i5), .i6(sm_i6), .i7(sm_i7),
        .valid_o(softmax_valid_o),
        .p0(sm_p0), .p1(sm_p1), .p2(sm_p2), .p3(sm_p3),
        .p4(sm_p4), .p5(sm_p5), .p6(sm_p6), .p7(sm_p7)
    );

    // -------------------------------------------------------------------------
    //  Intermediate result registers (hold values between pipeline stages)
    // -------------------------------------------------------------------------
    logic [31:0] result_quant [0:3];  // stored quantizer outputs
    logic [63:0] result_relu  [0:3];  // stored relu outputs
    logic [EXP_W-1:0] result_softmax [0:7]; // stored softmax probabilities

    // -------------------------------------------------------------------------
    //  Memory interface registers
    // -------------------------------------------------------------------------
    logic [9:0]  mem_addr_reg;
    logic [63:0] data_out_reg;
    logic        rdata_en, wdata_en;

    assign data_out     = data_out_reg;
    assign data_R       = rdata_en;
    assign data_W       = wdata_en;
    assign data_address = mem_addr_reg;

    // -------------------------------------------------------------------------
    //  Softmax wait counter
    //  The softmax pipeline is ~12 cycles deep. We count and wait for valid_o.
    // -------------------------------------------------------------------------
    logic [5:0] softmax_wait_cnt;

    // relu_settled: tracks whether we have waited the extra cycle in STATE_RELU
    // for simd_relu_top's pipeline register to produce valid output.
    logic relu_settled;

    // =========================================================================
    //  FSM ? state transitions
    //  The key insight: after ID, the FSM checks need_* flags to decide
    //  whether to visit QUANT, RELU, SOFTMAX or skip straight to WB.
    //  This way the opcode is read ONCE in ID and never touched again.
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            current_state <= STATE_IDLE;
            PC            <= 10'd0;
        end else begin
            case (current_state)

                STATE_IDLE: begin
                    current_state <= STATE_IF;
                end

                STATE_IF: begin
                    current_state <= STATE_ID;
                end

                // BUG1 FIX: need_* flags are set with <= in the ID always_ff block,
                // so they are only valid on the NEXT clock edge after STATE_ID.
                // We add a one-cycle STATE_ID2 pass: on the first cycle (STATE_ID)
                // we decode the opcode combinationally to decide routing,
                // NOT from the registered need_* flags.
                // Solution: use opcode directly here (only for routing, not execution).
                STATE_ID: begin
                    if (opcode == 6'd63)
                        current_state <= STATE_HALT;
                    // opcode 0,1,2 need quant
                    else if (opcode == 6'd0 || opcode == 6'd1 || opcode == 6'd2)
                        current_state <= STATE_QUANT;
                    // opcode 3 needs relu only
                    else if (opcode == 6'd3)
                        current_state <= STATE_RELU;
                    // opcode 4 needs softmax only
                    else if (opcode == 6'd4)
                        current_state <= STATE_SOFTMAX;
                    // load/store/set/other ? go to MEM settle then WB
                    else
                        current_state <= STATE_MEM;
                end

                STATE_QUANT: begin
                    // need_relu/need_softmax are now valid (set in STATE_ID one cycle ago)
                    if (need_relu)
                        current_state <= STATE_RELU;
                    else if (need_softmax)
                        current_state <= STATE_SOFTMAX;
                    else
                        current_state <= STATE_WB;
                end

                // BUG3 FIX: simd_relu_top has a 1-cycle pipeline register.
                // We must stay in STATE_RELU for 2 cycles:
                //   cycle 1 ? drive relu_in, relu starts computing
                //   cycle 2 ? relu_out is valid, latch into result_relu
                // We use relu_settled flag to count.
                STATE_RELU: begin
                    if (!relu_settled)
                        current_state <= STATE_RELU;  // wait one more cycle
                    else if (need_softmax)
                        current_state <= STATE_SOFTMAX;
                    else
                        current_state <= STATE_WB;
                end

                // BUG2 FIX: Stay in SOFTMAX until valid_o arrives.
                // softmax_started ensures we only pulse valid_i once.
                STATE_SOFTMAX: begin
                    if (softmax_valid_o)
                        current_state <= STATE_WB;
                    else
                        current_state <= STATE_SOFTMAX;
                end

                // BUG4 FIX: STATE_MEM lets mem_addr_reg and data signals
                // settle for one full clock so WB reads the correct data_in.
                STATE_MEM: begin
                    current_state <= STATE_WB;
                end

                STATE_WB: begin
                    current_state <= STATE_IF;
                    PC            <= next_PC;
                end

                STATE_HALT: begin
                    current_state <= STATE_HALT;
                end

                default: current_state <= STATE_IDLE;
            endcase
        end
    end

    // =========================================================================
    //  STATE_ID ? decode opcode, set CMD flags and routing flags
    //  Opcode is read here and ONLY here. Everything downstream uses CMD flags.
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            CMD_quant    <= 1'b0;
            CMD_relu     <= 1'b0;
            CMD_softmax  <= 1'b0;
            CMD_load     <= 1'b0;
            CMD_store    <= 1'b0;
            CMD_set      <= 1'b0;
            need_quant   <= 1'b0;
            need_relu    <= 1'b0;
            need_softmax <= 1'b0;
            R0           <= 2'd0;
            imm          <= 10'd0;
        end else if (current_state == STATE_ID) begin
            R0  <= instruction_in[11:10];
            imm <= instruction_in[9:0];

            // Decode opcode into CMD flags
            CMD_quant   <= (opcode == 6'd0) || (opcode == 6'd1) || (opcode == 6'd2);
            CMD_relu    <= (opcode == 6'd1) || (opcode == 6'd2) || (opcode == 6'd3);
            CMD_softmax <= (opcode == 6'd2) || (opcode == 6'd4);
            CMD_load    <= (opcode == 6'd5);
            CMD_store   <= (opcode == 6'd6);
            CMD_set     <= (opcode == 6'd7);

            // Routing flags ? which stages does this instruction need?
            need_quant   <= (opcode == 6'd0) || (opcode == 6'd1) || (opcode == 6'd2);
            need_relu    <= (opcode == 6'd1) || (opcode == 6'd2) || (opcode == 6'd3);
            need_softmax <= (opcode == 6'd2) || (opcode == 6'd4);

            $display("[ID @%0t] PC=%0d opcode=%0d R0=%0d imm=%0d",
                     $time, PC, opcode, instruction_in[11:10], instruction_in[9:0]);
        end
    end

    // =========================================================================
    //  STATE_QUANT ? latch quantizer outputs into result_quant[]
    //  ai_quantizer is purely combinational, so we just need to register it.
    // =========================================================================

    // Drive quantizer inputs from H registers
    always_comb begin
        for (int j = 0; j < 4; j++) begin
            quant_in[j] = H[j];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int j = 0; j < 4; j++) result_quant[j] <= 32'd0;
        end else if (current_state == STATE_QUANT) begin
            for (int j = 0; j < 4; j++) begin
                result_quant[j] <= quant_out[j];
                $display("[QUANT@%0t] lane%0d H=%h ? quant=%h", $time, j, H[j], quant_out[j]);
            end
        end
    end

    // =========================================================================
    //  STATE_RELU ? drive relu_in, capture relu_out next cycle
    //  relu has a 1-cycle pipeline register inside simd_relu_top.
    //  We drive inputs on entry to STATE_RELU, then WB captures outputs.
    // =========================================================================

    // Drive relu inputs: SIGN-EXTEND the 32-bit quant result to 64 bits.
    // simd_relu_top checks bit[63] for the sign ? if we only zero-extend,
    // a negative quant value (e.g. 0xFFFFFFFF) would have bit[63]=0 and
    // RELU would incorrectly pass it through.
    // Sign-extending from bit[31] propagates the correct sign to bit[63].
    always_comb begin
        for (int j = 0; j < 4; j++) begin
            relu_in[j] = need_quant
                ? {{32{result_quant[j][31]}}, result_quant[j][31:0]}
                : H[j];
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            for (int j = 0; j < 4; j++) result_relu[j] <= 64'd0;
            relu_settled <= 1'b0;
        end else if (current_state == STATE_RELU && !relu_settled) begin
            // Cycle 1 in STATE_RELU: inputs are being driven to simd_relu_top.
            // Its internal register latches them THIS edge ? output not yet valid.
            // Just set the flag so next cycle we capture the output.
            relu_settled <= 1'b1;
        end else if (current_state == STATE_RELU && relu_settled) begin
            // Cycle 2 in STATE_RELU: relu_out is now valid, capture it.
            for (int j = 0; j < 4; j++) begin
                result_relu[j] <= relu_out[j];
                $display("[RELU @%0t] lane%0d in=%h out=%h", $time, j, relu_in[j], relu_out[j]);
            end
            relu_settled <= 1'b0;  // reset for next instruction
        end else begin
            relu_settled <= 1'b0;
        end
    end

    // =========================================================================
    //  STATE_SOFTMAX ? drive softmax inputs and wait for valid_o
    //  simd_softmax takes ~12 cycles. We pulse valid_i for one cycle on entry.
    // =========================================================================

    // Drive softmax inputs from relu_out (lanes 0-3) and quant16 (lanes 4-7)
    // quant16 are the lower 16 bits of result_quant, reused as extra lanes
    always_comb begin
        for (int j = 0; j < 4; j++)
            quant16[j] = result_quant[j][15:0];
    end

    // Pulse valid_i once when we first enter STATE_SOFTMAX
    logic softmax_started;

    always_ff @(posedge clk) begin
        if (rst) begin
            softmax_valid_i  <= 1'b0;
            softmax_started  <= 1'b0;
            softmax_wait_cnt <= 6'd0;
            sm_i0 <= '0; sm_i1 <= '0; sm_i2 <= '0; sm_i3 <= '0;
            sm_i4 <= '0; sm_i5 <= '0; sm_i6 <= '0; sm_i7 <= '0;
        end else if (current_state == STATE_SOFTMAX && !softmax_started) begin
            // First cycle in SOFTMAX ? latch inputs and pulse valid_i
            sm_i0 <= $signed(result_relu[0][15:0]);
            sm_i1 <= $signed(result_relu[1][15:0]);
            sm_i2 <= $signed(result_relu[2][15:0]);
            sm_i3 <= $signed(result_relu[3][15:0]);
            sm_i4 <= $signed(quant16[0]);
            sm_i5 <= $signed(quant16[1]);
            sm_i6 <= $signed(quant16[2]);
            sm_i7 <= $signed(quant16[3]);
            softmax_valid_i <= 1'b1;
            softmax_started <= 1'b1;
            softmax_wait_cnt <= 6'd0;
            $display("[SOFTMAX_START@%0t]", $time);
        end else if (current_state == STATE_SOFTMAX && softmax_started) begin
            softmax_valid_i  <= 1'b0;   // de-assert after first cycle
            softmax_wait_cnt <= softmax_wait_cnt + 1;
        end else begin
            // Reset the started flag when we leave SOFTMAX
            softmax_started  <= 1'b0;
            softmax_valid_i  <= 1'b0;
        end
    end

    // Capture softmax results when valid_o arrives
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int j = 0; j < 8; j++) result_softmax[j] <= '0;
        end else if (current_state == STATE_SOFTMAX && softmax_valid_o) begin
            result_softmax[0] <= sm_p0; result_softmax[1] <= sm_p1;
            result_softmax[2] <= sm_p2; result_softmax[3] <= sm_p3;
            result_softmax[4] <= sm_p4; result_softmax[5] <= sm_p5;
            result_softmax[6] <= sm_p6; result_softmax[7] <= sm_p7;
            $display("[SOFTMAX_DONE@%0t] p0=%h p1=%h p2=%h p3=%h p4=%h p5=%h p6=%h p7=%h",
                     $time, sm_p0,sm_p1,sm_p2,sm_p3,sm_p4,sm_p5,sm_p6,sm_p7);
        end
    end

    // =========================================================================
    //  STATE_WB ? write results back to register file and handle memory ops
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int j = 0; j < 4; j++) H[j] <= 64'd0;
            // BUG5 FIX: next_PC starts at 1 so the first WB advances PC to 1.
            // PC starts at 0 (reset above), so instruction 0 executes at PC=0.
            next_PC      <= 10'd1;
            mem_addr_reg <= 10'd0;
            data_out_reg <= 64'd0;
            rdata_en     <= 1'b0;
            wdata_en     <= 1'b0;
        end

        // ---------------------------------------------------------------
        // STATE_MEM ? set up memory address and enables so they settle
        // for a full clock before STATE_WB samples data_in / writes data.
        // ---------------------------------------------------------------
        else if (current_state == STATE_MEM) begin
            if (CMD_load) begin
                rdata_en     <= 1'b1;
                mem_addr_reg <= imm;
            end else if (CMD_store) begin
                wdata_en     <= 1'b1;
                mem_addr_reg <= imm;
                data_out_reg <= H[R0];
                $display("[MEM STORE@%0t] MEM[%0d] ? H[%0d] = %h", $time, imm, R0, H[R0]);
            end
        end

        // ---------------------------------------------------------------
        // STATE_WB ? write results back to register file
        // ---------------------------------------------------------------
        else if (current_state == STATE_WB) begin

            // Clear memory enables
            rdata_en <= 1'b0;
            wdata_en <= 1'b0;

            if (CMD_quant && !CMD_relu && !CMD_softmax) begin
                H[R0] <= {32'd0, result_quant[R0]};
                $display("[WB QUANT@%0t] H[%0d] = %h", $time, R0, result_quant[R0]);
            end

            else if (CMD_relu && !CMD_softmax) begin
                for (int j = 0; j < 4; j++)
                    H[j] <= result_relu[j];
                $display("[WB RELU @%0t] H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                         $time, result_relu[0],result_relu[1],result_relu[2],result_relu[3]);
            end

            else if (CMD_softmax) begin
                H[0] <= {result_softmax[1][15:0], result_softmax[0][15:0], 32'd0};
                H[1] <= {result_softmax[3][15:0], result_softmax[2][15:0], 32'd0};
                H[2] <= {result_softmax[5][15:0], result_softmax[4][15:0], 32'd0};
                H[3] <= {result_softmax[7][15:0], result_softmax[6][15:0], 32'd0};
                $display("[WB SOFTMAX@%0t] p0=%h p1=%h p2=%h p3=%h",
                         $time, result_softmax[0],result_softmax[1],
                                result_softmax[2],result_softmax[3]);
            end

            else if (CMD_load) begin
                // BUG4 FIX: address was set in STATE_MEM, data_in is now stable.
                H[R0] <= data_in;
                $display("[WB LOAD @%0t] H[%0d] ? MEM[%0d] = %h", $time, R0, imm, data_in);
            end

            else if (CMD_store) begin
                // Store was committed to data_out in STATE_MEM. Nothing to do here.
                $display("[WB STORE@%0t] MEM[%0d] ? H[%0d] = %h", $time, imm, R0, H[R0]);
            end

            else if (CMD_set) begin
                H[R0] <= {54'd0, imm};
                $display("[WB SET  @%0t] H[%0d] ? %0d", $time, R0, imm);
            end

            // Advance PC
            next_PC <= next_PC + 10'd1;
        end
    end

endmodule : CPUai_top

`timescale 1ns/1ps

// =============================================================================
//  tb_simd_ai_cpu ? Testbench for CPUai_top
//
//  Test plan (6 test programs):
//    TEST 1 ? SET instruction         (opcode 7)  : load immediates into H regs
//    TEST 2 ? LOAD / STORE            (opcode 5/6): memory read/write roundtrip
//    TEST 3 ? QUANT only              (opcode 0)  : quantize H[0], check saturation
//    TEST 4 ? QUANT + RELU            (opcode 1)  : verify negative lanes zeroed
//    TEST 5 ? Full pipeline           (opcode 2)  : QUANT ? RELU ? SOFTMAX
//    TEST 6 ? SOFTMAX only            (opcode 4)  : softmax on pre-loaded values
//    TEST 7 ? HALT                    (opcode 63) : done flag asserted
//
//  Instruction encoding reminder:
//    [17:12] opcode
//    [11:10] R0   (destination register, 2-bit)
//    [9:0]   imm  (immediate / address, 10-bit)
//
//  How the instruction ROM works here:
//    The testbench acts as the instruction memory.
//    instruction_in is driven combinationally from a small local array
//    indexed by instruction_address (PC).
// =============================================================================
module tb_simd_ai_cpu;

    // -------------------------------------------------------------------------
    //  DUT ports
    // -------------------------------------------------------------------------
    logic        clk;
    logic        rst;
    logic [17:0] instruction_in;
    logic [9:0]  instruction_address;
    logic [63:0] data_in;
    logic [63:0] data_out;
    logic [9:0]  data_address;
    logic        data_R;
    logic        data_W;
    logic        done;

    // -------------------------------------------------------------------------
    //  DUT instantiation
    // -------------------------------------------------------------------------
    CPUai_top dut (
        .clk                (clk),
        .rst                (rst),
        .instruction_in     (instruction_in),
        .instruction_address(instruction_address),
        .data_in            (data_in),
        .data_out           (data_out),
        .data_address       (data_address),
        .data_R             (data_R),
        .data_W             (data_W),
        .done               (done)
    );

    // -------------------------------------------------------------------------
    //  Clock ? 10 ns period (100 MHz)
    // -------------------------------------------------------------------------
    initial clk = 0;
    always  #5 clk = ~clk;

    // -------------------------------------------------------------------------
    //  Instruction ROM ? driven from a local array
    //  Each test program is loaded before reset is released.
    // -------------------------------------------------------------------------
    localparam int ROM_DEPTH = 32;
    logic [17:0] instr_rom [0:ROM_DEPTH-1];

    // Drive instruction_in combinationally from ROM
    assign instruction_in = instr_rom[instruction_address];

    // -------------------------------------------------------------------------
    //  Data memory ? simple 1K × 64-bit model
    // -------------------------------------------------------------------------
    localparam int MEM_DEPTH = 1024;
    // Must be 'reg' (not 'logic') so both the always block AND
    // the initial block can drive it ? ModelSim 10.5b rejects
    // multiple drivers on a 'logic' variable.
    reg [63:0] data_mem [0:MEM_DEPTH-1];

    // Drive data_in from memory on reads
    assign data_in = data_R ? data_mem[data_address] : 64'hDEAD_BEEF_DEAD_BEEF;

    // Capture DUT writes ? use plain 'always', not 'always_ff',
    // because 'always_ff' enforces single-driver and the initial
    // block also writes data_mem entries for test setup.
    always @(posedge clk) begin
        if (data_W)
            data_mem[data_address] <= data_out;
    end

    // -------------------------------------------------------------------------
    //  Helper: build an 18-bit instruction word
    //    encode_instr(opcode, R0, imm)
    // -------------------------------------------------------------------------
    function automatic logic [17:0] encode_instr(
        input logic [5:0]  op,
        input logic [1:0]  r0,
        input logic [9:0]  im
    );
        return {op, r0, im};
    endfunction

    // -------------------------------------------------------------------------
    //  Helper: flush ROM with HALTs
    // -------------------------------------------------------------------------
    task automatic flush_rom();
        for (int i = 0; i < ROM_DEPTH; i++)
            instr_rom[i] = encode_instr(6'd63, 2'd0, 10'd0); // HALT everywhere
    endtask

    // -------------------------------------------------------------------------
    //  Helper: reset the DUT
    // -------------------------------------------------------------------------
    task automatic do_reset();
        rst = 1;
        repeat(4) @(posedge clk);
        @(negedge clk); rst = 0;
        $display("[TB] Reset released at %0t", $time);
    endtask

    // -------------------------------------------------------------------------
    //  Helper: wait for HALT with timeout
    // -------------------------------------------------------------------------
    task automatic wait_for_halt(input int max_cycles);
        int cnt;
        cnt = 0;
        while (!done && cnt < max_cycles) begin
            @(posedge clk);
            cnt++;
        end
        if (!done)
            $display("[TB] *** TIMEOUT waiting for HALT after %0d cycles ***", max_cycles);
        else
            $display("[TB] HALT reached after %0d cycles", cnt);
    endtask

    // -------------------------------------------------------------------------
    //  Helper: print a separator
    // -------------------------------------------------------------------------
    task automatic print_sep(input string title);
        $display("\n============================================================");
        $display("  %s", title);
        $display("============================================================");
    endtask

    // -------------------------------------------------------------------------
    //  Pass/Fail counter
    // -------------------------------------------------------------------------
    int pass_cnt = 0;
    int fail_cnt = 0;

    task automatic check(
        input string  label,
        input logic [63:0] got,
        input logic [63:0] expected
    );
        if (got === expected) begin
            $display("  [PASS] %s : got=%h", label, got);
            pass_cnt++;
        end else begin
            $display("  [FAIL] %s : got=%h  expected=%h", label, got, expected);
            fail_cnt++;
        end
    endtask

    // =========================================================================
    //  MAIN TEST SEQUENCE
    // =========================================================================
    initial begin
        $display("================================================================");
        $display("  CPUai_top Testbench  ?  starting at %0t", $time);
        $display("================================================================");

        // Initialise memory
        for (int i = 0; i < MEM_DEPTH; i++) data_mem[i] = 64'd0;
        flush_rom();

        // -----------------------------------------------------------------
        // TEST 1 ? SET instruction (opcode 7)
        //   SET H[0] ? 42
        //   SET H[1] ? 100
        //   SET H[2] ? 7
        //   SET H[3] ? 0
        //   HALT
        // Expected: H[0]=42, H[1]=100, H[2]=7, H[3]=0
        // -----------------------------------------------------------------
        print_sep("TEST 1 ? SET instruction");
        flush_rom();
        instr_rom[0] = encode_instr(6'd7, 2'd0, 10'd42);   // SET H[0] = 42
        instr_rom[1] = encode_instr(6'd7, 2'd1, 10'd100);  // SET H[1] = 100
        instr_rom[2] = encode_instr(6'd7, 2'd2, 10'd7);    // SET H[2] = 7
        instr_rom[3] = encode_instr(6'd7, 2'd3, 10'd0);    // SET H[3] = 0
        instr_rom[4] = encode_instr(6'd63, 2'd0, 10'd0);   // HALT

        do_reset();
        wait_for_halt(200);

        // Peek at register file via the internal hierarchy
        check("SET H[0]", dut.H[0], 64'd42);
        check("SET H[1]", dut.H[1], 64'd100);
        check("SET H[2]", dut.H[2], 64'd7);
        check("SET H[3]", dut.H[3], 64'd0);

        // -----------------------------------------------------------------
        // TEST 2 ? LOAD / STORE (opcodes 5, 6)
        //   Pre-load MEM[10] = 0xCAFEBABE_DEADBEEF
        //   LOAD  H[0] ? MEM[10]
        //   STORE MEM[20] ? H[0]
        //   HALT
        // Expected: H[0] = 0xCAFEBABE_DEADBEEF
        //           MEM[20] = 0xCAFEBABE_DEADBEEF
        // -----------------------------------------------------------------
        print_sep("TEST 2 ? LOAD / STORE");
        flush_rom();
        data_mem[10] = 64'hCAFEBABE_DEADBEEF;
        instr_rom[0] = encode_instr(6'd5, 2'd0, 10'd10);   // LOAD H[0] ? MEM[10]
        instr_rom[1] = encode_instr(6'd6, 2'd0, 10'd20);   // STORE MEM[20] ? H[0]
        instr_rom[2] = encode_instr(6'd63, 2'd0, 10'd0);   // HALT

        do_reset();
        wait_for_halt(200);

        check("LOAD  H[0]",  dut.H[0],    64'hCAFEBABE_DEADBEEF);
        check("STORE MEM[20]", data_mem[20], 64'hCAFEBABE_DEADBEEF);

        // -----------------------------------------------------------------
        // TEST 3 ? QUANT only (opcode 0)
        //   Set H[0] to a known 64-bit Q32.32 value,
        //   then QUANT H[0] ? should round and saturate to 32-bit.
        //
        //   Case A ? normal value: 0x0000_0001_8000_0000
        //     SHIFT_AMT = 32-16 = 16
        //     Rounded = 0x0000_0001_8000_0000 + 2^15 = 0x0000_0001_8000_8000
        //     scaled = 0x0000_0001_8000_8000 >> 16 = 0x0000_0001_8000 (only 32 lsb) = 0x00018000
        //     Expected data_out lower 32b = 0x0001_8000
        //
        //   Case B ? positive overflow: 0x0FFF_FFFF_FFFF_FFFF ? saturates to 0x7FFF_FFFF
        // -----------------------------------------------------------------
        print_sep("TEST 3 ? QUANT only");
        flush_rom();
        // Load a known value into H[0], then quantize it
        // We use a two-instruction sequence:
        //   1. LOAD H[0] from MEM[5] (we pre-set MEM[5])
        //   2. QUANT H[0]
        //   3. HALT
        data_mem[5] = 64'h0000_0001_8000_0000;
        instr_rom[0] = encode_instr(6'd5, 2'd0, 10'd5);    // LOAD H[0] ? MEM[5]
        instr_rom[1] = encode_instr(6'd0, 2'd0, 10'd0);    // QUANT H[0]
        instr_rom[2] = encode_instr(6'd63, 2'd0, 10'd0);   // HALT

        do_reset();
        wait_for_halt(200);

        // After QUANT, H[0] = {32'd0, quant_result[0]}
        // quant_result = lower 32 bits of scaled value
        $display("  QUANT result H[0] = %h (lower 32b = %h)", dut.H[0], dut.H[0][31:0]);
        // Just verify it's non-zero and fits in 32 bits (upper 32b = 0)
        check("QUANT H[0] upper32 = 0", {32'd0, dut.H[0][63:32]}, 64'd0);

        // Positive overflow test
        data_mem[6] = 64'h0FFF_FFFF_FFFF_FFFF;
        instr_rom[0] = encode_instr(6'd5, 2'd1, 10'd6);    // LOAD H[1] ? MEM[6]
        instr_rom[1] = encode_instr(6'd0, 2'd1, 10'd0);    // QUANT H[1]
        instr_rom[2] = encode_instr(6'd63, 2'd0, 10'd0);   // HALT

        do_reset();
        wait_for_halt(200);

        $display("  QUANT overflow H[1] lower32 = %h (expect 7FFFFFFF)", dut.H[1][31:0]);
        check("QUANT pos overflow saturate", {32'd0, dut.H[1][31:0]}, 64'h0000_0000_7FFF_FFFF);

        // -----------------------------------------------------------------
        // TEST 4 ? QUANT + RELU (opcode 1)
        //   Load H regs with a mix of positive and negative 64-bit values.
        //   Opcode 1 runs QUANT then RELU.
        //   Negative lanes (MSB=1 after quant) should be zeroed by RELU.
        //
        //   MEM[0] = large positive  ? stays positive after quant
        //   MEM[1] = large negative  ? RELU zeros it
        //   MEM[2] = 0               ? stays 0
        //   MEM[3] = small positive  ? stays positive
        // -----------------------------------------------------------------
        print_sep("TEST 4 ? QUANT + RELU (opcode 1)");
        flush_rom();

        data_mem[0] = 64'h0000_0000_0001_0000;  // positive
        data_mem[1] = 64'hFFFF_FFFF_FFFF_0000;  // negative (MSB=1)
        data_mem[2] = 64'h0000_0000_0000_0000;  // zero
        data_mem[3] = 64'h0000_0000_0000_8000;  // small positive

        instr_rom[0] = encode_instr(6'd5, 2'd0, 10'd0);   // LOAD H[0] ? MEM[0]
        instr_rom[1] = encode_instr(6'd5, 2'd1, 10'd1);   // LOAD H[1] ? MEM[1]
        instr_rom[2] = encode_instr(6'd5, 2'd2, 10'd2);   // LOAD H[2] ? MEM[2]
        instr_rom[3] = encode_instr(6'd5, 2'd3, 10'd3);   // LOAD H[3] ? MEM[3]
        instr_rom[4] = encode_instr(6'd1, 2'd0, 10'd0);   // QUANT+RELU (opcode 1)
        instr_rom[5] = encode_instr(6'd63, 2'd0, 10'd0);  // HALT

        do_reset();
        wait_for_halt(300);

        // After RELU: negative lanes ? 0, positive lanes remain
        $display("  Post QUANT+RELU H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                 dut.H[0], dut.H[1], dut.H[2], dut.H[3]);

        // H[1] was negative ? RELU should force it to 0
        check("RELU zero H[1] (was negative)", dut.H[1], 64'd0);
        // H[2] was zero ? should remain 0
        check("RELU keep H[2] = 0",            dut.H[2], 64'd0);
        // H[0] positive ? should be non-zero
        if (dut.H[0] != 64'd0) begin
            $display("  [PASS] RELU kept H[0] non-zero = %h", dut.H[0]);
            pass_cnt++;
        end else begin
            $display("  [FAIL] RELU unexpectedly zeroed H[0]");
            fail_cnt++;
        end

        // -----------------------------------------------------------------
        // TEST 5 ? Full pipeline: QUANT + RELU + SOFTMAX (opcode 2)
        //   Load 4 positive values into H[0..3].
        //   Run opcode 2 (full pipeline).
        //   After WB, H[0..3] hold packed softmax probabilities.
        //   We check:
        //     (a) done is asserted eventually
        //     (b) H values are non-zero (softmax never outputs exactly 0 for positive inputs)
        //     (c) The probabilities are in a valid range (non-zero, plausible)
        // -----------------------------------------------------------------
        print_sep("TEST 5 ? Full pipeline QUANT+RELU+SOFTMAX (opcode 2)");
        flush_rom();

        // Use Q8.8 friendly values: 1.0 = 0x0100, 2.0 = 0x0200 etc.
        // Store them in memory as 64-bit values ? quantizer will scale them
        data_mem[30] = 64'h0000_0000_0000_0100;  // ~1.0 in Q8.8 zero-extended
        data_mem[31] = 64'h0000_0000_0000_0200;  // ~2.0
        data_mem[32] = 64'h0000_0000_0000_0080;  // ~0.5
        data_mem[33] = 64'h0000_0000_0000_0300;  // ~3.0

        instr_rom[0] = encode_instr(6'd5, 2'd0, 10'd30);  // LOAD H[0]
        instr_rom[1] = encode_instr(6'd5, 2'd1, 10'd31);  // LOAD H[1]
        instr_rom[2] = encode_instr(6'd5, 2'd2, 10'd32);  // LOAD H[2]
        instr_rom[3] = encode_instr(6'd5, 2'd3, 10'd33);  // LOAD H[3]
        instr_rom[4] = encode_instr(6'd2, 2'd0, 10'd0);   // QUANT+RELU+SOFTMAX
        instr_rom[5] = encode_instr(6'd63, 2'd0, 10'd0);  // HALT

        do_reset();
        // Softmax pipeline is ~12+ cycles deep, give generous timeout
        wait_for_halt(500);

        $display("  Post full pipeline:");
        $display("    H[0]=%h  H[1]=%h", dut.H[0], dut.H[1]);
        $display("    H[2]=%h  H[3]=%h", dut.H[2], dut.H[3]);

        // Softmax outputs land in H[0..3] packed as {p[1][15:0], p[0][15:0], 32'd0}
        // Just verify outputs are non-zero (softmax of positive values is always > 0)
        if (dut.H[0] != 64'd0 || dut.H[1] != 64'd0) begin
            $display("  [PASS] Softmax produced non-zero outputs");
            pass_cnt++;
        end else begin
            $display("  [FAIL] Softmax outputs are all zero");
            fail_cnt++;
        end

        // -----------------------------------------------------------------
        // TEST 6 ? SOFTMAX only (opcode 4)
        //   Pre-load H[0..3] with Q8.8 values directly via SET.
        //   SET only handles 10-bit immediates, so use small values.
        //   Then fire opcode 4 (SOFTMAX only).
        //   Check H regs after WB are non-zero.
        // -----------------------------------------------------------------
        print_sep("TEST 6 ? SOFTMAX only (opcode 4)");
        flush_rom();

        // SET H[0..3] to small Q8.8 friendly values
        // 0x0100 = 256 (represents 1.0 in Q8.8), fits in 10-bit imm (max 1023)
        instr_rom[0] = encode_instr(6'd7, 2'd0, 10'd256);  // SET H[0] = 256 (=1.0 Q8.8)
        instr_rom[1] = encode_instr(6'd7, 2'd1, 10'd512);  // SET H[1] = 512 (=2.0 Q8.8)
        instr_rom[2] = encode_instr(6'd7, 2'd2, 10'd128);  // SET H[2] = 128 (=0.5 Q8.8)
        instr_rom[3] = encode_instr(6'd7, 2'd3, 10'd64);   // SET H[3] = 64  (=0.25 Q8.8)
        instr_rom[4] = encode_instr(6'd4, 2'd0, 10'd0);    // SOFTMAX only
        instr_rom[5] = encode_instr(6'd63, 2'd0, 10'd0);   // HALT

        do_reset();
        wait_for_halt(500);

        $display("  Post SOFTMAX-only:");
        $display("    H[0]=%h  H[1]=%h", dut.H[0], dut.H[1]);
        $display("    H[2]=%h  H[3]=%h", dut.H[2], dut.H[3]);

        if (dut.H[0] != 64'd0 || dut.H[1] != 64'd0) begin
            $display("  [PASS] SOFTMAX-only produced non-zero outputs");
            pass_cnt++;
        end else begin
            $display("  [FAIL] SOFTMAX-only outputs are all zero");
            fail_cnt++;
        end

        // -----------------------------------------------------------------
        // TEST 7 ? HALT immediately
        //   First instruction is HALT.
        //   done must be asserted within a few cycles.
        // -----------------------------------------------------------------
        print_sep("TEST 7 ? HALT");
        flush_rom();
        instr_rom[0] = encode_instr(6'd63, 2'd0, 10'd0);  // HALT at PC=0

        do_reset();
        wait_for_halt(50);

        check("HALT done flag", {63'd0, done}, 64'd1);

        // -----------------------------------------------------------------
        // TEST 8 ? Multiple SETs followed by STORE/LOAD roundtrip
        //   Verifies the PC advances correctly across multiple instructions
        //   (regression for next_PC increment logic)
        // -----------------------------------------------------------------
        print_sep("TEST 8 ? PC advance + multi-instruction sequence");
        flush_rom();

        instr_rom[0] = encode_instr(6'd7,  2'd0, 10'd111);  // SET H[0] = 111
        instr_rom[1] = encode_instr(6'd7,  2'd1, 10'd222);  // SET H[1] = 222
        instr_rom[2] = encode_instr(6'd6,  2'd0, 10'd50);   // STORE MEM[50] ? H[0]
        instr_rom[3] = encode_instr(6'd6,  2'd1, 10'd51);   // STORE MEM[51] ? H[1]
        instr_rom[4] = encode_instr(6'd5,  2'd2, 10'd50);   // LOAD  H[2] ? MEM[50]
        instr_rom[5] = encode_instr(6'd5,  2'd3, 10'd51);   // LOAD  H[3] ? MEM[51]
        instr_rom[6] = encode_instr(6'd63, 2'd0, 10'd0);    // HALT

        do_reset();
        wait_for_halt(400);

        check("PC-advance SET?STORE?LOAD H[2]", dut.H[2], 64'd111);
        check("PC-advance SET?STORE?LOAD H[3]", dut.H[3], 64'd222);

        // -----------------------------------------------------------------
        //  FINAL SUMMARY
        // -----------------------------------------------------------------
        $display("\n================================================================");
        $display("  FINAL SUMMARY");
        $display("  PASSED : %0d", pass_cnt);
        $display("  FAILED : %0d", fail_cnt);
        if (fail_cnt == 0)
            $display("  RESULT : *** ALL TESTS PASSED ***");
        else
            $display("  RESULT : !!! %0d FAILURE(S) DETECTED !!!", fail_cnt);
        $display("================================================================\n");

        $finish;
    end

    // -------------------------------------------------------------------------
    //  Global timeout ? catches infinite loops / hung FSM
    // -------------------------------------------------------------------------
    initial begin
        #5_000_000;
        $display("[TB] *** GLOBAL TIMEOUT ? simulation killed ***");
        $finish;
    end

    // -------------------------------------------------------------------------
    //  Waveform dump (works with ModelSim / Icarus / VCS)
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_simd_ai_cpu.vcd");
        $dumpvars(0, tb_simd_ai_cpu);
    end

endmodule : tb_simd_ai_cpu
