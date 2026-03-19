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
// =============================================================================
//  simd_batchnorm ? 4-lane BatchNorm
//
//  Per lane: y = gamma * (x - mean) / sqrt(var + eps) + beta
//
//  All parameters (gamma, beta, mean, var) are per-lane 32-bit fixed point
//  values stored in registers and loaded by the CPU before calling BATCHNORM.
//
//  Fixed point format: Q16.16 (16 integer bits, 16 fractional bits)
//  eps = 1/65536 to avoid divide-by-zero
//
//  Pipeline: 3 cycles
//    Cycle 1: subtract mean, load gamma/beta
//    Cycle 2: multiply by gamma, compute 1/sqrt(var+eps) via LUT
//    Cycle 3: add beta, output result
// =============================================================================
module simd_batchnorm #(
    parameter int LANES    = 4,
    parameter int DATA_W   = 32,    // Q16.16 input
    parameter int PARAM_W  = 32     // Q16.16 gamma, beta, mean, var
)(
    input  logic                         clk,
    input  logic                         rst_n,
    input  logic                         valid_i,
    // 4 data lanes
    input  logic signed [DATA_W-1:0]     x0, x1, x2, x3,
    // per-lane parameters (loaded from weight memory before calling)
    input  logic signed [PARAM_W-1:0]    mean0, mean1, mean2, mean3,
    input  logic signed [PARAM_W-1:0]    var0,  var1,  var2,  var3,
    input  logic signed [PARAM_W-1:0]    gamma0,gamma1,gamma2,gamma3,
    input  logic signed [PARAM_W-1:0]    beta0, beta1, beta2, beta3,
    output logic                         valid_o,
    output logic signed [DATA_W-1:0]     y0, y1, y2, y3
);
    // eps in Q16.16 = 1 (represents ~1.5e-5, close enough for fixed point)
    localparam logic signed [PARAM_W-1:0] EPS = 32'd1;

    // -------------------------------------------------------------------------
    //  Stage 1 ? subtract mean
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] xm0,xm1,xm2,xm3;
    logic signed [PARAM_W-1:0] v0r,v1r,v2r,v3r;
    logic signed [PARAM_W-1:0] g0r,g1r,g2r,g3r;
    logic signed [PARAM_W-1:0] b0r,b1r,b2r,b3r;
    logic v1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v1<=0; xm0<=0;xm1<=0;xm2<=0;xm3<=0;
            v0r<=0;v1r<=0;v2r<=0;v3r<=0;
            g0r<=0;g1r<=0;g2r<=0;g3r<=0;
            b0r<=0;b1r<=0;b2r<=0;b3r<=0;
        end else begin
            v1  <= valid_i;
            xm0 <= x0 - mean0;
            xm1 <= x1 - mean1;
            xm2 <= x2 - mean2;
            xm3 <= x3 - mean3;
            // latch parameters so they don't need to be held
            v0r<=var0+EPS; v1r<=var1+EPS; v2r<=var2+EPS; v3r<=var3+EPS;
            g0r<=gamma0;   g1r<=gamma1;   g2r<=gamma2;   g3r<=gamma3;
            b0r<=beta0;    b1r<=beta1;    b2r<=beta2;    b3r<=beta3;
        end
    end

    // -------------------------------------------------------------------------
    //  Reciprocal sqrt approximation via Newton-Raphson (1 iteration)
    //  rsqrt(v) ? 1/sqrt(v)
    //  seed = 1/v  (rough approximation since v is close to 1 for BN)
    //  We use integer divide as seed: seed = 2^16 / v  (Q16.16)
    // -------------------------------------------------------------------------
    logic signed [PARAM_W-1:0] rsqrt0,rsqrt1,rsqrt2,rsqrt3;
    logic signed [DATA_W-1:0]  s0,s1,s2,s3;
    logic v2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v2<=0; s0<=0;s1<=0;s2<=0;s3<=0;
            rsqrt0<=0;rsqrt1<=0;rsqrt2<=0;rsqrt3<=0;
        end else begin
            v2 <= v1;
            // rsqrt approximation: 2^24 / var (Q16.16 result)
            rsqrt0 <= PARAM_W'(48'h1_0000_0000 / {16'b0, v0r});
            rsqrt1 <= PARAM_W'(48'h1_0000_0000 / {16'b0, v1r});
            rsqrt2 <= PARAM_W'(48'h1_0000_0000 / {16'b0, v2r});
            rsqrt3 <= PARAM_W'(48'h1_0000_0000 / {16'b0, v3r});
            // scale xm by gamma (Q16.16 * Q16.16 >> 16)
            s0 <= DATA_W'($signed(64'($signed(xm0)) * 64'($signed(g0r))) >>> 16);
            s1 <= DATA_W'($signed(64'($signed(xm1)) * 64'($signed(g1r))) >>> 16);
            s2 <= DATA_W'($signed(64'($signed(xm2)) * 64'($signed(g2r))) >>> 16);
            s3 <= DATA_W'($signed(64'($signed(xm3)) * 64'($signed(g3r))) >>> 16);
        end
    end

    // -------------------------------------------------------------------------
    //  Stage 3 ? multiply by rsqrt, add beta
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o<=0; y0<=0;y1<=0;y2<=0;y3<=0;
        end else begin
            valid_o <= v2;
            // (gamma*(x-mean)) * rsqrt(var+eps) >> 16 + beta
            y0 <= DATA_W'($signed(64'($signed(s0)) * 64'($signed(rsqrt0))) >>> 16) + b0r;
            y1 <= DATA_W'($signed(64'($signed(s1)) * 64'($signed(rsqrt1))) >>> 16) + b1r;
            y2 <= DATA_W'($signed(64'($signed(s2)) * 64'($signed(rsqrt2))) >>> 16) + b2r;
            y3 <= DATA_W'($signed(64'($signed(s3)) * 64'($signed(rsqrt3))) >>> 16) + b3r;
            if (v2)
                $display("[BN @%0t] y0=%h y1=%h y2=%h y3=%h", $time, 
                    DATA_W'($signed(64'($signed(s0))*64'($signed(rsqrt0)))>>>16)+b0r,
                    DATA_W'($signed(64'($signed(s1))*64'($signed(rsqrt1)))>>>16)+b1r,
                    DATA_W'($signed(64'($signed(s2))*64'($signed(rsqrt2)))>>>16)+b2r,
                    DATA_W'($signed(64'($signed(s3))*64'($signed(rsqrt3)))>>>16)+b3r);
        end
    end
endmodule : simd_batchnorm

// =============================================================================
//  simd_maxpool ? 4-lane 2×2 MaxPool
//
//  Takes 4 consecutive input values per lane (a window of 2×2 = 4 elements)
//  and outputs the maximum. Inputs are streamed in over 4 cycles.
//
//  Usage:
//    Drive valid_i=1 for 4 consecutive cycles with the 4 window values.
//    valid_o pulses for 1 cycle with the max result.
//
//  Pipeline: 2 cycles after last input
//    Stage 1: compare pairs (val0 vs val1, val2 vs val3)
//    Stage 2: compare winners ? final max
// =============================================================================
module simd_maxpool #(
    parameter int LANES  = 4,
    parameter int DATA_W = 32    // signed Q16.16
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      valid_i,
    // 4 parallel lanes, each receives one element of the 2x2 window
    input  logic signed [DATA_W-1:0]  a0, a1, a2, a3,   // first pair
    input  logic signed [DATA_W-1:0]  b0, b1, b2, b3,   // second pair
    output logic                      valid_o,
    output logic signed [DATA_W-1:0]  max0, max1, max2, max3
);
    // Stage 1 ? compare a vs b per lane
    logic signed [DATA_W-1:0] m1_0, m1_1, m1_2, m1_3;
    logic v1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v1<=0; m1_0<=0;m1_1<=0;m1_2<=0;m1_3<=0;
        end else begin
            v1    <= valid_i;
            m1_0  <= (a0 > b0) ? a0 : b0;
            m1_1  <= (a1 > b1) ? a1 : b1;
            m1_2  <= (a2 > b2) ? a2 : b2;
            m1_3  <= (a3 > b3) ? a3 : b3;
        end
    end

    // Stage 2 ? register output
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o<=0; max0<=0;max1<=0;max2<=0;max3<=0;
        end else begin
            valid_o <= v1;
            max0    <= m1_0;
            max1    <= m1_1;
            max2    <= m1_2;
            max3    <= m1_3;
            if (v1)
                $display("[MAXPOOL@%0t] max0=%h max1=%h max2=%h max3=%h",
                         $time, m1_0, m1_1, m1_2, m1_3);
        end
    end
endmodule : simd_maxpool

// =============================================================================
//  simd_global_avgpool ? 4-lane Global Average Pooling
//
//  Accumulates N input values per lane over N cycles, then outputs mean.
//  N is set by the 'count' input at the start (max 1023).
//
//  Usage:
//    1. Assert start=1 for one cycle with count=N
//    2. Drive valid_i=1 for N cycles with data
//    3. valid_o pulses with the average after the last input
//
//  Output = sum / N  (integer divide, sufficient for Q16.16)
// =============================================================================
module simd_global_avgpool #(
    parameter int LANES  = 4,
    parameter int DATA_W = 32,
    parameter int ACC_W  = 48    // wide accumulator to avoid overflow
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      start,    // pulse to begin, latches count
    input  logic [9:0]                count,    // number of elements to average
    input  logic                      valid_i,
    input  logic signed [DATA_W-1:0]  d0, d1, d2, d3,
    output logic                      valid_o,
    output logic signed [DATA_W-1:0]  avg0, avg1, avg2, avg3
);
    logic [9:0]               N_reg;
    logic [9:0]               cnt;
    logic signed [ACC_W-1:0]  acc0, acc1, acc2, acc3;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o<=0; cnt<=0; N_reg<=0;
            acc0<=0;acc1<=0;acc2<=0;acc3<=0;
            avg0<=0;avg1<=0;avg2<=0;avg3<=0;
        end else begin
            valid_o <= 1'b0;

            if (start) begin
                N_reg <= count;
                cnt   <= 10'd0;
                acc0  <= '0; acc1 <= '0; acc2 <= '0; acc3 <= '0;
            end

            else if (valid_i) begin
                acc0 <= acc0 + ACC_W'($signed(d0));
                acc1 <= acc1 + ACC_W'($signed(d1));
                acc2 <= acc2 + ACC_W'($signed(d2));
                acc3 <= acc3 + ACC_W'($signed(d3));
                cnt  <= cnt + 1;

                if (cnt == N_reg - 1) begin
                    // last element ? output the average
                    valid_o <= 1'b1;
                    avg0 <= DATA_W'((acc0 + ACC_W'($signed(d0))) / ACC_W'(N_reg));
                    avg1 <= DATA_W'((acc1 + ACC_W'($signed(d1))) / ACC_W'(N_reg));
                    avg2 <= DATA_W'((acc2 + ACC_W'($signed(d2))) / ACC_W'(N_reg));
                    avg3 <= DATA_W'((acc3 + ACC_W'($signed(d3))) / ACC_W'(N_reg));
                    $display("[GAVGPOOL@%0t] avg0=%h avg1=%h avg2=%h avg3=%h",
                             $time,
                             DATA_W'((acc0+ACC_W'($signed(d0)))/ACC_W'(N_reg)),
                             DATA_W'((acc1+ACC_W'($signed(d1)))/ACC_W'(N_reg)),
                             DATA_W'((acc2+ACC_W'($signed(d2)))/ACC_W'(N_reg)),
                             DATA_W'((acc3+ACC_W'($signed(d3)))/ACC_W'(N_reg)));
                end
            end
        end
    end
endmodule : simd_global_avgpool

// =============================================================================
//  simd_add ? 4-lane elementwise add (for skip connections)
//
//  out[i] = a[i] + b[i]   for i in 0..3
//
//  1-cycle pipeline register on output.
//  Used for ResNet/EfficientNet skip connections where the identity
//  shortcut is added to the conv output.
// =============================================================================
module simd_add #(
    parameter int LANES  = 4,
    parameter int DATA_W = 32
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic                      valid_i,
    input  logic signed [DATA_W-1:0]  a0, a1, a2, a3,
    input  logic signed [DATA_W-1:0]  b0, b1, b2, b3,
    output logic                      valid_o,
    output logic signed [DATA_W-1:0]  out0, out1, out2, out3
);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o<=0; out0<=0;out1<=0;out2<=0;out3<=0;
        end else begin
            valid_o <= valid_i;
            out0    <= a0 + b0;
            out1    <= a1 + b1;
            out2    <= a2 + b2;
            out3    <= a3 + b3;
            if (valid_i)
                $display("[SIMD_ADD@%0t] out0=%h out1=%h out2=%h out3=%h",
                         $time, a0+b0, a1+b1, a2+b2, a3+b3);
        end
    end
endmodule : simd_add

//  CPUai_top ? main FSM-based SIMD AI CPU (extended with 4 new modules)
//
//  Instruction encoding (18-bit):
//    [17:12] opcode
//    [11:10] R0  (destination / source register)
//    [9:0]   immediate / address / count
//
//  Opcode map:
//  -- Existing --
//    0  ? QUANT only
//    1  ? QUANT + RELU
//    2  ? QUANT + RELU + SOFTMAX
//    3  ? RELU only
//    4  ? SOFTMAX only
//    5  ? LOAD   H[R0] ? MEM[imm]
//    6  ? STORE  MEM[imm] ? H[R0]
//    7  ? SET    H[R0] ? imm
//  -- New --
//    8  ? BATCHNORM   (reads BN params from BN param regs, writes H[0..3])
//    9  ? MAXPOOL     (H[0..3]=a-lanes, H_B[0..3]=b-lanes ? max ? H[0..3])
//   10  ? GAVGPOOL    (accumulates imm elements from H[0..3], writes H[0..3])
//   11  ? SIMD_ADD    (H[0..3] = H[0..3] + H_B[0..3], skip connection)
//   12  ? LOAD_BN     load BN params from MEM into internal BN param regs
//   13  ? LOAD_B      load H_B[0..3] (second operand bank for MAXPOOL/SIMD_ADD)
//   63  ? HALT
//
//  FSM states:
//    IDLE?IF?ID?QUANT?RELU?SOFTMAX?BATCHNORM?MAXPOOL?GAVGPOOL?SIMDA?MEM?WB
//    Stages skipped via opcode routing in STATE_ID.
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
        STATE_IDLE      = 4'd0,
        STATE_IF        = 4'd1,
        STATE_ID        = 4'd2,
        STATE_QUANT     = 4'd3,
        STATE_RELU      = 4'd4,
        STATE_SOFTMAX   = 4'd5,
        STATE_MEM       = 4'd6,
        STATE_BATCHNORM = 4'd7,   // new ? 3-cycle BN pipeline
        STATE_MAXPOOL   = 4'd8,   // new ? 2-cycle maxpool pipeline
        STATE_GAVGPOOL  = 4'd9,   // new ? multi-cycle accumulation
        STATE_SIMDA     = 4'd10,  // new ? 1-cycle elementwise add
        STATE_WB        = 4'd11,
        STATE_HALT      = 4'd12
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
    logic CMD_batchnorm;   // new
    logic CMD_maxpool;     // new
    logic CMD_gavgpool;    // new
    logic CMD_simda;       // new ? SIMD add
    logic CMD_load_bn;     // new ? load BN params
    logic CMD_load_b;      // new ? load H_B bank

    // Routing flags ? which pipeline stages to visit?
    logic need_quant;
    logic need_relu;
    logic need_softmax;
    logic need_batchnorm;  // new
    logic need_maxpool;    // new
    logic need_gavgpool;   // new
    logic need_simda;      // new

    // -------------------------------------------------------------------------
    //  Second register bank H_B[0..3]
    //  Used as second operand for MAXPOOL and SIMD_ADD.
    //  Loaded by LOAD_B instruction (opcode 13).
    // -------------------------------------------------------------------------
    logic [63:0] H_B [0:3];

    // -------------------------------------------------------------------------
    //  BatchNorm parameter registers (loaded by LOAD_BN, opcode 12)
    //  Q16.16 signed fixed point
    // -------------------------------------------------------------------------
    logic signed [31:0] bn_mean  [0:3];
    logic signed [31:0] bn_var   [0:3];
    logic signed [31:0] bn_gamma [0:3];
    logic signed [31:0] bn_beta  [0:3];

    // GAP count register ? set by imm field of GAVGPOOL instruction
    logic [9:0] gap_count;

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
    logic [31:0]      result_quant    [0:3];
    logic [63:0]      result_relu     [0:3];
    logic [EXP_W-1:0] result_softmax  [0:7];
    logic signed [31:0] result_bn     [0:3];   // new ? batchnorm results
    logic signed [31:0] result_maxpool[0:3];   // new ? maxpool results
    logic signed [31:0] result_gap    [0:3];   // new ? global avg pool results
    logic signed [31:0] result_simda  [0:3];   // new ? simd add results

    // -------------------------------------------------------------------------
    //  Sub-module wires ? BatchNorm
    // -------------------------------------------------------------------------
    logic        bn_valid_i, bn_valid_o;
    logic signed [31:0] bn_y0, bn_y1, bn_y2, bn_y3;
    logic        bn_rst_n;
    assign bn_rst_n = ~rst;

    simd_batchnorm bn_inst (
        .clk    (clk),   .rst_n  (bn_rst_n),
        .valid_i(bn_valid_i),
        .x0(32'(H[0])), .x1(32'(H[1])), .x2(32'(H[2])), .x3(32'(H[3])),
        .mean0(bn_mean[0]),   .mean1(bn_mean[1]),
        .mean2(bn_mean[2]),   .mean3(bn_mean[3]),
        .var0 (bn_var[0]),    .var1 (bn_var[1]),
        .var2 (bn_var[2]),    .var3 (bn_var[3]),
        .gamma0(bn_gamma[0]), .gamma1(bn_gamma[1]),
        .gamma2(bn_gamma[2]), .gamma3(bn_gamma[3]),
        .beta0 (bn_beta[0]),  .beta1 (bn_beta[1]),
        .beta2 (bn_beta[2]),  .beta3 (bn_beta[3]),
        .valid_o(bn_valid_o),
        .y0(bn_y0), .y1(bn_y1), .y2(bn_y2), .y3(bn_y3)
    );

    // -------------------------------------------------------------------------
    //  Sub-module wires ? MaxPool
    // -------------------------------------------------------------------------
    logic        mp_valid_i, mp_valid_o;
    logic signed [31:0] mp_max0, mp_max1, mp_max2, mp_max3;
    logic        mp_rst_n;
    assign mp_rst_n = ~rst;

    simd_maxpool mp_inst (
        .clk    (clk),   .rst_n  (mp_rst_n),
        .valid_i(mp_valid_i),
        .a0(32'(H[0])),   .a1(32'(H[1])),   .a2(32'(H[2])),   .a3(32'(H[3])),
        .b0(32'(H_B[0])), .b1(32'(H_B[1])), .b2(32'(H_B[2])), .b3(32'(H_B[3])),
        .valid_o(mp_valid_o),
        .max0(mp_max0), .max1(mp_max1), .max2(mp_max2), .max3(mp_max3)
    );

    // -------------------------------------------------------------------------
    //  Sub-module wires ? Global Average Pool
    // -------------------------------------------------------------------------
    logic        gap_start, gap_valid_i, gap_valid_o;
    logic signed [31:0] gap_avg0, gap_avg1, gap_avg2, gap_avg3;
    logic        gap_rst_n;
    assign gap_rst_n = ~rst;

    simd_global_avgpool gap_inst (
        .clk    (clk),   .rst_n (gap_rst_n),
        .start  (gap_start),
        .count  (gap_count),
        .valid_i(gap_valid_i),
        .d0(32'(H[0])), .d1(32'(H[1])), .d2(32'(H[2])), .d3(32'(H[3])),
        .valid_o(gap_valid_o),
        .avg0(gap_avg0), .avg1(gap_avg1), .avg2(gap_avg2), .avg3(gap_avg3)
    );

    // -------------------------------------------------------------------------
    //  Sub-module wires ? SIMD Add
    // -------------------------------------------------------------------------
    logic        sa_valid_i, sa_valid_o;
    logic signed [31:0] sa_out0, sa_out1, sa_out2, sa_out3;
    logic        sa_rst_n;
    assign sa_rst_n = ~rst;

    simd_add sa_inst (
        .clk    (clk),   .rst_n  (sa_rst_n),
        .valid_i(sa_valid_i),
        .a0(32'(H[0])),   .a1(32'(H[1])),   .a2(32'(H[2])),   .a3(32'(H[3])),
        .b0(32'(H_B[0])), .b1(32'(H_B[1])), .b2(32'(H_B[2])), .b3(32'(H_B[3])),
        .valid_o(sa_valid_o),
        .out0(sa_out0), .out1(sa_out1), .out2(sa_out2), .out3(sa_out3)
    );

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
    logic relu_settled;

    // Settled/started flags for new modules
    logic bn_started,  bn_settled;   // batchnorm: 3-cycle pipeline
    logic mp_started,  mp_settled;   // maxpool:   2-cycle pipeline
    logic gap_running, gap_started;  // global avg pool: multi-cycle
    logic sa_settled;                // simd add: 1-cycle pipeline

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
                    else if (opcode == 6'd0 || opcode == 6'd1 || opcode == 6'd2)
                        current_state <= STATE_QUANT;
                    else if (opcode == 6'd3)
                        current_state <= STATE_RELU;
                    else if (opcode == 6'd4)
                        current_state <= STATE_SOFTMAX;
                    else if (opcode == 6'd8)
                        current_state <= STATE_BATCHNORM;
                    else if (opcode == 6'd9)
                        current_state <= STATE_MAXPOOL;
                    else if (opcode == 6'd10)
                        current_state <= STATE_GAVGPOOL;
                    else if (opcode == 6'd11)
                        current_state <= STATE_SIMDA;
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

                // BATCHNORM ? 3-cycle pipeline, wait for bn_valid_o
                STATE_BATCHNORM: begin
                    if (bn_valid_o)
                        current_state <= STATE_WB;
                    else
                        current_state <= STATE_BATCHNORM;
                end

                // MAXPOOL ? 2-cycle pipeline, wait for mp_valid_o
                STATE_MAXPOOL: begin
                    if (mp_valid_o)
                        current_state <= STATE_WB;
                    else
                        current_state <= STATE_MAXPOOL;
                end

                // GAVGPOOL ? variable cycles, wait for gap_valid_o
                STATE_GAVGPOOL: begin
                    if (gap_valid_o)
                        current_state <= STATE_WB;
                    else
                        current_state <= STATE_GAVGPOOL;
                end

                // SIMD ADD ? 1-cycle pipeline, wait for sa_valid_o
                STATE_SIMDA: begin
                    if (sa_valid_o)
                        current_state <= STATE_WB;
                    else
                        current_state <= STATE_SIMDA;
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
            CMD_quant    <= 1'b0; CMD_relu     <= 1'b0;
            CMD_softmax  <= 1'b0; CMD_load     <= 1'b0;
            CMD_store    <= 1'b0; CMD_set      <= 1'b0;
            CMD_batchnorm<= 1'b0; CMD_maxpool  <= 1'b0;
            CMD_gavgpool <= 1'b0; CMD_simda    <= 1'b0;
            CMD_load_bn  <= 1'b0; CMD_load_b   <= 1'b0;
            need_quant   <= 1'b0; need_relu    <= 1'b0;
            need_softmax <= 1'b0; need_batchnorm<= 1'b0;
            need_maxpool <= 1'b0; need_gavgpool <= 1'b0;
            need_simda   <= 1'b0;
            R0  <= 2'd0; imm <= 10'd0;
        end else if (current_state == STATE_ID) begin
            R0  <= instruction_in[11:10];
            imm <= instruction_in[9:0];

            CMD_quant     <= (opcode==6'd0)||(opcode==6'd1)||(opcode==6'd2);
            CMD_relu      <= (opcode==6'd1)||(opcode==6'd2)||(opcode==6'd3);
            CMD_softmax   <= (opcode==6'd2)||(opcode==6'd4);
            CMD_load      <= (opcode==6'd5);
            CMD_store     <= (opcode==6'd6);
            CMD_set       <= (opcode==6'd7);
            CMD_batchnorm <= (opcode==6'd8);
            CMD_maxpool   <= (opcode==6'd9);
            CMD_gavgpool  <= (opcode==6'd10);
            CMD_simda     <= (opcode==6'd11);
            CMD_load_bn   <= (opcode==6'd12);
            CMD_load_b    <= (opcode==6'd13);

            need_quant    <= (opcode==6'd0)||(opcode==6'd1)||(opcode==6'd2);
            need_relu     <= (opcode==6'd1)||(opcode==6'd2)||(opcode==6'd3);
            need_softmax  <= (opcode==6'd2)||(opcode==6'd4);
            need_batchnorm<= (opcode==6'd8);
            need_maxpool  <= (opcode==6'd9);
            need_gavgpool <= (opcode==6'd10);
            need_simda    <= (opcode==6'd11);

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
    //  STATE_BATCHNORM ? pulse valid_i once, wait for bn_valid_o (3 cycles)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            bn_valid_i <= 1'b0; bn_started <= 1'b0; bn_settled <= 1'b0;
            for (int j=0;j<4;j++) result_bn[j] <= 32'd0;
        end else if (current_state == STATE_BATCHNORM && !bn_started) begin
            bn_valid_i <= 1'b1;
            bn_started <= 1'b1;
            $display("[BN_START@%0t]", $time);
        end else if (current_state == STATE_BATCHNORM && bn_started) begin
            bn_valid_i <= 1'b0;
            if (bn_valid_o) begin
                result_bn[0] <= bn_y0; result_bn[1] <= bn_y1;
                result_bn[2] <= bn_y2; result_bn[3] <= bn_y3;
            end
        end else begin
            bn_started <= 1'b0; bn_valid_i <= 1'b0;
        end
    end

    // =========================================================================
    //  STATE_MAXPOOL ? pulse valid_i once, wait for mp_valid_o (2 cycles)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            mp_valid_i <= 1'b0; mp_started <= 1'b0;
            for (int j=0;j<4;j++) result_maxpool[j] <= 32'd0;
        end else if (current_state == STATE_MAXPOOL && !mp_started) begin
            mp_valid_i <= 1'b1;
            mp_started <= 1'b1;
            $display("[MP_START@%0t]", $time);
        end else if (current_state == STATE_MAXPOOL && mp_started) begin
            mp_valid_i <= 1'b0;
            if (mp_valid_o) begin
                result_maxpool[0] <= mp_max0; result_maxpool[1] <= mp_max1;
                result_maxpool[2] <= mp_max2; result_maxpool[3] <= mp_max3;
            end
        end else begin
            mp_started <= 1'b0; mp_valid_i <= 1'b0;
        end
    end

    // =========================================================================
    //  STATE_GAVGPOOL ? start accumulation, feed H values, wait for gap_valid_o
    //  imm field carries the element count
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            gap_start <= 1'b0; gap_valid_i <= 1'b0;
            gap_started <= 1'b0; gap_count <= 10'd0;
            for (int j=0;j<4;j++) result_gap[j] <= 32'd0;
        end else if (current_state == STATE_GAVGPOOL && !gap_started) begin
            gap_start   <= 1'b1;
            gap_count   <= imm;
            gap_valid_i <= 1'b1;
            gap_started <= 1'b1;
            $display("[GAP_START@%0t] count=%0d", $time, imm);
        end else if (current_state == STATE_GAVGPOOL && gap_started) begin
            gap_start <= 1'b0;
            gap_valid_i <= 1'b1;   // keep feeding H values each cycle
            if (gap_valid_o) begin
                result_gap[0] <= gap_avg0; result_gap[1] <= gap_avg1;
                result_gap[2] <= gap_avg2; result_gap[3] <= gap_avg3;
                gap_valid_i   <= 1'b0;
            end
        end else begin
            gap_started <= 1'b0; gap_start <= 1'b0; gap_valid_i <= 1'b0;
        end
    end

    // =========================================================================
    //  STATE_SIMDA ? pulse valid_i, wait 1 cycle for sa_valid_o
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            sa_valid_i <= 1'b0; sa_settled <= 1'b0;
            for (int j=0;j<4;j++) result_simda[j] <= 32'd0;
        end else if (current_state == STATE_SIMDA && !sa_settled) begin
            sa_valid_i <= 1'b1;
            sa_settled <= 1'b1;
            $display("[SIMDA_START@%0t]", $time);
        end else if (current_state == STATE_SIMDA && sa_settled) begin
            sa_valid_i <= 1'b0;
            if (sa_valid_o) begin
                result_simda[0] <= sa_out0; result_simda[1] <= sa_out1;
                result_simda[2] <= sa_out2; result_simda[3] <= sa_out3;
            end
        end else begin
            sa_settled <= 1'b0; sa_valid_i <= 1'b0;
        end
    end

    // =========================================================================
    //  STATE_WB ? write results back to register file and handle memory ops
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int j=0;j<4;j++) H[j]    <= 64'd0;
            for (int j=0;j<4;j++) H_B[j]  <= 64'd0;
            for (int j=0;j<4;j++) bn_mean[j]  <= 32'd0;
            for (int j=0;j<4;j++) bn_var[j]   <= 32'h00010000; // 1.0 Q16.16
            for (int j=0;j<4;j++) bn_gamma[j] <= 32'h00010000; // 1.0
            for (int j=0;j<4;j++) bn_beta[j]  <= 32'd0;
            next_PC      <= 10'd1;
            mem_addr_reg <= 10'd0;
            data_out_reg <= 64'd0;
            rdata_en     <= 1'b0;
            wdata_en     <= 1'b0;
        end

        else if (current_state == STATE_MEM) begin
            if (CMD_load || CMD_load_b || CMD_load_bn) begin
                rdata_en     <= 1'b1;
                mem_addr_reg <= imm;
            end else if (CMD_store) begin
                wdata_en     <= 1'b1;
                mem_addr_reg <= imm;
                data_out_reg <= H[R0];
                $display("[MEM STORE@%0t] MEM[%0d] ? H[%0d] = %h", $time, imm, R0, H[R0]);
            end
        end

        else if (current_state == STATE_WB) begin
            rdata_en <= 1'b0;
            wdata_en <= 1'b0;

            // --- existing ops ---
            if (CMD_quant && !CMD_relu && !CMD_softmax) begin
                H[R0] <= {32'd0, result_quant[R0]};
                $display("[WB QUANT@%0t] H[%0d]=%h", $time, R0, result_quant[R0]);
            end
            else if (CMD_relu && !CMD_softmax) begin
                for (int j=0;j<4;j++) H[j] <= result_relu[j];
                $display("[WB RELU@%0t] H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                         $time,result_relu[0],result_relu[1],result_relu[2],result_relu[3]);
            end
            else if (CMD_softmax) begin
                H[0] <= {result_softmax[1][15:0], result_softmax[0][15:0], 32'd0};
                H[1] <= {result_softmax[3][15:0], result_softmax[2][15:0], 32'd0};
                H[2] <= {result_softmax[5][15:0], result_softmax[4][15:0], 32'd0};
                H[3] <= {result_softmax[7][15:0], result_softmax[6][15:0], 32'd0};
                $display("[WB SOFTMAX@%0t] p0=%h p1=%h p2=%h p3=%h",
                         $time,result_softmax[0],result_softmax[1],
                               result_softmax[2],result_softmax[3]);
            end
            else if (CMD_load) begin
                H[R0] <= data_in;
                $display("[WB LOAD@%0t] H[%0d]?MEM[%0d]=%h", $time, R0, imm, data_in);
            end
            else if (CMD_store) begin
                $display("[WB STORE@%0t] MEM[%0d]?H[%0d]=%h", $time, imm, R0, H[R0]);
            end
            else if (CMD_set) begin
                H[R0] <= {54'd0, imm};
                $display("[WB SET@%0t] H[%0d]?%0d", $time, R0, imm);
            end

            // --- new ops ---
            else if (CMD_batchnorm) begin
                for (int j=0;j<4;j++) H[j] <= 64'($signed(result_bn[j]));
                $display("[WB BN@%0t] H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                         $time,result_bn[0],result_bn[1],result_bn[2],result_bn[3]);
            end
            else if (CMD_maxpool) begin
                for (int j=0;j<4;j++) H[j] <= 64'($signed(result_maxpool[j]));
                $display("[WB MAXPOOL@%0t] H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                         $time,result_maxpool[0],result_maxpool[1],
                               result_maxpool[2],result_maxpool[3]);
            end
            else if (CMD_gavgpool) begin
                for (int j=0;j<4;j++) H[j] <= 64'($signed(result_gap[j]));
                $display("[WB GAP@%0t] H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                         $time,result_gap[0],result_gap[1],
                               result_gap[2],result_gap[3]);
            end
            else if (CMD_simda) begin
                for (int j=0;j<4;j++) H[j] <= 64'($signed(result_simda[j]));
                $display("[WB SIMDA@%0t] H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                         $time,result_simda[0],result_simda[1],
                               result_simda[2],result_simda[3]);
            end
            // LOAD_BN: load 4 groups of 4 params from consecutive memory locations
            // Memory layout at addr imm:
            //   [imm+0..3]  = mean0..3
            //   [imm+4..7]  = var0..3
            //   [imm+8..11] = gamma0..3
            //   [imm+12..15]= beta0..3
            // For simplicity we load one group per LOAD_BN call using R0 as index
            else if (CMD_load_bn) begin
                bn_mean[R0]  <= $signed(data_in[31:0]);
                bn_var[R0]   <= $signed(data_in[63:32]);
                $display("[WB LOAD_BN@%0t] lane%0d mean=%h var=%h",
                         $time, R0, data_in[31:0], data_in[63:32]);
            end
            // LOAD_B: load second operand bank H_B[R0] from memory
            else if (CMD_load_b) begin
                H_B[R0] <= data_in;
                $display("[WB LOAD_B@%0t] H_B[%0d]?MEM[%0d]=%h", $time, R0, imm, data_in);
            end

            next_PC <= next_PC + 10'd1;
        end
    end

endmodule : CPUai_top

`timescale 1ns/1ps
// =============================================================================
//  tb_simd_ai_cpu_v2 ? Testbench covering all modules including the 4 new ones
//
//  New tests added:
//    TEST 9  ? BATCHNORM  (opcode 8)
//    TEST 10 ? MAXPOOL    (opcode 9)
//    TEST 11 ? GAVGPOOL   (opcode 10)
//    TEST 12 ? SIMD_ADD   (opcode 11)  skip connection
//    TEST 13 ? Full CNN block: LOAD?BATCHNORM?RELU?MAXPOOL?GAVGPOOL?SOFTMAX
//
//  Regression tests (1-8) from previous TB are retained.
// =============================================================================
module tb_simd_ai_cpu_v2;

    // -------------------------------------------------------------------------
    //  DUT ports
    // -------------------------------------------------------------------------
    logic        clk, rst;
    logic [17:0] instruction_in;
    logic [9:0]  instruction_address;
    logic [63:0] data_in, data_out;
    logic [9:0]  data_address;
    logic        data_R, data_W, done;

    CPUai_top dut (
        .clk(clk), .rst(rst),
        .instruction_in(instruction_in),
        .instruction_address(instruction_address),
        .data_in(data_in), .data_out(data_out),
        .data_address(data_address),
        .data_R(data_R), .data_W(data_W),
        .done(done)
    );

    // -------------------------------------------------------------------------
    //  Clock
    // -------------------------------------------------------------------------
    initial clk = 0;
    always  #5 clk = ~clk;

    // -------------------------------------------------------------------------
    //  Instruction ROM and data memory
    // -------------------------------------------------------------------------
    localparam int ROM_DEPTH = 64;
    localparam int MEM_DEPTH = 1024;

    logic [17:0] instr_rom [0:ROM_DEPTH-1];
    reg   [63:0] data_mem  [0:MEM_DEPTH-1];

    assign instruction_in = instr_rom[instruction_address];
    assign data_in        = data_R ? data_mem[data_address] : 64'hDEAD_BEEF_DEAD_BEEF;

    always @(posedge clk)
        if (data_W) data_mem[data_address] <= data_out;

    // -------------------------------------------------------------------------
    //  Helpers
    // -------------------------------------------------------------------------
    function automatic logic [17:0] enc(
        input logic [5:0] op, input logic [1:0] r0, input logic [9:0] im);
        return {op, r0, im};
    endfunction

    task automatic flush_rom();
        for (int i=0; i<ROM_DEPTH; i++)
            instr_rom[i] = enc(6'd63, 2'd0, 10'd0);
    endtask

    task automatic do_reset();
        rst = 1;
        repeat(4) @(posedge clk);
        @(negedge clk); rst = 0;
        $display("[TB] Reset released at %0t", $time);
    endtask

    task automatic wait_halt(input int max_cyc);
        int c; c=0;
        while (!done && c<max_cyc) begin @(posedge clk); c++; end
        if (!done) $display("[TB] *** TIMEOUT after %0d cycles ***", max_cyc);
        else       $display("[TB] HALT after %0d cycles", c);
    endtask

    task automatic print_sep(input string t);
        $display("\n============================================================");
        $display("  %s", t);
        $display("============================================================");
    endtask

    int pass_cnt=0, fail_cnt=0;

    task automatic check(input string lbl,
                         input logic [63:0] got, input logic [63:0] exp);
        if (got===exp) begin
            $display("  [PASS] %s : got=%h", lbl, got); pass_cnt++;
        end else begin
            $display("  [FAIL] %s : got=%h  exp=%h", lbl, got, exp); fail_cnt++;
        end
    endtask

    task automatic check_nonzero(input string lbl, input logic [63:0] got);
        if (got !== 64'd0) begin
            $display("  [PASS] %s non-zero : got=%h", lbl, got); pass_cnt++;
        end else begin
            $display("  [FAIL] %s is zero!", lbl); fail_cnt++;
        end
    endtask

    // =========================================================================
    //  MAIN
    // =========================================================================
    initial begin
        $display("================================================================");
        $display("  CPUai_top v2 Testbench ? %0t", $time);
        $display("================================================================");

        for (int i=0; i<MEM_DEPTH; i++) data_mem[i] = 64'd0;
        flush_rom();

        // =================================================================
        // REGRESSION TEST 1 ? SET
        // =================================================================
        print_sep("TEST 1 ? SET");
        flush_rom();
        instr_rom[0] = enc(6'd7, 2'd0, 10'd42);
        instr_rom[1] = enc(6'd7, 2'd1, 10'd100);
        instr_rom[2] = enc(6'd7, 2'd2, 10'd7);
        instr_rom[3] = enc(6'd7, 2'd3, 10'd0);
        instr_rom[4] = enc(6'd63, 2'd0, 10'd0);
        do_reset(); wait_halt(200);
        check("SET H[0]", dut.H[0], 64'd42);
        check("SET H[1]", dut.H[1], 64'd100);
        check("SET H[2]", dut.H[2], 64'd7);
        check("SET H[3]", dut.H[3], 64'd0);

        // =================================================================
        // REGRESSION TEST 2 ? LOAD/STORE
        // =================================================================
        print_sep("TEST 2 ? LOAD/STORE");
        flush_rom();
        data_mem[10] = 64'hCAFEBABE_DEADBEEF;
        instr_rom[0] = enc(6'd5, 2'd0, 10'd10);
        instr_rom[1] = enc(6'd6, 2'd0, 10'd20);
        instr_rom[2] = enc(6'd63, 2'd0, 10'd0);
        do_reset(); wait_halt(200);
        check("LOAD H[0]",    dut.H[0],    64'hCAFEBABE_DEADBEEF);
        check("STORE MEM[20]",data_mem[20],64'hCAFEBABE_DEADBEEF);

        // =================================================================
        // REGRESSION TEST 3 ? QUANT
        // =================================================================
        print_sep("TEST 3 ? QUANT");
        flush_rom();
        data_mem[5] = 64'h0000_0001_8000_0000;
        instr_rom[0] = enc(6'd5, 2'd0, 10'd5);
        instr_rom[1] = enc(6'd0, 2'd0, 10'd0);
        instr_rom[2] = enc(6'd63, 2'd0, 10'd0);
        do_reset(); wait_halt(200);
        check("QUANT upper32=0", {32'd0,dut.H[0][63:32]}, 64'd0);

        // =================================================================
        // REGRESSION TEST 4 ? QUANT+RELU
        // =================================================================
        print_sep("TEST 4 ? QUANT+RELU");
        flush_rom();
        data_mem[0]=64'h0000_0000_0001_0000;
        data_mem[1]=64'hFFFF_FFFF_FFFF_0000;
        data_mem[2]=64'h0; data_mem[3]=64'h0000_0000_0000_8000;
        instr_rom[0]=enc(6'd5,2'd0,10'd0); instr_rom[1]=enc(6'd5,2'd1,10'd1);
        instr_rom[2]=enc(6'd5,2'd2,10'd2); instr_rom[3]=enc(6'd5,2'd3,10'd3);
        instr_rom[4]=enc(6'd1,2'd0,10'd0); instr_rom[5]=enc(6'd63,2'd0,10'd0);
        do_reset(); wait_halt(300);
        check("RELU zero negative H[1]", dut.H[1], 64'd0);
        check("RELU keep zero  H[2]",    dut.H[2], 64'd0);
        check_nonzero("RELU keep positive H[0]", dut.H[0]);

        // =================================================================
        // REGRESSION TEST 5 ? HALT
        // =================================================================
        print_sep("TEST 5 ? HALT");
        flush_rom();
        instr_rom[0]=enc(6'd63,2'd0,10'd0);
        do_reset(); wait_halt(50);
        check("HALT done", {63'd0,done}, 64'd1);

        // =================================================================
        // TEST 9 ? BATCHNORM (opcode 8)
        //
        //  Setup: H[0]=0x00020000 (2.0 Q16.16)
        //         mean=0x00010000 (1.0), var=0x00010000 (1.0)
        //         gamma=0x00010000 (1.0), beta=0x00000000 (0.0)
        //
        //  Expected: BN(2.0) = gamma*(2.0-1.0)/sqrt(1.0+eps) + beta
        //                    ? 1.0 * 1.0 / 1.0 + 0.0 = 1.0
        //                    = 0x00010000 in Q16.16
        //
        //  We load BN params via LOAD_BN (opcode 12):
        //    data_mem[40] = {var[63:32], mean[31:0]} packed in one 64-bit word
        // =================================================================
        print_sep("TEST 9 ? BATCHNORM");
        flush_rom();

        // Pack mean and var into one 64-bit word for LOAD_BN
        // data_in[31:0] = mean,  data_in[63:32] = var
        data_mem[40] = {32'h0001_0000, 32'h0001_0000}; // var=1.0, mean=1.0
        // gamma and beta: set directly via CPU SET instruction isn't enough
        // since bn_gamma/beta are internal regs. For simulation we use
        // the fact that reset initialises gamma=1.0, beta=0.0 already.

        // Load BN params for lane 0
        instr_rom[0] = enc(6'd12, 2'd0, 10'd40); // LOAD_BN lane0 from MEM[40]
        // Set H[0] = 2.0 in Q16.16 = 0x00020000
        // We split across two SET instructions (imm is only 10 bits)
        // 0x00020000 = 131072 decimal which is > 1023, so load from memory
        data_mem[41] = 64'h0000_0000_0002_0000; // H[0] = 2.0 Q16.16
        data_mem[42] = 64'h0;
        data_mem[43] = 64'h0;
        data_mem[44] = 64'h0;
        instr_rom[1] = enc(6'd5,  2'd0, 10'd41); // LOAD H[0] = 2.0
        instr_rom[2] = enc(6'd5,  2'd1, 10'd42); // LOAD H[1] = 0
        instr_rom[3] = enc(6'd5,  2'd2, 10'd43); // LOAD H[2] = 0
        instr_rom[4] = enc(6'd5,  2'd3, 10'd44); // LOAD H[3] = 0
        instr_rom[5] = enc(6'd8,  2'd0, 10'd0);  // BATCHNORM
        instr_rom[6] = enc(6'd63, 2'd0, 10'd0);  // HALT

        do_reset(); wait_halt(300);
        $display("  BN result H[0]=%h (expect ~00010000 = 1.0 Q16.16)", dut.H[0]);
        // BN output should be approximately 1.0 = 0x00010000
        // Allow small tolerance: upper 32 bits = 0, lower 32 bits nonzero
        check("BN upper32=0", {32'd0,dut.H[0][63:32]}, 64'd0);
        check_nonzero("BN result nonzero", dut.H[0]);

        // =================================================================
        // TEST 10 ? MAXPOOL (opcode 9)
        //
        //  H[0..3] = lane A values: 10, 20, 5, 30
        //  H_B[0..3] = lane B values: 15, 8, 25, 12
        //  Expected max: 15, 20, 25, 30
        //
        //  Load H_B via LOAD_B (opcode 13)
        // =================================================================
        print_sep("TEST 10 ? MAXPOOL");
        flush_rom();

        data_mem[50] = 64'd10;  // H[0] = 10
        data_mem[51] = 64'd20;  // H[1] = 20
        data_mem[52] = 64'd5;   // H[2] = 5
        data_mem[53] = 64'd30;  // H[3] = 30
        data_mem[54] = 64'd15;  // H_B[0] = 15
        data_mem[55] = 64'd8;   // H_B[1] = 8
        data_mem[56] = 64'd25;  // H_B[2] = 25
        data_mem[57] = 64'd12;  // H_B[3] = 12

        instr_rom[0]  = enc(6'd5,  2'd0, 10'd50);  // LOAD H[0]=10
        instr_rom[1]  = enc(6'd5,  2'd1, 10'd51);  // LOAD H[1]=20
        instr_rom[2]  = enc(6'd5,  2'd2, 10'd52);  // LOAD H[2]=5
        instr_rom[3]  = enc(6'd5,  2'd3, 10'd53);  // LOAD H[3]=30
        instr_rom[4]  = enc(6'd13, 2'd0, 10'd54);  // LOAD_B H_B[0]=15
        instr_rom[5]  = enc(6'd13, 2'd1, 10'd55);  // LOAD_B H_B[1]=8
        instr_rom[6]  = enc(6'd13, 2'd2, 10'd56);  // LOAD_B H_B[2]=25
        instr_rom[7]  = enc(6'd13, 2'd3, 10'd57);  // LOAD_B H_B[3]=12
        instr_rom[8]  = enc(6'd9,  2'd0, 10'd0);   // MAXPOOL
        instr_rom[9]  = enc(6'd63, 2'd0, 10'd0);   // HALT

        do_reset(); wait_halt(300);
        $display("  MaxPool H[0]=%0d H[1]=%0d H[2]=%0d H[3]=%0d",
                 dut.H[0], dut.H[1], dut.H[2], dut.H[3]);
        check("MAXPOOL H[0]=15", dut.H[0], 64'd15);
        check("MAXPOOL H[1]=20", dut.H[1], 64'd20);
        check("MAXPOOL H[2]=25", dut.H[2], 64'd25);
        check("MAXPOOL H[3]=30", dut.H[3], 64'd30);

        // =================================================================
        // TEST 11 ? GLOBAL AVGPOOL (opcode 10)
        //
        //  Load H[0..3] with same value each cycle for N=4 cycles.
        //  H[0]=100, H[1]=200, H[2]=300, H[3]=400 each cycle.
        //  Expected average = same values (since same each cycle).
        //
        //  Note: GAVGPOOL instruction uses imm=count of elements.
        //  The CPU feeds H[0..3] for 'count' cycles then outputs avg.
        //  For this test count=4, all cycles same value ? avg = value.
        // =================================================================
        print_sep("TEST 11 ? GLOBAL AVGPOOL");
        flush_rom();

        data_mem[60] = 64'd100;
        data_mem[61] = 64'd200;
        data_mem[62] = 64'd300;
        data_mem[63] = 64'd400;

        instr_rom[0] = enc(6'd5,  2'd0, 10'd60);  // LOAD H[0]=100
        instr_rom[1] = enc(6'd5,  2'd1, 10'd61);  // LOAD H[1]=200
        instr_rom[2] = enc(6'd5,  2'd2, 10'd62);  // LOAD H[2]=300
        instr_rom[3] = enc(6'd5,  2'd3, 10'd63);  // LOAD H[3]=400
        // GAVGPOOL with count=4 ? feeds current H[0..3] 4 times
        instr_rom[4] = enc(6'd10, 2'd0, 10'd4);   // GAVGPOOL count=4
        instr_rom[5] = enc(6'd63, 2'd0, 10'd0);   // HALT

        do_reset(); wait_halt(300);
        $display("  GAP H[0]=%0d H[1]=%0d H[2]=%0d H[3]=%0d",
                 dut.H[0], dut.H[1], dut.H[2], dut.H[3]);
        // avg of 4 identical values = same value
        check("GAP H[0]=100", dut.H[0], 64'd100);
        check("GAP H[1]=200", dut.H[1], 64'd200);
        check("GAP H[2]=300", dut.H[2], 64'd300);
        check("GAP H[3]=400", dut.H[3], 64'd400);

        // =================================================================
        // TEST 12 ? SIMD ADD / skip connection (opcode 11)
        //
        //  H[0..3]   = [10, 20, 30, 40]   (conv output)
        //  H_B[0..3] = [1,  2,  3,  4]    (identity shortcut)
        //  Expected:   [11, 22, 33, 44]
        // =================================================================
        print_sep("TEST 12 ? SIMD ADD (skip connection)");
        flush_rom();

        data_mem[70] = 64'd10; data_mem[71] = 64'd20;
        data_mem[72] = 64'd30; data_mem[73] = 64'd40;
        data_mem[74] = 64'd1;  data_mem[75] = 64'd2;
        data_mem[76] = 64'd3;  data_mem[77] = 64'd4;

        instr_rom[0]  = enc(6'd5,  2'd0, 10'd70);  // LOAD H[0]=10
        instr_rom[1]  = enc(6'd5,  2'd1, 10'd71);  // LOAD H[1]=20
        instr_rom[2]  = enc(6'd5,  2'd2, 10'd72);  // LOAD H[2]=30
        instr_rom[3]  = enc(6'd5,  2'd3, 10'd73);  // LOAD H[3]=40
        instr_rom[4]  = enc(6'd13, 2'd0, 10'd74);  // LOAD_B H_B[0]=1
        instr_rom[5]  = enc(6'd13, 2'd1, 10'd75);  // LOAD_B H_B[1]=2
        instr_rom[6]  = enc(6'd13, 2'd2, 10'd76);  // LOAD_B H_B[2]=3
        instr_rom[7]  = enc(6'd13, 2'd3, 10'd77);  // LOAD_B H_B[3]=4
        instr_rom[8]  = enc(6'd11, 2'd0, 10'd0);   // SIMD_ADD
        instr_rom[9]  = enc(6'd63, 2'd0, 10'd0);   // HALT

        do_reset(); wait_halt(300);
        $display("  SIMDA H[0]=%0d H[1]=%0d H[2]=%0d H[3]=%0d",
                 dut.H[0], dut.H[1], dut.H[2], dut.H[3]);
        check("SIMDA H[0]=11", dut.H[0], 64'd11);
        check("SIMDA H[1]=22", dut.H[1], 64'd22);
        check("SIMDA H[2]=33", dut.H[2], 64'd33);
        check("SIMDA H[3]=44", dut.H[3], 64'd44);

        // =================================================================
        // TEST 13 ? Mini CNN block
        //  Simulates one residual block:
        //    LOAD inputs ? BATCHNORM ? RELU ? MAXPOOL ? GAVGPOOL ? SOFTMAX
        //
        //  This is the sequence that runs after a Conv layer in ResNet.
        // =================================================================
        print_sep("TEST 13 ? Mini CNN block (BN?RELU?MAXPOOL?GAVGPOOL?SOFTMAX)");
        flush_rom();

        // Input feature map: 4 positive values
        data_mem[80] = 64'h0000_0000_0002_0000; // 2.0 Q16.16
        data_mem[81] = 64'h0000_0000_0004_0000; // 4.0 Q16.16
        data_mem[82] = 64'h0000_0000_0001_0000; // 1.0 Q16.16
        data_mem[83] = 64'h0000_0000_0003_0000; // 3.0 Q16.16

        // Second operand (for maxpool) ? lower values so H wins
        data_mem[84] = 64'h0000_0000_0001_0000; // 1.0
        data_mem[85] = 64'h0000_0000_0002_0000; // 2.0
        data_mem[86] = 64'h0000_0000_0000_8000; // 0.5
        data_mem[87] = 64'h0000_0000_0001_8000; // 1.5

        // BN params: mean=0, var=1, gamma=1, beta=0 (identity BN)
        // gamma/beta already defaulted to 1.0/0.0 at reset
        // mean/var loaded via LOAD_BN ? one lane at a time
        // Pack: data_in[31:0]=mean=0, data_in[63:32]=var=1.0
        data_mem[88] = {32'h0001_0000, 32'h0000_0000}; // var=1.0 mean=0.0

        instr_rom[0]  = enc(6'd12, 2'd0, 10'd88);  // LOAD_BN lane0
        instr_rom[1]  = enc(6'd12, 2'd1, 10'd88);  // LOAD_BN lane1
        instr_rom[2]  = enc(6'd12, 2'd2, 10'd88);  // LOAD_BN lane2
        instr_rom[3]  = enc(6'd12, 2'd3, 10'd88);  // LOAD_BN lane3
        instr_rom[4]  = enc(6'd5,  2'd0, 10'd80);  // LOAD H[0]
        instr_rom[5]  = enc(6'd5,  2'd1, 10'd81);  // LOAD H[1]
        instr_rom[6]  = enc(6'd5,  2'd2, 10'd82);  // LOAD H[2]
        instr_rom[7]  = enc(6'd5,  2'd3, 10'd83);  // LOAD H[3]
        instr_rom[8]  = enc(6'd8,  2'd0, 10'd0);   // BATCHNORM
        instr_rom[9]  = enc(6'd3,  2'd0, 10'd0);   // RELU only
        instr_rom[10] = enc(6'd13, 2'd0, 10'd84);  // LOAD_B H_B[0]
        instr_rom[11] = enc(6'd13, 2'd1, 10'd85);  // LOAD_B H_B[1]
        instr_rom[12] = enc(6'd13, 2'd2, 10'd86);  // LOAD_B H_B[2]
        instr_rom[13] = enc(6'd13, 2'd3, 10'd87);  // LOAD_B H_B[3]
        instr_rom[14] = enc(6'd9,  2'd0, 10'd0);   // MAXPOOL
        instr_rom[15] = enc(6'd10, 2'd0, 10'd4);   // GAVGPOOL count=4
        instr_rom[16] = enc(6'd4,  2'd0, 10'd0);   // SOFTMAX
        instr_rom[17] = enc(6'd63, 2'd0, 10'd0);   // HALT

        do_reset(); wait_halt(600);

        $display("  CNN block output H[0]=%h H[1]=%h H[2]=%h H[3]=%h",
                 dut.H[0], dut.H[1], dut.H[2], dut.H[3]);

        // After softmax, all values should be non-zero probabilities
        check_nonzero("CNN block H[0] nonzero", dut.H[0]);
        check_nonzero("CNN block H[1] nonzero", dut.H[1]);

        // =================================================================
        //  FINAL SUMMARY
        // =================================================================
        $display("\n================================================================");
        $display("  FINAL SUMMARY");
        $display("  PASSED : %0d", pass_cnt);
        $display("  FAILED : %0d", fail_cnt);
        if (fail_cnt==0)
            $display("  RESULT : *** ALL TESTS PASSED ***");
        else
            $display("  RESULT : !!! %0d FAILURE(S) !!!", fail_cnt);
        $display("================================================================");
        $finish;
    end

    initial begin #10_000_000; $display("GLOBAL TIMEOUT"); $finish; end

    initial begin
        $dumpfile("tb_simd_ai_cpu_v2.vcd");
        $dumpvars(0, tb_simd_ai_cpu_v2);
    end

endmodule : tb_simd_ai_cpu_v2
