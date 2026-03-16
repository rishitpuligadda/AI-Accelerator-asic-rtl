`timescale 1ns/1ps
// =============================================================================
//  SIMD Softmax ? ModelSim 10.5b, NO array ports whatsoever
//
//  FINAL ROOT CAUSE SUMMARY:
//  ModelSim 10.5b cannot correctly index unpacked array ports in ANY context:
//    - always_ff NBA:          arr_b <= arr_a[i]  ? always reads [0] or [N-1]
//    - module-level assign:    assign w = port[i]  ? always reads [N-1]
//    - generate+assign:        assign w = port[ii] ? always reads [N-1]
//
//  SOLUTION: Use ONLY scalar ports. Both x_i and prob output are 8 individual
//  scalar signals. The top-level wrapper (if needed for integration) can
//  map scalar ports to arrays externally.
//
//  ALSO FIXED: LUT stored e^+k (wrong). It must store e^-k because all
//  inputs to exp_approx are negative (post max-subtraction).
//  exp(x) = LUT[|floor(x)|] × poly(frac)  where LUT[k] = e^(-k)
// =============================================================================
package softmax_pkg;
  parameter int N_LANES    = 8;
  parameter int DATA_W     = 16;
  parameter int FRAC_W     = 8;
  parameter int EXP_W      = 24;
  parameter int EXP_FRAC_W = 16;

  function automatic logic [EXP_W-1:0] exp_lut(input int k);
    // e^(-k) * 2^16 ? correct values for NEGATIVE inputs
    logic [EXP_W-1:0] LUT [0:15];
    LUT[0]  = 24'h010000; // e^-0  = 1.000000 * 65536 = 65536
    LUT[1]  = 24'h005E2D; // e^-1  = 0.367879 * 65536 = 24109
    LUT[2]  = 24'h0022A5; // e^-2  = 0.135335 * 65536 = 8869
    LUT[3]  = 24'h000CBE; // e^-3  = 0.049787 * 65536 = 3262
    LUT[4]  = 24'h0004B0; // e^-4  = 0.018316 * 65536 = 1200
    LUT[5]  = 24'h0001B9; // e^-5  = 0.006738 * 65536 = 441
    LUT[6]  = 24'h0000A2; // e^-6  = 0.002479 * 65536 = 162
    LUT[7]  = 24'h00003B; // e^-7  = 0.000912 * 65536 = 59
    LUT[8]  = 24'h000015; // e^-8  = 0.000335 * 65536 = 21
    LUT[9]  = 24'h000008; // e^-9  = 0.000123 * 65536 = 8
    LUT[10] = 24'h000002; // e^-10 = 0.000045 * 65536 = 2
    LUT[11] = 24'h000001; // e^-11 = 0.000017 * 65536 = 1
    LUT[12] = 24'h000000; // e^-12..15: negligible
    LUT[13] = 24'h000000;
    LUT[14] = 24'h000000;
    LUT[15] = 24'h000000;
    return LUT[k];
  endfunction
endpackage : softmax_pkg

// =============================================================================
//  exp_approx_lane ? scalar I/O, 3-cycle pipeline
//  Input x_i is post max-subtract, always in [-15, 0].
//  exp(x) = LUT[|floor(x)|] * (1 + f + f²/2 + f³/6 + f?/24)
//  where LUT[k] = e^(-k) and f = x - floor(x) in [0, 1)
// =============================================================================
module exp_approx_lane
  import softmax_pkg::*;
#(parameter int DW=DATA_W, parameter int FW=FRAC_W,
  parameter int OW=EXP_W,  parameter int OFW=EXP_FRAC_W)
(
  input  logic              clk, rst_n, valid_i,
  input  logic signed [DW-1:0] x_i,
  output logic              valid_o,
  output logic [OW-1:0]     exp_o
);
  localparam logic signed [DW-1:0] CLAMP_NEG = DW'(-15*(1<<FW));
  localparam logic [FW-1:0] HALF = FW'(1<<(FW-1));
  localparam logic [FW-1:0] SIXTH = FW'((1<<FW)/6);
  localparam logic [FW-1:0] T24 = FW'((1<<FW)/24);

  // Stage 1: clamp and register
  // IMPORTANT: use $signed cast and explicit 0 for signed comparison.
  // In ModelSim 10.5b, comparing against {DW{1'b0}} treats as unsigned,
  // making all negative values appear > 0. Use 16'sh0 (signed zero) instead.
  localparam logic signed [DW-1:0] ZERO = '0;
  logic signed [DW-1:0] xc; logic vc1;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin vc1<=0; xc<=0; end
    else begin
      vc1 <= valid_i;
      if      (x_i < CLAMP_NEG) xc <= CLAMP_NEG;
      else if (x_i > ZERO)      xc <= ZERO;
      else                       xc <= x_i;
      if (valid_i) $display("[EXP@%0t] x_i=%0d -> xc=%0d", $time, x_i,
        (x_i<CLAMP_NEG)?CLAMP_NEG:(x_i>ZERO)?ZERO:x_i);
    end
  end

  // Combinational: split into integer k and fractional part
  // For x in [-15,0]: floor(x) rounds toward -infinity (NOT toward zero).
  // k = |floor(x)| = ceiling(|x| / 2^FW) = ((-xc) + 2^FW - 1) >> FW
  // floor_x = -(k * 2^FW)  [as signed]
  // frac = xc - floor_x = xc + k*2^FW  (always in [0, 2^FW))
  localparam logic [FW-1:0] FW_MASK = {FW{1'b1}};  // (1<<FW)-1 = 255 for FW=8
  logic signed [DW-1:0] floor_x;
  logic [3:0]  k;
  logic [FW-1:0] frac;
  assign k      = 4'(($unsigned(-xc) + FW_MASK) >> FW);  // ceiling div = |floor(x)|
  assign floor_x = -(DW'(k) << FW);                       // floor(x) as signed Q8.8
  assign frac   = FW'($unsigned(xc - floor_x));            // frac in [0,1), Q0.FW

  // Combinational: 4th-order Taylor series for exp(frac), frac in [0,1)
  // All terms in Q0.(2*FW) = Q0.16 format.
  // CRITICAL: multiplications must be explicitly widened.
  // In SV, N-bit * M-bit = max(N,M)-bit (truncated!).
  // f2[2*FW-1:FW] is FW=8 bits, HALF/SIXTH/T24 are FW=8 bits.
  // 8*8=8 bit result ? WRONG. Must cast to wider type first.
  logic [2*FW-1:0] f2, f3, f4;
  logic [2*FW+1:0] t1, t2, t3, t4;
  assign f2 = frac * frac;                                    // 8*8->16 bit OK
  assign f3 = f2[2*FW-1:FW] * frac;                          // 8*8->16 bit OK (used for f3)
  assign f4 = f3[2*FW-1:FW] * frac;                          // 8*8->16 bit OK
  assign t1 = {{2{1'b0}}, frac, {FW{1'b0}}};                 // f in Q0.(2*FW)
  // Widen to [2*FW-1:0] before multiply to get full 16-bit products
  assign t2 = {2'b00, (2*FW)'(f2[2*FW-1:FW]) * (2*FW)'(HALF)};   // f²/2
  assign t3 = {2'b00, (2*FW)'(f3[2*FW-1:FW]) * (2*FW)'(SIXTH)};  // f³/6
  assign t4 = {2'b00, (2*FW)'(f4[2*FW-1:FW]) * (2*FW)'(T24)};    // f?/24

  // Stage 2: LUT base + polynomial (registered)
  // poly accumulates in Q0.(2*FW): base constant = 1<<(2*FW) = 65536
  logic [OW-1:0]    base;
  logic [2*FW+3:0]  poly;  // Q0.(2*FW), max ? e * 2^16 < 2^(2*FW+2)
  logic vc2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin vc2<=0; base<=0; poly<=0; end
    else begin
      vc2  <= vc1;
      base <= exp_lut(int'(k));
      // 1.0 in Q0.(2*FW) = 1<<(2*FW) = 65536
      poly <= (2*FW+4)'(1 << (2*FW)) + (2*FW+4)'(t1)
            + (2*FW+4)'(t2) + (2*FW+4)'(t3) + (2*FW+4)'(t4);
    end
  end

  // Stage 3: base × poly, right-shift to normalise (registered)
  // base: Q8.OFW (24 bits)
  // poly: Q0.(2*FW) (20 bits, ? e*65536 ? 178145)
  // prod: Q8.(OFW+2*FW) ? shift right by 2*FW ? Q8.OFW ? take low OW bits
  logic [OW+2*FW+3:0] prod;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin valid_o<=0; exp_o<=0; end
    else begin
      valid_o  <= vc2;
      prod      = (OW+2*FW+4)'(base) * (OW+2*FW+4)'(poly);
      exp_o    <= OW'(prod >> (2*FW));  // shift by 2*FW (not FW)
      if (vc2) $display("[EXP@%0t] base=%0d poly=%0d -> exp_o=0x%06h",
        $time, base, poly, OW'(prod>>(2*FW)));
    end
  end
endmodule : exp_approx_lane

// =============================================================================
//  simd_softmax ? ALL scalar ports, no unpacked arrays at any port boundary
// =============================================================================
module simd_softmax
  import softmax_pkg::*;
#(parameter int N=N_LANES, parameter int DW=DATA_W, parameter int FW=FRAC_W,
  parameter int OW=EXP_W,  parameter int OFW=EXP_FRAC_W)
(
  input  logic              clk, rst_n, valid_i,
  // 8 scalar inputs ? no array port (ModelSim 10.5b array port indexing broken)
  input  logic signed [DW-1:0] i0, i1, i2, i3, i4, i5, i6, i7,
  output logic              valid_o,
  // 8 scalar outputs ? no array port
  output logic [OW-1:0]    p0, p1, p2, p3, p4, p5, p6, p7
);

  // =========================================================================
  //  STAGE A: Max reduction ? 3 registered pipeline levels
  // =========================================================================
  logic signed [DW-1:0] m1a,m1b,m1c,m1d; logic mv1;
  logic signed [DW-1:0] m2a,m2b;          logic mv2;
  logic signed [DW-1:0] xmax;             logic mv3;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin mv1<=0; m1a<=0;m1b<=0;m1c<=0;m1d<=0; end
    else begin
      mv1<=valid_i;
      m1a<=(i0>i1)?i0:i1; m1b<=(i2>i3)?i2:i3;
      m1c<=(i4>i5)?i4:i5; m1d<=(i6>i7)?i6:i7;
      if (valid_i) $display("[MAX@%0t] i0..i7=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
        $time,i0,i1,i2,i3,i4,i5,i6,i7);
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

  // =========================================================================
  //  Input delay: 3 explicit scalar register stages (align with max output)
  // =========================================================================
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

  // =========================================================================
  //  STAGE B: Subtract max ? 1 registered cycle
  // =========================================================================
  logic signed [DW-1:0] xs0,xs1,xs2,xs3,xs4,xs5,xs6,xs7; logic sv;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin sv<=0;xs0<=0;xs1<=0;xs2<=0;xs3<=0;xs4<=0;xs5<=0;xs6<=0;xs7<=0; end
    else begin
      sv<=mv3;
      xs0<=d3_0-xmax; xs1<=d3_1-xmax; xs2<=d3_2-xmax; xs3<=d3_3-xmax;
      xs4<=d3_4-xmax; xs5<=d3_5-xmax; xs6<=d3_6-xmax; xs7<=d3_7-xmax;
      if (mv3) $display("[SUB@%0t] d3=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d xmax=%0d xs=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
        $time,d3_0,d3_1,d3_2,d3_3,d3_4,d3_5,d3_6,d3_7,xmax,
        d3_0-xmax,d3_1-xmax,d3_2-xmax,d3_3-xmax,
        d3_4-xmax,d3_5-xmax,d3_6-xmax,d3_7-xmax);
    end
  end

  // =========================================================================
  //  STAGE C: 8 explicit exp_approx_lane instances ? scalar connections only
  // =========================================================================
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

  // =========================================================================
  //  STAGE D: Sum reduction ? 3 registered levels
  // =========================================================================
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

  // =========================================================================
  //  Exp delay: 5 scalar register stages (C+7 ? C+12, aligns with NR E2)
  // =========================================================================
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

  // =========================================================================
  //  STAGE E: Newton-Raphson reciprocal ? 2 registered cycles
  // =========================================================================
  logic [SW-1:0]   nr_sr; logic [OW-1:0] nr_seed,nr_recip;
  logic            nr_e1,nr_e2;
  logic [2*OW-1:0] nr_sp,nr_fl; logic [OW-1:0] nr_2m;
  assign nr_sp = (2*OW)'(nr_sr)   * (2*OW)'(nr_seed);
  assign nr_2m = OW'(2<<OFW)      - OW'(nr_sp>>OFW);
  assign nr_fl = (2*OW)'(nr_seed) * (2*OW)'(nr_2m);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin nr_e1<=0;nr_sr<=0;nr_seed<=0; end
    else begin
      nr_e1<=ss3; nr_sr<=esum;
      if (esum==0) nr_seed<={OW{1'b1}};
      else         nr_seed<=OW'(64'h1_0000_0000/{32'b0,esum});
      if (ss3) $display("[NR@%0t] esum=%0d seed=%0d", $time, esum,
        (esum==0)?{OW{1'b1}}:OW'(64'h1_0000_0000/{32'b0,esum}));
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin nr_e2<=0;nr_recip<=0; end
    else begin nr_e2<=nr_e1; nr_recip<=OW'(nr_fl>>OFW); end
  end

  // =========================================================================
  //  STAGE F: Scale ? 1 registered cycle, hold on invalid cycles
  // =========================================================================
  logic vd; assign vd = nr_e2 & evd5;
  logic [2*OW-1:0] sc0,sc1,sc2,sc3,sc4,sc5,sc6,sc7;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_o<=0; p0<=0;p1<=0;p2<=0;p3<=0;p4<=0;p5<=0;p6<=0;p7<=0;
    end else begin
      valid_o<=vd;
      if (vd) begin
        sc0=(2*OW)'(e5_0)*(2*OW)'(nr_recip); p0<=OW'(sc0>>OFW);
        sc1=(2*OW)'(e5_1)*(2*OW)'(nr_recip); p1<=OW'(sc1>>OFW);
        sc2=(2*OW)'(e5_2)*(2*OW)'(nr_recip); p2<=OW'(sc2>>OFW);
        sc3=(2*OW)'(e5_3)*(2*OW)'(nr_recip); p3<=OW'(sc3>>OFW);
        sc4=(2*OW)'(e5_4)*(2*OW)'(nr_recip); p4<=OW'(sc4>>OFW);
        sc5=(2*OW)'(e5_5)*(2*OW)'(nr_recip); p5<=OW'(sc5>>OFW);
        sc6=(2*OW)'(e5_6)*(2*OW)'(nr_recip); p6<=OW'(sc6>>OFW);
        sc7=(2*OW)'(e5_7)*(2*OW)'(nr_recip); p7<=OW'(sc7>>OFW);
        $display("[SCL@%0t] e5=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d recip=%0d",
          $time,e5_0,e5_1,e5_2,e5_3,e5_4,e5_5,e5_6,e5_7,nr_recip);
        $display("[SCL@%0t] p0..p7=0x%06h,0x%06h,0x%06h,0x%06h,0x%06h,0x%06h,0x%06h,0x%06h",
          $time,OW'(sc0>>OFW),OW'(sc1>>OFW),OW'(sc2>>OFW),OW'(sc3>>OFW),
               OW'(sc4>>OFW),OW'(sc5>>OFW),OW'(sc6>>OFW),OW'(sc7>>OFW));
      end
    end
  end

endmodule : simd_softmax
// =============================================================================
//  Testbench for simd_softmax ? scalar output ports p0..p7
//
//  IMPORTANT ? how to get a genuinely fresh simulation in ModelSim GUI:
//    In the Transcript window type:
//      quit -sim
//      vdel -lib work -all
//      vlib work
//      vlog -sv -work work -stats=none C:/intelFPGA/18.1/softmax_simd.sv
//      vsim work.tb_simd_softmax_full
//      run -all
//
//  Or from Windows command prompt:
//      cd C:\intelFPGA\18.1
//      vdel -lib work -all
//      vlib work
//      vlog -sv -work work -stats=none softmax_simd.sv
//      vsim -c work.tb_simd_softmax_full -do "run -all; quit"
// =============================================================================
`timescale 1ns/1ps

module tb_simd_softmax_full;
  import softmax_pkg::*;

  localparam int N   = N_LANES;
  localparam int DW  = DATA_W;
  localparam int FW  = FRAC_W;
  localparam int OW  = EXP_W;
  localparam int OFW = EXP_FRAC_W;
  localparam int MAX_WAIT = 60;
  localparam real REL_TOL = 0.05;
  localparam real ABS_TOL = 4.0 / 65536.0;

  // ---- DUT ports ? ALL scalar, no arrays ---------------------------------
  logic clk=0, rst_n=0, valid_i=0;
  // 8 scalar inputs (no array port ? ModelSim 10.5b array port indexing broken)
  logic signed [DW-1:0] xi0,xi1,xi2,xi3,xi4,xi5,xi6,xi7;
  logic valid_o;
  logic [OW-1:0] p0,p1,p2,p3,p4,p5,p6,p7;

  simd_softmax dut (
    .clk(clk), .rst_n(rst_n), .valid_i(valid_i),
    .i0(xi0),.i1(xi1),.i2(xi2),.i3(xi3),
    .i4(xi4),.i5(xi5),.i6(xi6),.i7(xi7),
    .valid_o(valid_o),
    .p0(p0),.p1(p1),.p2(p2),.p3(p3),.p4(p4),.p5(p5),.p6(p6),.p7(p7)
  );

  always #5 clk = ~clk;

  // ---- Helpers -------------------------------------------------------------
  function automatic logic signed [DW-1:0] toQ(input real v);
    int t; t=int'(v*real'(1<<FW)); return DW'(t);
  endfunction

  function automatic real fromQ(input logic [OW-1:0] v);
    return real'(v)/real'(1<<OFW);
  endfunction

  function automatic real fabsr(input real v);
    return (v<0.0)?-v:v;
  endfunction

  // ---- Collect scalar outputs into an array for checking -------------------
  // This is a local variable assignment in a task ? safe, not a port read
  task automatic capture_outputs(output real got[N]);
    got[0]=fromQ(p0); got[1]=fromQ(p1); got[2]=fromQ(p2); got[3]=fromQ(p3);
    got[4]=fromQ(p4); got[5]=fromQ(p5); got[6]=fromQ(p6); got[7]=fromQ(p7);
  endtask

  // ---- Per-lane check ------------------------------------------------------
  function automatic bit check_lane(
    input int  tid,
    input int  lane,
    input real got,
    input real expected
  );
    real err, tol;
    err = fabsr(got-expected);
    tol = fabsr(expected)*REL_TOL;
    if (tol < ABS_TOL) tol = ABS_TOL;
    if (err<=tol) begin
      $display("  [PASS] T%0d lane[%0d] got=%f exp=%f err=%f", tid,lane,got,expected,err);
      return 1'b1;
    end else begin
      $display("  [FAIL] T%0d lane[%0d] got=%f exp=%f err=%f (tol=%f)", tid,lane,got,expected,err,tol);
      return 1'b0;
    end
  endfunction

  // ---- Test vector type ----------------------------------------------------
  typedef struct { real inp[N]; real expv[N]; string name; } tv_t;
  tv_t tests[6];
  int  pass_cnt, fail_cnt;

  // ---- run_test task -------------------------------------------------------
  task automatic run_test(input tv_t tv, input int tid);
    real got[N];
    real sum_got;
    int  wait_cnt;
    bit  timed_out;
    int  tp, tf;

    // Drive: set scalar inputs on negedge, valid_i=1, captured on next posedge
    @(negedge clk);
    xi0=toQ(tv.inp[0]); xi1=toQ(tv.inp[1]); xi2=toQ(tv.inp[2]); xi3=toQ(tv.inp[3]);
    xi4=toQ(tv.inp[4]); xi5=toQ(tv.inp[5]); xi6=toQ(tv.inp[6]); xi7=toQ(tv.inp[7]);
    valid_i = 1'b1;
    $display("[TB] Test %0d: xi(Q8.8)=%0d %0d %0d %0d %0d %0d %0d %0d",
             tid,xi0,xi1,xi2,xi3,xi4,xi5,xi6,xi7);
    @(posedge clk);
    @(negedge clk);
    valid_i = 1'b0;

    // Wait for valid_o pulse
    timed_out=0; wait_cnt=0;
    @(posedge clk);
    while (!valid_o && wait_cnt < MAX_WAIT) begin
      @(posedge clk); wait_cnt++;
    end

    // Advance one more posedge: prob outputs NBA committed at valid_o posedge,
    // reading in same active region sees old value. Next posedge sees new value.
    if (valid_o) @(posedge clk);

    if (!valid_o && wait_cnt>=MAX_WAIT) begin
      timed_out=1;
      $display("\n--- Test %0d: %s [TIMEOUT] ---", tid, tv.name);
    end else begin
      $display("\n--- Test %0d: %s (valid_o +%0d cycles) ---", tid, tv.name, wait_cnt+2);
    end

    $display("  inp=[%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f]",
             tv.inp[0],tv.inp[1],tv.inp[2],tv.inp[3],
             tv.inp[4],tv.inp[5],tv.inp[6],tv.inp[7]);
    $display("  p0..p7 (hex) = %06h %06h %06h %06h %06h %06h %06h %06h",
             p0,p1,p2,p3,p4,p5,p6,p7);

    tp=0; tf=0;
    if (timed_out) begin
      for (int i=0;i<N;i++) begin
        $display("  [FAIL] T%0d lane[%0d] TIMEOUT",tid,i); tf++;
      end
    end else begin
      capture_outputs(got);
      sum_got=0.0;
      for (int i=0;i<N;i++) sum_got+=got[i];
      for (int i=0;i<N;i++)
        if (check_lane(tid,i,got[i],tv.expv[i])) tp++; else tf++;
      $display("  Sum=%f (ideal 1.0)", sum_got);
      if (sum_got>0.985 && sum_got<1.015)
        $display("  [PASS] sum ok");
      else
        $display("  [FAIL] sum=%f out of range", sum_got);
    end
    pass_cnt+=tp; fail_cnt+=tf;
    repeat(MAX_WAIT+5) @(posedge clk);
  endtask

  // ---- Golden values -------------------------------------------------------
  task automatic init_tests();
    // Test 0: x=[1,2,0.5,-1,0,1.5,-0.5,3], max=3, shifted=[-2,-1,-2.5,-4,-3,-1.5,-3.5,0]
    tests[0].name="Baseline mixed";
    tests[0].inp='{1.0,2.0,0.5,-1.0,0.0,1.5,-0.5,3.0};
    tests[0].expv='{0.07096,0.19292,0.04305,0.00961,0.02612,0.11703,0.01584,0.52447};

    // Test 1: x=[0]*8, all uniform
    tests[1].name="All-zeros";
    tests[1].inp='{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    tests[1].expv='{0.125,0.125,0.125,0.125,0.125,0.125,0.125,0.125};

    // Test 2: x=[0]*7+[8], lane7 dominates
    tests[2].name="One-hot lane7=8";
    tests[2].inp='{0.0,0.0,0.0,0.0,0.0,0.0,0.0,8.0};
    tests[2].expv='{0.000334,0.000334,0.000334,0.000334,0.000334,0.000334,0.000334,0.997657};

    // Test 3: x=[-4,-3,-2,-1,-0.5,-1.5,-2.5,-3.5], max=-0.5
    tests[3].name="All-negative";
    tests[3].inp='{-4.0,-3.0,-2.0,-1.0,-0.5,-1.5,-2.5,-3.5};
    tests[3].expv='{0.01211,0.03291,0.08946,0.24315,0.40083,0.14745,0.05426,0.01995};

    // Test 4: x=[-1,0,1,-1,0,1,-1,0], symmetric pairs
    tests[4].name="Symmetric pairs";
    tests[4].inp='{-1.0,0.0,1.0,-1.0,0.0,1.0,-1.0,0.0};
    tests[4].expv='{0.03856,0.10481,0.28491,0.03856,0.10481,0.28491,0.03856,0.10481};

    // Test 5: x=[-7,-6,-5,-4,-3,-2,-1,0], large spread
    tests[5].name="Large spread";
    tests[5].inp='{-7.0,-6.0,-5.0,-4.0,-3.0,-2.0,-1.0,0.0};
    tests[5].expv='{0.000576,0.001567,0.004259,0.011578,0.031472,0.085535,0.232536,0.632177};
  endtask

  // ---- Main ----------------------------------------------------------------
  initial begin
    $display("================================================================");
    $display(" SIMD Softmax TB ? scalar ports p0..p7, N=%0d Q%0d.%0d->Q8.%0d",
             N,DW-FW,FW,OFW);
    $display(" Simulation start time: %0t", $time);
    $display("================================================================");

    xi0=0;xi1=0;xi2=0;xi3=0;xi4=0;xi5=0;xi6=0;xi7=0;
    rst_n=0; valid_i=0;
    repeat(8) @(posedge clk);
    @(negedge clk); rst_n=1;
    repeat(4) @(posedge clk);

    pass_cnt=0; fail_cnt=0;
    init_tests();
    for (int t=0;t<6;t++) run_test(tests[t],t);

    $display("\n================================================================");
    $display(" FINAL SUMMARY");
    $display("  Lanes  : %0d  PASSED: %0d  FAILED: %0d",
             pass_cnt+fail_cnt, pass_cnt, fail_cnt);
    if (fail_cnt==0) $display("  RESULT: *** ALL TESTS PASSED ***");
    else             $display("  RESULT: !!! %0d FAILURES !!!", fail_cnt);
    $display(" Simulation end time: %0t", $time);
    $display("================================================================");
    $finish;
  end

  initial begin #20_000_000; $display("GLOBAL TIMEOUT"); $finish; end

  initial begin
    $dumpfile("softmax_tb.vcd");
    $dumpvars(0, tb_simd_softmax_full);
  end

endmodule : tb_simd_softmax_full
