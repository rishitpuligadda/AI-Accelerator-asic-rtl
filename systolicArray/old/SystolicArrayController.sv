// SRAM macro used in this design is part of the SKY130 open-source PDK.
// The macros are provided by the OpenRAM-generated SKY130 SRAM library.
// Repository: https://github.com/VLSIDA/sky130_sram_macros

// ============================================================
//  sys_data_dim_sram
//
//  FSM phases:
//   S_IDLE       : wait one cycle after reset
//   S_LOAD_X     : read all X elements from SRAM into x_reg[][]
//                  (one element per cycle via Port 0)
//   S_LOAD_Y     : read all Y elements from SRAM into y_reg[][]
//                  (one element per cycle via Port 1)
//   S_RUN        : feed skewed tokens from registers into
//                  systolic array for TOTAL_CYCLES cycles
//   S_DRAIN      : one extra cycle so last token fully accumulates
//   S_WRITE_Z_LO : write Z[i][j][31:0]  to SRAM (one/cycle)
//   S_WRITE_Z_HI : write Z[i][j][63:32] to SRAM (one/cycle)
//   S_DONE       : assert done
// ============================================================
module SystolicController#(
    parameter s1     = 2,
    parameter M      = 2,
    parameter s2     = 2,
    parameter X_BASE = 8'd0,
    parameter Y_BASE = 8'(X_BASE + s1*M),
    parameter Z_BASE = 8'(Y_BASE + M*s2)
)(
    input  logic clk,
    input  logic rst,
    output logic done,

    // Port 0 : RW
    output logic        sram_cs0_o,
    output logic        sram_we0_o,
    output logic [3:0]  sram_wmask0_o,
    output logic [7:0]  sram_addr0_o,
    output logic [31:0] sram_din0_o,
    input  logic [31:0] sram_dout0_i,

    // Port 1 : R
    output logic        sram_cs1_o,
    output logic [7:0]  sram_addr1_o,
    input  logic [31:0] sram_dout1_i
);

    // ----------------------------------------------------------
    // Local register files ? loaded from SRAM before running
    // ----------------------------------------------------------
    logic signed [31:0] x_reg [s1][M];
    logic signed [31:0] y_reg [M][s2];

    // Systolic array ports
    logic signed [31:0] a [s2];
    logic signed [31:0] b [s1];
    logic signed [63:0] z [s1][s2];

    systolic #(.s1(s1), .s2(s2)) array (
        .clk(clk), .rst(rst), .c(z), .a(a), .b(b)
    );

    // ----------------------------------------------------------
    // Constants
    // ----------------------------------------------------------
    localparam int TOTAL_CYCLES = M + s1 + s2 - 2;
    localparam int X_WORDS      = s1 * M;
    localparam int Y_WORDS      = M  * s2;

    // ----------------------------------------------------------
    // FSM state encoding
    // ----------------------------------------------------------
    typedef enum logic [2:0] {
        S_IDLE       = 3'd0,
        S_LOAD_X     = 3'd1,
        S_LOAD_Y     = 3'd2,
        S_RUN        = 3'd3,
        S_DRAIN      = 3'd4,
        S_WRITE_Z_LO = 3'd5,
        S_WRITE_Z_HI = 3'd6,
        S_DONE       = 3'd7
    } state_t;

    state_t state;

    integer cnt;        // element counter for load phases / time step for RUN
    integer wi, wj;     // row/col write-back indices
    integer ii;         // loop variable

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            cnt   <= 0;
            wi    <= 0;
            wj    <= 0;
            done  <= 1'b0;
            for (ii = 0; ii < s2; ii++) a[ii] <= 32'sd0;
            for (ii = 0; ii < s1; ii++) b[ii] <= 32'sd0;
        end
        else begin
            case (state)

                // ---- Wait one cycle then start X load -----------
                S_IDLE: begin
                    cnt   <= 0;
                    state <= S_LOAD_X;
                end

                // ---- Load X from SRAM via Port 0 ----------------
                // Cycle cnt=0 : address X[0][0] is presented to SRAM
                // Cycle cnt=1 : dout holds X[0][0] -> capture it
                //               address X[0][1] is presented
                // ...
                // Cycle cnt=X_WORDS : capture last element, done
                S_LOAD_X: begin
                    if (cnt >= 1) begin
                        x_reg[(cnt-1)/M][(cnt-1)%M] <= $signed(sram_dout0_i);
                    end
                    if (cnt == X_WORDS) begin
                        cnt   <= 0;
                        state <= S_LOAD_Y;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ---- Load Y from SRAM via Port 1 ----------------
                S_LOAD_Y: begin
                    if (cnt >= 1) begin
                        y_reg[(cnt-1)/s2][(cnt-1)%s2] <= $signed(sram_dout1_i);
                    end
                    if (cnt == Y_WORDS) begin
                        cnt   <= 0;
                        state <= S_RUN;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ---- Run systolic array -------------------------
                // Identical skew schedule to original sys_data_dim
                S_RUN: begin
                    for (ii = 0; ii < s2; ii++) begin
                        if (cnt - ii >= 0 && cnt - ii < M)
                            a[ii] <= y_reg[cnt-ii][ii];
                        else
                            a[ii] <= 32'sd0;
                    end
                    for (ii = 0; ii < s1; ii++) begin
                        if (cnt - ii >= 0 && cnt - ii < M)
                            b[ii] <= x_reg[ii][cnt-ii];
                        else
                            b[ii] <= 32'sd0;
                    end

                    if (cnt == TOTAL_CYCLES - 1) begin
                        cnt   <= 0;
                        state <= S_DRAIN;
                    end else begin
                        cnt <= cnt + 1;
                    end
                end

                // ---- Drain: zero inputs, let last token settle --
                S_DRAIN: begin
                    for (ii = 0; ii < s2; ii++) a[ii] <= 32'sd0;
                    for (ii = 0; ii < s1; ii++) b[ii] <= 32'sd0;
                    wi    <= 0;
                    wj    <= 0;
                    state <= S_WRITE_Z_LO;
                end

                // ---- Write Z[wi][wj][31:0] to SRAM --------------
                // Address is driven combinatorially this cycle;
                // advance indices so next cycle writes the next element
                S_WRITE_Z_LO: begin
                    if (wj == s2 - 1) begin
                        wj <= 0;
                        if (wi == s1 - 1) begin
                            wi    <= 0;
                            wj    <= 0;
                            state <= S_WRITE_Z_HI;
                        end else begin
                            wi <= wi + 1;
                        end
                    end else begin
                        wj <= wj + 1;
                    end
                end

                // ---- Write Z[wi][wj][63:32] to SRAM -------------
                S_WRITE_Z_HI: begin
                    if (wj == s2 - 1) begin
                        wj <= 0;
                        if (wi == s1 - 1) begin
                            state <= S_DONE;
                        end else begin
                            wi <= wi + 1;
                        end
                    end else begin
                        wj <= wj + 1;
                    end
                end

                // ---- All done -----------------------------------
                S_DONE: begin
                    done <= 1'b1;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

    // ----------------------------------------------------------
    // Combinational SRAM control
    // Address and data are presented in the same cycle as the
    // FSM state, captured by the SRAM on posedge, data valid
    // the following cycle on negedge+DELAY (captured in FSM above)
    // ----------------------------------------------------------
    always_comb begin
        sram_cs0_o    = 1'b0;
        sram_we0_o    = 1'b0;
        sram_wmask0_o = 4'b1111;
        sram_addr0_o  = 8'd0;
        sram_din0_o   = 32'd0;
        sram_cs1_o    = 1'b0;
        sram_addr1_o  = 8'd0;

        case (state)

            // Read X[cnt] via Port 0
            S_LOAD_X: begin
                if (cnt < X_WORDS) begin
                    sram_cs0_o   = 1'b1;
                    sram_we0_o   = 1'b0;
                    sram_addr0_o = X_BASE + 8'(cnt);
                end
            end

            // Read Y[cnt] via Port 1
            S_LOAD_Y: begin
                if (cnt < Y_WORDS) begin
                    sram_cs1_o   = 1'b1;
                    sram_addr1_o = Y_BASE + 8'(cnt);
                end
            end

            // Write Z[wi][wj] lo-word via Port 0
            S_WRITE_Z_LO: begin
                sram_cs0_o    = 1'b1;
                sram_we0_o    = 1'b1;
                sram_wmask0_o = 4'b1111;
                sram_addr0_o  = Z_BASE + 8'((wi * s2 + wj) * 2);
                sram_din0_o   = z[wi][wj][31:0];
            end

            // Write Z[wi][wj] hi-word via Port 0
            S_WRITE_Z_HI: begin
                sram_cs0_o    = 1'b1;
                sram_we0_o    = 1'b1;
                sram_wmask0_o = 4'b1111;
                sram_addr0_o  = Z_BASE + 8'((wi * s2 + wj) * 2 + 1);
                sram_din0_o   = z[wi][wj][63:32];
            end

            default: ;
        endcase
    end

endmodule


// ============================================================
//  Full integration: Controller + SRAM
// ============================================================
module systolic_sram_top #(
    parameter s1 = 2,
    parameter M  = 2,
    parameter s2 = 2
)(
    input  logic clk,
    input  logic rst,
    output logic done
);
    logic        cs0, we0;
    logic [3:0]  wmask0;
    logic [7:0]  addr0;
    logic [31:0] din0, dout0;

    logic        cs1;
    logic [7:0]  addr1;
    logic [31:0] dout1;

    SystolicController#(.s1(s1), .M(M), .s2(s2)) ctrl (
        .clk          (clk),
        .rst          (rst),
        .done         (done),
        .sram_cs0_o   (cs0),
        .sram_we0_o   (we0),
        .sram_wmask0_o(wmask0),
        .sram_addr0_o (addr0),
        .sram_din0_o  (din0),
        .sram_dout0_i (dout0),
        .sram_cs1_o   (cs1),
        .sram_addr1_o (addr1),
        .sram_dout1_i (dout1)
    );

    sky130_sram_1kbyte_1rw1r_32x256_8 sram (
        .clk0   (clk),
        .csb0   (~cs0),
        .web0   (~we0),
        .wmask0 (wmask0),
        .addr0  (addr0),
        .din0   (din0),
        .dout0  (dout0),
        .clk1   (clk),
        .csb1   (~cs1),
        .addr1  (addr1),
        .dout1  (dout1)
    );
endmodule
