module feed_sequencer #(
    parameter int TILE_MAX = 4,
    parameter int REAL_M   = 6,
    parameter int AW       = 8
)(
    input  logic clk,
    input  logic rst,

    // Handshake
    input  logic start_i,
    output logic done_o,
    output logic busy_o,

    // Tile geometry (stable from start_i to done_o)
    input  logic [$clog2(TILE_MAX+1)-1:0] eff_rows_i,
    input  logic [$clog2(TILE_MAX+1)-1:0] eff_cols_i,

    // Register file inputs (from sram_shadow_reader)
    input  logic signed [7:0] x_rf_i [TILE_MAX][REAL_M],
    input  logic signed [7:0] y_rf_i [REAL_M][TILE_MAX],

    // Systolic array feeds
    output logic signed [7:0] a_feed_o [TILE_MAX],
    output logic signed [7:0] b_feed_o [TILE_MAX],

    // Reset to systolic array (hold high for one cycle before feeding)
    output logic local_rst_o
);

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        FS_IDLE     = 3'd0,
        FS_RESET    = 3'd1,
        FS_DEASSERT = 3'd2,   // one cycle: local_rst goes low, feeds driven but not yet accumulated
        FS_RUN      = 3'd3,
        FS_DONE     = 3'd4
    } fs_t;
    fs_t fs_state;

    logic [5:0] run_cnt;
    logic [5:0] capture_t;

    // Latched geometry
    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_rows, lat_eff_cols;

    // -------------------------------------------------------------------------
    // Sequential
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            fs_state     <= FS_IDLE;
            run_cnt      <= '0;
            capture_t    <= '0;
            lat_eff_rows <= '0;
            lat_eff_cols <= '0;
            done_o       <= 1'b0;
            busy_o       <= 1'b0;
            local_rst_o  <= 1'b1;   // array starts in reset at power-on
        end else begin
            done_o <= 1'b0;

            case (fs_state)

                FS_IDLE: begin
                    busy_o <= 1'b0;
                    // FIX: local_rst_o is NOT re-asserted here.
                    // The array holds its last tile's accumulated values so
                    // output_writer can read tile_z after fs_done.
                    // local_rst is only pulsed in FS_RESET at the start of
                    // each new tile, safely clearing accumulators before new
                    // data feeds in.
                    if (start_i) begin
                        lat_eff_rows <= eff_rows_i;
                        lat_eff_cols <= eff_cols_i;
                        capture_t    <= 6'(REAL_M + int'(eff_rows_i) + int'(eff_cols_i) - 2);
                        run_cnt      <= '0;
                        busy_o       <= 1'b1;
                        fs_state     <= FS_RESET;
                    end
                end

                // Assert reset to systolic array for one cycle so accumulators
                // are cleared before the new tile's data starts flowing in
                FS_RESET: begin
                    local_rst_o <= 1'b1;
                    fs_state    <= FS_DEASSERT;
                    run_cnt     <= '0;
                end

                // Deassert reset: local_rst_o goes low this cycle.
                // The MAC will see rst=1 until end of this cycle, then
                // rst=0 is registered. First accumulation happens in FS_RUN.
                FS_DEASSERT: begin
                    local_rst_o <= 1'b0;
                    fs_state    <= FS_RUN;
                    run_cnt     <= '0;
                end

                FS_RUN: begin
                    local_rst_o <= 1'b0;
                    if (run_cnt == capture_t) begin
                        done_o   <= 1'b1;
                        busy_o   <= 1'b0;
                        fs_state <= FS_DONE;
                    end else begin
                        run_cnt <= run_cnt + 1'b1;
                    end
                end

                FS_DONE: begin
                    done_o   <= 1'b0;
                    fs_state <= FS_IDLE;
                end

                default: fs_state <= FS_IDLE;

            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Combinational feed generation
    //
    // a_feed[j] = y_rf[t-j][j]  if (t-j) in [0, REAL_M) and j < eff_cols
    //           = 0              otherwise
    //
    // b_feed[i] = x_rf[i][t-i]  if (t-i) in [0, REAL_M) and i < eff_rows
    //           = 0              otherwise
    //
    // where t = run_cnt
    // -------------------------------------------------------------------------
    always_comb begin
        for (int j = 0; j < TILE_MAX; j++) begin
            automatic int t_minus_j = int'(run_cnt) - j;
            if ((fs_state == FS_RUN || fs_state == FS_DEASSERT) &&
                j < int'(lat_eff_cols) &&
                t_minus_j >= 0 && t_minus_j < REAL_M)
                a_feed_o[j] = y_rf_i[t_minus_j][j];
            else
                a_feed_o[j] = 8'sd0;
        end

        for (int i = 0; i < TILE_MAX; i++) begin
            automatic int t_minus_i = int'(run_cnt) - i;
            if ((fs_state == FS_RUN || fs_state == FS_DEASSERT) &&
                i < int'(lat_eff_rows) &&
                t_minus_i >= 0 && t_minus_i < REAL_M)
                b_feed_o[i] = x_rf_i[i][t_minus_i];
            else
                b_feed_o[i] = 8'sd0;
        end
    end

endmodule
