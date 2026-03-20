module feed_sequencer #(
    parameter int TILE_MAX = 4,
    parameter int REAL_M   = 9,
    parameter int AW       = 8
)(
    input  logic clk,
    input  logic rst,

    input  logic start_i,
    output logic done_o,
    output logic busy_o,

    input  logic [$clog2(TILE_MAX+1)-1:0] eff_rows_i,
    input  logic [$clog2(TILE_MAX+1)-1:0] eff_cols_i,

    input  logic signed [7:0] x_rf_i [TILE_MAX][REAL_M],
    input  logic signed [7:0] y_rf_i [REAL_M][TILE_MAX],

    output logic signed [7:0] a_feed_o [TILE_MAX],
    output logic signed [7:0] b_feed_o [TILE_MAX],

    output logic local_rst_o
);

    typedef enum logic [2:0] {
        FS_IDLE     = 3'd0,
        FS_RESET    = 3'd1,
        FS_DEASSERT = 3'd2,
        FS_RUN      = 3'd3,
        FS_DONE     = 3'd4
    } fs_t;
    fs_t fs_state;

    logic [5:0] run_cnt;
    logic [5:0] capture_t;

    logic [$clog2(TILE_MAX+1)-1:0] lat_eff_rows, lat_eff_cols;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            fs_state     <= FS_IDLE;
            run_cnt      <= '0;
            capture_t    <= '0;
            lat_eff_rows <= '0;
            lat_eff_cols <= '0;
            done_o       <= 1'b0;
            busy_o       <= 1'b0;
            local_rst_o  <= 1'b1;
        end else begin
            done_o <= 1'b0;

            case (fs_state)
                FS_IDLE: begin
                    busy_o <= 1'b0;
                    if (start_i) begin
                        lat_eff_rows <= eff_rows_i;
                        lat_eff_cols <= eff_cols_i;
                        capture_t    <= 6'(REAL_M + int'(eff_rows_i) + int'(eff_cols_i) - 2);
                        run_cnt      <= '0;
                        busy_o       <= 1'b1;
                        fs_state     <= FS_RESET;
                    end
                end

                FS_RESET: begin
                    local_rst_o <= 1'b1;
                    fs_state    <= FS_DEASSERT;
                    run_cnt     <= '0;
                end

                FS_DEASSERT: begin
                    local_rst_o <= 1'b0;
                    fs_state    <= FS_RUN;
                    run_cnt     <= '0;
                end

                FS_RUN: begin
                    local_rst_o <= 1'b0;
                    if (run_cnt <= 3) begin  // print first 4 cycles
                        $display("[FS @%0t] t=%0d a[0]=%0d a[1]=%0d b[0]=%0d b[1]=%0d b[2]=%0d b[3]=%0d",
                            $time, run_cnt,
                            $signed(a_feed_o[0]), $signed(a_feed_o[1]),
                            $signed(b_feed_o[0]), $signed(b_feed_o[1]),
                            $signed(b_feed_o[2]), $signed(b_feed_o[3]));
                    end
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
