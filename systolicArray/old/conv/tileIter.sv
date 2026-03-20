module tile_iter #(
    parameter int TILE_MAX = 4,
    parameter int REAL_K   = 4,
    parameter int REAL_M   = 9,
    parameter int REAL_N   = 1,
    parameter int AW       = 8
)(
    input  logic clk,
    input  logic rst,

    input  logic start_i,
    input  logic advance_i,
    output logic valid_o,
    output logic all_done_o,

    output logic [$clog2(REAL_K):0]        row_tile_o,
    output logic [$clog2(REAL_N):0]        col_tile_o,

    output logic [$clog2(TILE_MAX+1)-1:0]  eff_rows_o,
    output logic [$clog2(TILE_MAX+1)-1:0]  eff_cols_o,
    output logic [5:0]                     words_a_o,
    output logic [5:0]                     words_b_o,
    output logic [AW-1:0]                  sram_a_base_o
);

    function automatic int imin(input int a, input int b);
        return (a < b) ? a : b;
    endfunction

    function automatic int words_of(input int elements);
        return (elements + 3) / 4;
    endfunction

    logic [$clog2(REAL_K):0] row_tile;
    logic [$clog2(REAL_N):0] col_tile;

    logic [$clog2(TILE_MAX+1)-1:0] eff_rows, eff_cols;

    always_comb begin
        eff_rows = $bits(eff_rows)'(imin(REAL_K - int'(row_tile), TILE_MAX));
        eff_cols = $bits(eff_cols)'(imin(REAL_N - int'(col_tile), TILE_MAX));
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            row_tile   <= '0;
            col_tile   <= '0;
            valid_o    <= 1'b0;
            all_done_o <= 1'b0;
        end else begin
            if (start_i) begin
                row_tile   <= '0;
                col_tile   <= '0;
                valid_o    <= 1'b1;
                all_done_o <= 1'b0;
            end else if (advance_i && valid_o && !all_done_o) begin
                if (int'(col_tile) + int'(eff_cols) < REAL_N) begin
                    col_tile <= $bits(col_tile)'(int'(col_tile) + int'(eff_cols));
                end else if (int'(row_tile) + int'(eff_rows) < REAL_K) begin
                    row_tile <= $bits(row_tile)'(int'(row_tile) + int'(eff_rows));
                    col_tile <= '0;
                end else begin
                    valid_o    <= 1'b0;
                    all_done_o <= 1'b1;
                end
            end
        end
    end

    assign row_tile_o    = row_tile;
    assign col_tile_o    = col_tile;
    assign eff_rows_o    = eff_rows;
    assign eff_cols_o    = eff_cols;
    assign words_a_o     = 6'(words_of(int'(eff_rows) * REAL_M));
    assign words_b_o     = 6'(words_of(REAL_M * int'(eff_cols)));
    assign sram_a_base_o = AW'(int'(row_tile) * REAL_M / 4);

endmodule
