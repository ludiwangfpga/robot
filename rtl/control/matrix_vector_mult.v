// matrix_vector_mult.v
// 6x6 matrix × 6x1 vector: y = A * x
// ALL 6 rows computed in parallel (was 6 serial cycles)
// Latency: 1 cycle compute + 1 cycle done = 2 cycles (was 8)
// Uses 36 multipliers in parallel (6 rows × 6 elements)

`timescale 1ns / 1ps

(* use_dsp = "yes" *)
module matrix_vector_mult #(
    parameter MAT_WIDTH = 24,
    parameter VEC_WIDTH = 24,
    parameter MAT_FRAC = 19,
    parameter VEC_FRAC = 15
)(
    input wire clk,
    input wire rst,
    input wire start,

    // A matrix (6x6)
    input wire signed [MAT_WIDTH-1:0] a_00, a_01, a_02, a_03, a_04, a_05,
    input wire signed [MAT_WIDTH-1:0] a_10, a_11, a_12, a_13, a_14, a_15,
    input wire signed [MAT_WIDTH-1:0] a_20, a_21, a_22, a_23, a_24, a_25,
    input wire signed [MAT_WIDTH-1:0] a_30, a_31, a_32, a_33, a_34, a_35,
    input wire signed [MAT_WIDTH-1:0] a_40, a_41, a_42, a_43, a_44, a_45,
    input wire signed [MAT_WIDTH-1:0] a_50, a_51, a_52, a_53, a_54, a_55,

    // x vector (6x1)
    input wire signed [VEC_WIDTH-1:0] x_0, x_1, x_2, x_3, x_4, x_5,

    // y vector (6x1)
    output reg signed [VEC_WIDTH-1:0] y_0, y_1, y_2, y_3, y_4, y_5,

    output reg done
);

    localparam IDLE       = 2'd0;
    localparam COMPUTE    = 2'd1;
    localparam DONE_STATE = 2'd2;

    reg [1:0] state;

    // 6 parallel dot products — all rows computed simultaneously
    // Row 0
    wire signed [MAT_WIDTH+VEC_WIDTH-1:0] sum0 =
        a_00 * x_0 + a_01 * x_1 + a_02 * x_2 + a_03 * x_3 + a_04 * x_4 + a_05 * x_5;
    // Row 1
    wire signed [MAT_WIDTH+VEC_WIDTH-1:0] sum1 =
        a_10 * x_0 + a_11 * x_1 + a_12 * x_2 + a_13 * x_3 + a_14 * x_4 + a_15 * x_5;
    // Row 2
    wire signed [MAT_WIDTH+VEC_WIDTH-1:0] sum2 =
        a_20 * x_0 + a_21 * x_1 + a_22 * x_2 + a_23 * x_3 + a_24 * x_4 + a_25 * x_5;
    // Row 3
    wire signed [MAT_WIDTH+VEC_WIDTH-1:0] sum3 =
        a_30 * x_0 + a_31 * x_1 + a_32 * x_2 + a_33 * x_3 + a_34 * x_4 + a_35 * x_5;
    // Row 4
    wire signed [MAT_WIDTH+VEC_WIDTH-1:0] sum4 =
        a_40 * x_0 + a_41 * x_1 + a_42 * x_2 + a_43 * x_3 + a_44 * x_4 + a_45 * x_5;
    // Row 5
    wire signed [MAT_WIDTH+VEC_WIDTH-1:0] sum5 =
        a_50 * x_0 + a_51 * x_1 + a_52 * x_2 + a_53 * x_3 + a_54 * x_4 + a_55 * x_5;

    // Fixed-point truncation
    wire signed [VEC_WIDTH-1:0] dot0 = sum0[MAT_FRAC +: VEC_WIDTH];
    wire signed [VEC_WIDTH-1:0] dot1 = sum1[MAT_FRAC +: VEC_WIDTH];
    wire signed [VEC_WIDTH-1:0] dot2 = sum2[MAT_FRAC +: VEC_WIDTH];
    wire signed [VEC_WIDTH-1:0] dot3 = sum3[MAT_FRAC +: VEC_WIDTH];
    wire signed [VEC_WIDTH-1:0] dot4 = sum4[MAT_FRAC +: VEC_WIDTH];
    wire signed [VEC_WIDTH-1:0] dot5 = sum5[MAT_FRAC +: VEC_WIDTH];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            y_0 <= 0; y_1 <= 0; y_2 <= 0;
            y_3 <= 0; y_4 <= 0; y_5 <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 0;
                    if (start) begin
                        state <= COMPUTE;
                    end
                end

                // All 6 dot products registered in a single cycle
                COMPUTE: begin
                    y_0 <= dot0;
                    y_1 <= dot1;
                    y_2 <= dot2;
                    y_3 <= dot3;
                    y_4 <= dot4;
                    y_5 <= dot5;
                    done <= 1;
                    state <= DONE_STATE;
                end

                DONE_STATE: begin
                    if (!start) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
