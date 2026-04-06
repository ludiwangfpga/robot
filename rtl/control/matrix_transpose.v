// matrix_transpose.v
// 6x6 矩阵转置（纯组合逻辑）
// 输入：6x6矩阵（按行存储）
// 输出：转置后的6x6矩阵

`timescale 1ns / 1ps

module matrix_transpose #(
    parameter WIDTH = 24  // 矩阵元素位宽
)(
    // 输入矩阵（按行存储）
    input wire signed [WIDTH-1:0] in_00, in_01, in_02, in_03, in_04, in_05,
    input wire signed [WIDTH-1:0] in_10, in_11, in_12, in_13, in_14, in_15,
    input wire signed [WIDTH-1:0] in_20, in_21, in_22, in_23, in_24, in_25,
    input wire signed [WIDTH-1:0] in_30, in_31, in_32, in_33, in_34, in_35,
    input wire signed [WIDTH-1:0] in_40, in_41, in_42, in_43, in_44, in_45,
    input wire signed [WIDTH-1:0] in_50, in_51, in_52, in_53, in_54, in_55,

    // 输出矩阵（转置后，按行存储）
    output wire signed [WIDTH-1:0] out_00, out_01, out_02, out_03, out_04, out_05,
    output wire signed [WIDTH-1:0] out_10, out_11, out_12, out_13, out_14, out_15,
    output wire signed [WIDTH-1:0] out_20, out_21, out_22, out_23, out_24, out_25,
    output wire signed [WIDTH-1:0] out_30, out_31, out_32, out_33, out_34, out_35,
    output wire signed [WIDTH-1:0] out_40, out_41, out_42, out_43, out_44, out_45,
    output wire signed [WIDTH-1:0] out_50, out_51, out_52, out_53, out_54, out_55
);

// 转置操作：out[i][j] = in[j][i]
assign out_00 = in_00;
assign out_01 = in_10;
assign out_02 = in_20;
assign out_03 = in_30;
assign out_04 = in_40;
assign out_05 = in_50;

assign out_10 = in_01;
assign out_11 = in_11;
assign out_12 = in_21;
assign out_13 = in_31;
assign out_14 = in_41;
assign out_15 = in_51;

assign out_20 = in_02;
assign out_21 = in_12;
assign out_22 = in_22;
assign out_23 = in_32;
assign out_24 = in_42;
assign out_25 = in_52;

assign out_30 = in_03;
assign out_31 = in_13;
assign out_32 = in_23;
assign out_33 = in_33;
assign out_34 = in_43;
assign out_35 = in_53;

assign out_40 = in_04;
assign out_41 = in_14;
assign out_42 = in_24;
assign out_43 = in_34;
assign out_44 = in_44;
assign out_45 = in_54;

assign out_50 = in_05;
assign out_51 = in_15;
assign out_52 = in_25;
assign out_53 = in_35;
assign out_54 = in_45;
assign out_55 = in_55;

endmodule
