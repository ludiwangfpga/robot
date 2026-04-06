`timescale 1ns / 1ps

// jt_force_mapper.v - Jacobian Transpose Force-to-Torque Mapper
// Computes joint torques: τ = J^T · F
// where J is the 6x6 Jacobian and F is the task-space wrench
//
// Uses matrix_transpose (combinational) and matrix_vector_mult (6 cycles)
// Re-parameterized for Q16.16 format (32-bit)
//
// Q16.16 format throughout. Latency: ~8 clock cycles.

module jt_force_mapper (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // Jacobian matrix J (6x6, Q16.16)
    input  wire signed [31:0] J00, J01, J02, J03, J04, J05,
    input  wire signed [31:0] J10, J11, J12, J13, J14, J15,
    input  wire signed [31:0] J20, J21, J22, J23, J24, J25,
    input  wire signed [31:0] J30, J31, J32, J33, J34, J35,
    input  wire signed [31:0] J40, J41, J42, J43, J44, J45,
    input  wire signed [31:0] J50, J51, J52, J53, J54, J55,

    // Task-space wrench F (6x1, Q16.16)
    input  wire signed [31:0] F0, F1, F2, F3, F4, F5,

    // Joint torques τ (6x1, Q16.16)
    output wire signed [31:0] tau0, tau1, tau2, tau3, tau4, tau5,

    output wire        done
);

    // J^T (combinational, 0 delay)
    wire signed [31:0] Jt00, Jt01, Jt02, Jt03, Jt04, Jt05;
    wire signed [31:0] Jt10, Jt11, Jt12, Jt13, Jt14, Jt15;
    wire signed [31:0] Jt20, Jt21, Jt22, Jt23, Jt24, Jt25;
    wire signed [31:0] Jt30, Jt31, Jt32, Jt33, Jt34, Jt35;
    wire signed [31:0] Jt40, Jt41, Jt42, Jt43, Jt44, Jt45;
    wire signed [31:0] Jt50, Jt51, Jt52, Jt53, Jt54, Jt55;

    matrix_transpose #(
        .WIDTH(32)
    ) transpose_inst (
        .in_00(J00), .in_01(J01), .in_02(J02), .in_03(J03), .in_04(J04), .in_05(J05),
        .in_10(J10), .in_11(J11), .in_12(J12), .in_13(J13), .in_14(J14), .in_15(J15),
        .in_20(J20), .in_21(J21), .in_22(J22), .in_23(J23), .in_24(J24), .in_25(J25),
        .in_30(J30), .in_31(J31), .in_32(J32), .in_33(J33), .in_34(J34), .in_35(J35),
        .in_40(J40), .in_41(J41), .in_42(J42), .in_43(J43), .in_44(J44), .in_45(J45),
        .in_50(J50), .in_51(J51), .in_52(J52), .in_53(J53), .in_54(J54), .in_55(J55),
        .out_00(Jt00), .out_01(Jt01), .out_02(Jt02), .out_03(Jt03), .out_04(Jt04), .out_05(Jt05),
        .out_10(Jt10), .out_11(Jt11), .out_12(Jt12), .out_13(Jt13), .out_14(Jt14), .out_15(Jt15),
        .out_20(Jt20), .out_21(Jt21), .out_22(Jt22), .out_23(Jt23), .out_24(Jt24), .out_25(Jt25),
        .out_30(Jt30), .out_31(Jt31), .out_32(Jt32), .out_33(Jt33), .out_34(Jt34), .out_35(Jt35),
        .out_40(Jt40), .out_41(Jt41), .out_42(Jt42), .out_43(Jt43), .out_44(Jt44), .out_45(Jt45),
        .out_50(Jt50), .out_51(Jt51), .out_52(Jt52), .out_53(Jt53), .out_54(Jt54), .out_55(Jt55)
    );

    // Invert rst_n to rst for matrix_vector_mult (which uses active-high reset)
    wire rst = ~rst_n;

    // τ = J^T · F (6 clock cycles)
    matrix_vector_mult #(
        .MAT_WIDTH(32),
        .VEC_WIDTH(32),
        .MAT_FRAC(16),
        .VEC_FRAC(16)
    ) matvec_inst (
        .clk(clk),
        .rst(rst),
        .start(start),
        .a_00(Jt00), .a_01(Jt01), .a_02(Jt02), .a_03(Jt03), .a_04(Jt04), .a_05(Jt05),
        .a_10(Jt10), .a_11(Jt11), .a_12(Jt12), .a_13(Jt13), .a_14(Jt14), .a_15(Jt15),
        .a_20(Jt20), .a_21(Jt21), .a_22(Jt22), .a_23(Jt23), .a_24(Jt24), .a_25(Jt25),
        .a_30(Jt30), .a_31(Jt31), .a_32(Jt32), .a_33(Jt33), .a_34(Jt34), .a_35(Jt35),
        .a_40(Jt40), .a_41(Jt41), .a_42(Jt42), .a_43(Jt43), .a_44(Jt44), .a_45(Jt45),
        .a_50(Jt50), .a_51(Jt51), .a_52(Jt52), .a_53(Jt53), .a_54(Jt54), .a_55(Jt55),
        .x_0(F0), .x_1(F1), .x_2(F2), .x_3(F3), .x_4(F4), .x_5(F5),
        .y_0(tau0), .y_1(tau1), .y_2(tau2), .y_3(tau3), .y_4(tau4), .y_5(tau5),
        .done(done)
    );

endmodule
