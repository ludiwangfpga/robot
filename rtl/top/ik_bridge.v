`timescale 1ns / 1ps

// ik_bridge.v - IK Format Conversion Bridge
//
// Bridges the LM-IK solver (S3.12 joint angles) with the impedance control
// pipeline (Q16.16 format). The IK solver runs asynchronously from the main
// control loop, only when the desired pose changes.
//
// Flow: Target pose (Q16.16) → [format convert] → IK solver → joint angles (S3.12)
//       → [format convert] → Q16.16 joint angles for trajectory reference
//
// Note: In pure task-space impedance control, IK is NOT in the critical path.
//       The impedance controller directly uses xd (desired pose) in task space.
//       IK is only needed for joint-space trajectory planning or feasibility checks.
//
// All conversions are combinational (0 delay).

module ik_bridge (
    // Target pose in Q16.16 (from trajectory planner)
    input  wire signed [31:0] xd_q16_x,  xd_q16_y,  xd_q16_z,
    input  wire signed [31:0] xd_q16_rx, xd_q16_ry, xd_q16_rz,

    // Target pose converted to S8.15 (for IK solver input)
    output wire signed [23:0] xd_s815_x,  xd_s815_y,  xd_s815_z,
    output wire signed [23:0] xd_s815_rx, xd_s815_ry, xd_s815_rz,

    // IK output joint angles in S3.12 (from IK solver)
    input  wire signed [15:0] q_s312_0, q_s312_1, q_s312_2,
    input  wire signed [15:0] q_s312_3, q_s312_4, q_s312_5,

    // IK output converted to Q16.16 (for FK/trajectory use)
    output wire signed [31:0] q_q16_0, q_q16_1, q_q16_2,
    output wire signed [31:0] q_q16_3, q_q16_4, q_q16_5,

    // Overflow flags
    output wire [5:0] pose_overflow,    // per-axis overflow flags for pose conversion
    output wire [5:0] q_overflow        // not used for S3.12→Q16.16 (always fits)
);

    // Q16.16 → S8.15 for pose (IK input)
    wire ovf_x, ovf_y, ovf_z, ovf_rx, ovf_ry, ovf_rz;

    q16_16_to_s8_15 conv_xd_x  (.in_q16_16(xd_q16_x),  .out_s8_15(xd_s815_x),  .overflow(ovf_x));
    q16_16_to_s8_15 conv_xd_y  (.in_q16_16(xd_q16_y),  .out_s8_15(xd_s815_y),  .overflow(ovf_y));
    q16_16_to_s8_15 conv_xd_z  (.in_q16_16(xd_q16_z),  .out_s8_15(xd_s815_z),  .overflow(ovf_z));
    q16_16_to_s8_15 conv_xd_rx (.in_q16_16(xd_q16_rx), .out_s8_15(xd_s815_rx), .overflow(ovf_rx));
    q16_16_to_s8_15 conv_xd_ry (.in_q16_16(xd_q16_ry), .out_s8_15(xd_s815_ry), .overflow(ovf_ry));
    q16_16_to_s8_15 conv_xd_rz (.in_q16_16(xd_q16_rz), .out_s8_15(xd_s815_rz), .overflow(ovf_rz));

    assign pose_overflow = {ovf_rz, ovf_ry, ovf_rx, ovf_z, ovf_y, ovf_x};

    // S3.12 → Q16.16 for joint angles (IK output)
    s3_12_to_q16_16 conv_q0 (.in_s3_12(q_s312_0), .out_q16_16(q_q16_0));
    s3_12_to_q16_16 conv_q1 (.in_s3_12(q_s312_1), .out_q16_16(q_q16_1));
    s3_12_to_q16_16 conv_q2 (.in_s3_12(q_s312_2), .out_q16_16(q_q16_2));
    s3_12_to_q16_16 conv_q3 (.in_s3_12(q_s312_3), .out_q16_16(q_q16_3));
    s3_12_to_q16_16 conv_q4 (.in_s3_12(q_s312_4), .out_q16_16(q_q16_4));
    s3_12_to_q16_16 conv_q5 (.in_s3_12(q_s312_5), .out_q16_16(q_q16_5));

    assign q_overflow = 6'b000000;  // S3.12→Q16.16 never overflows

endmodule
