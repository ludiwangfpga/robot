`timescale 1ns / 1ps

// impedance_controller.v - Task-Space Impedance Controller
//
// Implements the impedance control law with diagonal stiffness and damping:
//   F[i] = Ks[i] * (xd[i] - x[i]) + Ds[i] * (xd_dot[i] - x_dot[i])
//
// All values in Q16.16 format.
// Latency: 3 clock cycles (DIFF → MULT → SUM → DONE)

module impedance_controller (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // Desired pose (Q16.16)
    input  wire signed [31:0] xd_x,  xd_y,  xd_z,
    input  wire signed [31:0] xd_rx, xd_ry, xd_rz,

    // Actual pose from FK (Q16.16)
    input  wire signed [31:0] x_x,  x_y,  x_z,
    input  wire signed [31:0] x_rx, x_ry, x_rz,

    // Desired velocity (Q16.16, typically zero for regulation)
    input  wire signed [31:0] xd_dot_x,  xd_dot_y,  xd_dot_z,
    input  wire signed [31:0] xd_dot_rx, xd_dot_ry, xd_dot_rz,

    // Actual velocity (Q16.16)
    input  wire signed [31:0] x_dot_x,  x_dot_y,  x_dot_z,
    input  wire signed [31:0] x_dot_rx, x_dot_ry, x_dot_rz,

    // Stiffness (diagonal, Q16.16, N/m for linear, Nm/rad for angular)
    input  wire signed [31:0] Ks_x,  Ks_y,  Ks_z,
    input  wire signed [31:0] Ks_rx, Ks_ry, Ks_rz,

    // Damping (diagonal, Q16.16, Ns/m for linear, Nms/rad for angular)
    input  wire signed [31:0] Ds_x,  Ds_y,  Ds_z,
    input  wire signed [31:0] Ds_rx, Ds_ry, Ds_rz,

    // Output: task-space wrench (Q16.16, N for force, Nm for torque)
    output reg  signed [31:0] F_x,  F_y,  F_z,
    output reg  signed [31:0] F_rx, F_ry, F_rz,

    output reg         done
);

    // State machine
    localparam IDLE  = 2'd0;
    localparam CALC  = 2'd1;
    localparam DONE_ST = 2'd2;

    reg [1:0] state;

    // Position and velocity errors
    reg signed [31:0] e_x, e_y, e_z, e_rx, e_ry, e_rz;
    reg signed [31:0] ed_x, ed_y, ed_z, ed_rx, ed_ry, ed_rz;

    // Q16.16 multiply
    (* use_dsp = "yes" *)
    function signed [31:0] fmult;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [63:0] temp;
        begin
            temp = $signed(a) * $signed(b);
            fmult = temp[47:16];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            F_x <= 0; F_y <= 0; F_z <= 0;
            F_rx <= 0; F_ry <= 0; F_rz <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Compute position error: e = xd - x
                        e_x  <= xd_x  - x_x;
                        e_y  <= xd_y  - x_y;
                        e_z  <= xd_z  - x_z;
                        e_rx <= xd_rx - x_rx;
                        e_ry <= xd_ry - x_ry;
                        e_rz <= xd_rz - x_rz;

                        // Compute velocity error: ed = xd_dot - x_dot
                        ed_x  <= xd_dot_x  - x_dot_x;
                        ed_y  <= xd_dot_y  - x_dot_y;
                        ed_z  <= xd_dot_z  - x_dot_z;
                        ed_rx <= xd_dot_rx - x_dot_rx;
                        ed_ry <= xd_dot_ry - x_dot_ry;
                        ed_rz <= xd_dot_rz - x_dot_rz;

                        state <= CALC;
                    end
                end

                CALC: begin
                    // F = Ks * e + Ds * ed (all 6 channels in parallel)
                    F_x  <= fmult(Ks_x,  e_x)  + fmult(Ds_x,  ed_x);
                    F_y  <= fmult(Ks_y,  e_y)  + fmult(Ds_y,  ed_y);
                    F_z  <= fmult(Ks_z,  e_z)  + fmult(Ds_z,  ed_z);
                    F_rx <= fmult(Ks_rx, e_rx) + fmult(Ds_rx, ed_rx);
                    F_ry <= fmult(Ks_ry, e_ry) + fmult(Ds_ry, ed_ry);
                    F_rz <= fmult(Ks_rz, e_rz) + fmult(Ds_rz, ed_rz);

                    state <= DONE_ST;
                end

                DONE_ST: begin
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
