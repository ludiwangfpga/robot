`timescale 1ns / 1ps

// impedance_control_top.v - System Top-Level Impedance Controller
//
// Integrates the complete task-space impedance control pipeline:
//   Encoders → FK+Jacobian → Velocity Est → Impedance Ctrl → J^T·F → Torque→Iq → FOC
//
// Two clock domains:
//   sys_clk (~100MHz): Control pipeline (FK, Jacobian, impedance, torque mapping)
//   foc_clk (~73.728MHz): FOC motor controllers
//
// The control pipeline runs continuously:
//   1. Read encoder angles (Q16.16 radians)
//   2. FK + Jacobian computation (~63 cycles)
//      - 6× parallel CORDIC sin/cos pre-computation (20 cyc)
//      - DH alpha sin/cos as compile-time constants (0 cyc)
//      - 4-row parallel matrix chain multiply (18 cyc)
//      - 3× parallel atan2 Euler extraction (22 cyc)
//      - 6-column parallel Jacobian (2 cyc)
//   3. Velocity estimation (~4 cycles)
//   4. Impedance control F = Ks*(xd-x) + Ds*(xd_dot-x_dot) (~3 cycles)
//   5. Joint torque mapping τ = J^T * F (~2 cycles, 6-row parallel)
//   6. Torque to Iq conversion (~3 cycles)
//   7. FOC channels run independently with updated Iq references
//
// Total pipeline latency: ~75 cycles @ 100MHz ≈ 0.75μs → ~1.33MHz control bandwidth

module impedance_control_top #(
    // Velocity estimator parameters
    parameter signed [31:0] VEL_INV_DT = 32'sd655360000,  // 1/dt for 10kHz sampling
    parameter signed [31:0] VEL_ALPHA  = 32'sd19661,       // IIR filter alpha=0.3
    parameter signed [31:0] VEL_ONE_MINUS_ALPHA = 32'sd45875  // 1-alpha=0.7
) (
    // System clock and reset
    input  wire        sys_clk,     // System clock (~100MHz)
    input  wire        rst_n,       // Active-low reset

    // FOC clock (directly passed to FOC array)
    input  wire        foc_clk,     // FOC clock (~73.728MHz)

    // Control enable
    input  wire        ctrl_enable, // Enable impedance control loop

    // Desired pose (Q16.16, from trajectory planner or host)
    input  wire signed [31:0] xd_x,  xd_y,  xd_z,
    input  wire signed [31:0] xd_rx, xd_ry, xd_rz,

    // Desired velocity (Q16.16, typically zero for regulation)
    input  wire signed [31:0] xd_dot_x,  xd_dot_y,  xd_dot_z,
    input  wire signed [31:0] xd_dot_rx, xd_dot_ry, xd_dot_rz,

    // Stiffness gains (Q16.16, N/m for linear, Nm/rad for angular)
    input  wire signed [31:0] Ks_x,  Ks_y,  Ks_z,
    input  wire signed [31:0] Ks_rx, Ks_ry, Ks_rz,

    // Damping gains (Q16.16, Ns/m for linear, Nms/rad for angular)
    input  wire signed [31:0] Ds_x,  Ds_y,  Ds_z,
    input  wire signed [31:0] Ds_rx, Ds_ry, Ds_rz,

    // Joint angles from encoders (Q16.16 radians, from encoder_interface)
    input  wire signed [31:0] q0, q1, q2, q3, q4, q5,

    // FOC PI parameters
    input  wire [30:0] foc_Kp,
    input  wire [30:0] foc_Ki,

    // Encoder phi (12-bit, for FOC, from encoder_interface)
    input  wire [11:0] phi_0, phi_1, phi_2,
    input  wire [11:0] phi_3, phi_4, phi_5,

    // ADC interfaces (directly to/from external ADCs)
    output wire        sn_adc_0, sn_adc_1, sn_adc_2,
    output wire        sn_adc_3, sn_adc_4, sn_adc_5,
    input  wire        en_adc_0, en_adc_1, en_adc_2,
    input  wire        en_adc_3, en_adc_4, en_adc_5,
    input  wire [11:0] adc_a_0, adc_b_0, adc_c_0,
    input  wire [11:0] adc_a_1, adc_b_1, adc_c_1,
    input  wire [11:0] adc_a_2, adc_b_2, adc_c_2,
    input  wire [11:0] adc_a_3, adc_b_3, adc_c_3,
    input  wire [11:0] adc_a_4, adc_b_4, adc_c_4,
    input  wire [11:0] adc_a_5, adc_b_5, adc_c_5,

    // PWM outputs
    output wire        pwm_en_0, pwm_a_0, pwm_b_0, pwm_c_0,
    output wire        pwm_en_1, pwm_a_1, pwm_b_1, pwm_c_1,
    output wire        pwm_en_2, pwm_a_2, pwm_b_2, pwm_c_2,
    output wire        pwm_en_3, pwm_a_3, pwm_b_3, pwm_c_3,
    output wire        pwm_en_4, pwm_a_4, pwm_b_4, pwm_c_4,
    output wire        pwm_en_5, pwm_a_5, pwm_b_5, pwm_c_5,

    // Debug/monitoring outputs
    output wire signed [31:0] dbg_pose_x, dbg_pose_y, dbg_pose_z,
    output wire signed [31:0] dbg_pose_rx, dbg_pose_ry, dbg_pose_rz,
    output wire signed [31:0] dbg_F_x, dbg_F_y, dbg_F_z,
    output wire signed [31:0] dbg_tau0, dbg_tau1, dbg_tau2,
    output wire signed [31:0] dbg_tau3, dbg_tau4, dbg_tau5,
    output reg  [31:0] dbg_cycle_count,   // Pipeline cycle counter
    output wire        foc_all_init_done,  // All FOC channels initialized
    output reg         pipeline_active     // Pipeline is running
);

    // =====================================================================
    // State Machine
    // =====================================================================
    localparam S_IDLE       = 4'd0;
    localparam S_FK_START   = 4'd1;
    localparam S_FK_WAIT    = 4'd2;
    localparam S_VEL_START  = 4'd3;
    localparam S_VEL_WAIT   = 4'd4;
    localparam S_IMP_START  = 4'd5;
    localparam S_IMP_WAIT   = 4'd6;
    localparam S_JT_START   = 4'd7;
    localparam S_JT_WAIT    = 4'd8;
    localparam S_T2IQ_START = 4'd9;
    localparam S_T2IQ_WAIT  = 4'd10;
    localparam S_DONE       = 4'd11;

    reg [3:0] state;
    reg [31:0] cycle_timer;

    // =====================================================================
    // Inter-module wires
    // =====================================================================

    // FK + Jacobian outputs
    reg  fk_start;
    wire fk_done;
    wire signed [31:0] pose_x, pose_y, pose_z;
    wire signed [31:0] pose_rx, pose_ry, pose_rz;
    wire signed [31:0] J00, J01, J02, J03, J04, J05;
    wire signed [31:0] J10, J11, J12, J13, J14, J15;
    wire signed [31:0] J20, J21, J22, J23, J24, J25;
    wire signed [31:0] J30, J31, J32, J33, J34, J35;
    wire signed [31:0] J40, J41, J42, J43, J44, J45;
    wire signed [31:0] J50, J51, J52, J53, J54, J55;

    // Velocity estimator outputs
    reg  vel_enable;
    wire vel_done;
    wire vel_valid;
    wire signed [31:0] vx, vy, vz, wx, wy, wz;

    // Impedance controller outputs
    reg  imp_start;
    wire imp_done;
    wire signed [31:0] F_x, F_y, F_z, F_rx, F_ry, F_rz;

    // J^T force mapper outputs
    reg  jt_start;
    wire jt_done;
    wire signed [31:0] tau0, tau1, tau2, tau3, tau4, tau5;

    // Torque to Iq outputs
    reg  t2iq_start;
    wire t2iq_done;
    wire signed [15:0] iq0, iq1, iq2, iq3, iq4, iq5;

    // =====================================================================
    // Module Instantiations
    // =====================================================================

    // --- FK + Analytical Jacobian ---
    analytical_jacobian fk_jac_inst (
        .clk(sys_clk), .rst_n(rst_n), .start(fk_start),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        .pose_x(pose_x), .pose_y(pose_y), .pose_z(pose_z),
        .pose_rx(pose_rx), .pose_ry(pose_ry), .pose_rz(pose_rz),
        .J00(J00), .J01(J01), .J02(J02), .J03(J03), .J04(J04), .J05(J05),
        .J10(J10), .J11(J11), .J12(J12), .J13(J13), .J14(J14), .J15(J15),
        .J20(J20), .J21(J21), .J22(J22), .J23(J23), .J24(J24), .J25(J25),
        .J30(J30), .J31(J31), .J32(J32), .J33(J33), .J34(J34), .J35(J35),
        .J40(J40), .J41(J41), .J42(J42), .J43(J43), .J44(J44), .J45(J45),
        .J50(J50), .J51(J51), .J52(J52), .J53(J53), .J54(J54), .J55(J55),
        .done(fk_done)
    );

    // --- Velocity Estimator ---
    velocity_estimator #(
        .INV_DT(VEL_INV_DT),
        .ALPHA(VEL_ALPHA),
        .ONE_MINUS_ALPHA(VEL_ONE_MINUS_ALPHA)
    ) vel_est_inst (
        .clk(sys_clk), .rst_n(rst_n), .enable(vel_enable),
        .x(pose_x), .y(pose_y), .z(pose_z),
        .rx(pose_rx), .ry(pose_ry), .rz(pose_rz),
        .vx(vx), .vy(vy), .vz(vz),
        .wx(wx), .wy(wy), .wz(wz),
        .done(vel_done), .valid(vel_valid)
    );

    // --- Impedance Controller ---
    impedance_controller imp_ctrl_inst (
        .clk(sys_clk), .rst_n(rst_n), .start(imp_start),
        .xd_x(xd_x), .xd_y(xd_y), .xd_z(xd_z),
        .xd_rx(xd_rx), .xd_ry(xd_ry), .xd_rz(xd_rz),
        .x_x(pose_x), .x_y(pose_y), .x_z(pose_z),
        .x_rx(pose_rx), .x_ry(pose_ry), .x_rz(pose_rz),
        .xd_dot_x(xd_dot_x), .xd_dot_y(xd_dot_y), .xd_dot_z(xd_dot_z),
        .xd_dot_rx(xd_dot_rx), .xd_dot_ry(xd_dot_ry), .xd_dot_rz(xd_dot_rz),
        .x_dot_x(vx), .x_dot_y(vy), .x_dot_z(vz),
        .x_dot_rx(wx), .x_dot_ry(wy), .x_dot_rz(wz),
        .Ks_x(Ks_x), .Ks_y(Ks_y), .Ks_z(Ks_z),
        .Ks_rx(Ks_rx), .Ks_ry(Ks_ry), .Ks_rz(Ks_rz),
        .Ds_x(Ds_x), .Ds_y(Ds_y), .Ds_z(Ds_z),
        .Ds_rx(Ds_rx), .Ds_ry(Ds_ry), .Ds_rz(Ds_rz),
        .F_x(F_x), .F_y(F_y), .F_z(F_z),
        .F_rx(F_rx), .F_ry(F_ry), .F_rz(F_rz),
        .done(imp_done)
    );

    // --- J^T Force-to-Torque Mapper ---
    jt_force_mapper jt_map_inst (
        .clk(sys_clk), .rst_n(rst_n), .start(jt_start),
        .J00(J00), .J01(J01), .J02(J02), .J03(J03), .J04(J04), .J05(J05),
        .J10(J10), .J11(J11), .J12(J12), .J13(J13), .J14(J14), .J15(J15),
        .J20(J20), .J21(J21), .J22(J22), .J23(J23), .J24(J24), .J25(J25),
        .J30(J30), .J31(J31), .J32(J32), .J33(J33), .J34(J34), .J35(J35),
        .J40(J40), .J41(J41), .J42(J42), .J43(J43), .J44(J44), .J45(J45),
        .J50(J50), .J51(J51), .J52(J52), .J53(J53), .J54(J54), .J55(J55),
        .F0(F_x), .F1(F_y), .F2(F_z), .F3(F_rx), .F4(F_ry), .F5(F_rz),
        .tau0(tau0), .tau1(tau1), .tau2(tau2),
        .tau3(tau3), .tau4(tau4), .tau5(tau5),
        .done(jt_done)
    );

    // --- Torque to Iq Converter ---
    torque_to_iq t2iq_inst (
        .clk(sys_clk), .rst_n(rst_n), .start(t2iq_start),
        .tau0(tau0), .tau1(tau1), .tau2(tau2),
        .tau3(tau3), .tau4(tau4), .tau5(tau5),
        .iq0(iq0), .iq1(iq1), .iq2(iq2),
        .iq3(iq3), .iq4(iq4), .iq5(iq5),
        .done(t2iq_done)
    );

    // --- FOC Array (6 channels) ---
    foc_array foc_inst (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(foc_Kp), .Ki(foc_Ki),
        .iq_ref_0(iq0), .iq_ref_1(iq1), .iq_ref_2(iq2),
        .iq_ref_3(iq3), .iq_ref_4(iq4), .iq_ref_5(iq5),
        .phi_0(phi_0), .phi_1(phi_1), .phi_2(phi_2),
        .phi_3(phi_3), .phi_4(phi_4), .phi_5(phi_5),
        .sn_adc_0(sn_adc_0), .en_adc_0(en_adc_0),
        .adc_a_0(adc_a_0), .adc_b_0(adc_b_0), .adc_c_0(adc_c_0),
        .sn_adc_1(sn_adc_1), .en_adc_1(en_adc_1),
        .adc_a_1(adc_a_1), .adc_b_1(adc_b_1), .adc_c_1(adc_c_1),
        .sn_adc_2(sn_adc_2), .en_adc_2(en_adc_2),
        .adc_a_2(adc_a_2), .adc_b_2(adc_b_2), .adc_c_2(adc_c_2),
        .sn_adc_3(sn_adc_3), .en_adc_3(en_adc_3),
        .adc_a_3(adc_a_3), .adc_b_3(adc_b_3), .adc_c_3(adc_c_3),
        .sn_adc_4(sn_adc_4), .en_adc_4(en_adc_4),
        .adc_a_4(adc_a_4), .adc_b_4(adc_b_4), .adc_c_4(adc_c_4),
        .sn_adc_5(sn_adc_5), .en_adc_5(en_adc_5),
        .adc_a_5(adc_a_5), .adc_b_5(adc_b_5), .adc_c_5(adc_c_5),
        .pwm_en_0(pwm_en_0), .pwm_a_0(pwm_a_0), .pwm_b_0(pwm_b_0), .pwm_c_0(pwm_c_0),
        .pwm_en_1(pwm_en_1), .pwm_a_1(pwm_a_1), .pwm_b_1(pwm_b_1), .pwm_c_1(pwm_c_1),
        .pwm_en_2(pwm_en_2), .pwm_a_2(pwm_a_2), .pwm_b_2(pwm_b_2), .pwm_c_2(pwm_c_2),
        .pwm_en_3(pwm_en_3), .pwm_a_3(pwm_a_3), .pwm_b_3(pwm_b_3), .pwm_c_3(pwm_c_3),
        .pwm_en_4(pwm_en_4), .pwm_a_4(pwm_a_4), .pwm_b_4(pwm_b_4), .pwm_c_4(pwm_c_4),
        .pwm_en_5(pwm_en_5), .pwm_a_5(pwm_a_5), .pwm_b_5(pwm_b_5), .pwm_c_5(pwm_c_5),
        .iq_actual_0(), .iq_actual_1(), .iq_actual_2(),
        .iq_actual_3(), .iq_actual_4(), .iq_actual_5(),
        .all_init_done(foc_all_init_done)
    );

    // =====================================================================
    // Debug outputs
    // =====================================================================
    assign dbg_pose_x  = pose_x;
    assign dbg_pose_y  = pose_y;
    assign dbg_pose_z  = pose_z;
    assign dbg_pose_rx = pose_rx;
    assign dbg_pose_ry = pose_ry;
    assign dbg_pose_rz = pose_rz;
    assign dbg_F_x  = F_x;
    assign dbg_F_y  = F_y;
    assign dbg_F_z  = F_z;
    assign dbg_tau0 = tau0;
    assign dbg_tau1 = tau1;
    assign dbg_tau2 = tau2;
    assign dbg_tau3 = tau3;
    assign dbg_tau4 = tau4;
    assign dbg_tau5 = tau5;

    // =====================================================================
    // Main Control Pipeline State Machine
    // =====================================================================
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            fk_start <= 1'b0;
            vel_enable <= 1'b0;
            imp_start <= 1'b0;
            jt_start <= 1'b0;
            t2iq_start <= 1'b0;
            cycle_timer <= 32'd0;
            dbg_cycle_count <= 32'd0;
            pipeline_active <= 1'b0;
        end else begin
            // Default: deassert start signals after one cycle
            fk_start <= 1'b0;
            vel_enable <= 1'b0;
            imp_start <= 1'b0;
            jt_start <= 1'b0;
            t2iq_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    pipeline_active <= 1'b0;
                    if (ctrl_enable) begin
                        cycle_timer <= 32'd0;
                        pipeline_active <= 1'b1;
                        state <= S_FK_START;
                    end
                end

                // --- Stage 1: FK + Jacobian ---
                S_FK_START: begin
                    fk_start <= 1'b1;
                    cycle_timer <= cycle_timer + 1;
                    state <= S_FK_WAIT;
                end

                S_FK_WAIT: begin
                    cycle_timer <= cycle_timer + 1;
                    if (fk_done) begin
                        state <= S_VEL_START;
                    end
                end

                // --- Stage 2: Velocity Estimation ---
                S_VEL_START: begin
                    vel_enable <= 1'b1;
                    cycle_timer <= cycle_timer + 1;
                    state <= S_VEL_WAIT;
                end

                S_VEL_WAIT: begin
                    cycle_timer <= cycle_timer + 1;
                    if (vel_done) begin
                        state <= S_IMP_START;
                    end
                end

                // --- Stage 3: Impedance Control ---
                S_IMP_START: begin
                    imp_start <= 1'b1;
                    cycle_timer <= cycle_timer + 1;
                    state <= S_IMP_WAIT;
                end

                S_IMP_WAIT: begin
                    cycle_timer <= cycle_timer + 1;
                    if (imp_done) begin
                        state <= S_JT_START;
                    end
                end

                // --- Stage 4: J^T Force Mapping ---
                S_JT_START: begin
                    jt_start <= 1'b1;
                    cycle_timer <= cycle_timer + 1;
                    state <= S_JT_WAIT;
                end

                S_JT_WAIT: begin
                    cycle_timer <= cycle_timer + 1;
                    if (jt_done) begin
                        state <= S_T2IQ_START;
                    end
                end

                // --- Stage 5: Torque to Iq ---
                S_T2IQ_START: begin
                    t2iq_start <= 1'b1;
                    cycle_timer <= cycle_timer + 1;
                    state <= S_T2IQ_WAIT;
                end

                S_T2IQ_WAIT: begin
                    cycle_timer <= cycle_timer + 1;
                    if (t2iq_done) begin
                        state <= S_DONE;
                    end
                end

                // --- Pipeline Complete ---
                S_DONE: begin
                    dbg_cycle_count <= cycle_timer;
                    // Immediately restart if enabled (continuous loop)
                    if (ctrl_enable) begin
                        cycle_timer <= 32'd0;
                        state <= S_FK_START;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
