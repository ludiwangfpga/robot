`timescale 1ns / 1ps

// adaptive_impedance.v - Online Environment Stiffness Estimation + Adaptive K/D
//
// Method A: Current-based force estimation (sensorless)
//
// Data flow in real system:
//   1. FOC measures iq_actual (ADC) → τ_measured = Kt × iq_actual
//   2. Impedance controller outputs τ_cmd = J^T × F
//   3. External (contact) torque: τ_ext = τ_measured - τ_cmd
//   4. Joint-space RLS: τ_ext[i] = K_env_joint[i] × q_error[i]
//   5. Adaptive: Ks_out = f(K_env_joint), Ds_out follows
//
// This module operates in JOINT SPACE to avoid Jacobian inverse.
// 6 independent scalar RLS estimators, one per joint.
//
// RLS update (per joint):
//   gain    = P * q_err / (λ + q_err * P * q_err)
//   K_env   = K_env + gain * (τ_ext - K_env * q_err)
//   P       = (1/λ) * (P - gain * q_err * P)
//
// Adaptive rule (inverse proportional):
//   Ks_out[i]  = clamp(Ks_nom[i]² / K_env[i], KS_MIN, KS_MAX)
//   Ds_out[i]  = Ds_nom[i] * Ks_out[i] / Ks_nom[i]
//
// Dead-zone: freeze K_env update when |q_err| < Q_DEADZONE
//
// All values Q16.16 fixed-point. Latency: 6 joints × 8 cycles = 48 cycles.

module adaptive_impedance #(
    parameter signed [31:0] LAMBDA       = 32'sd64225,  // 0.98 in Q16.16
    parameter signed [31:0] INV_LAMBDA   = 32'sd66872,  // 1/0.98 ≈ 1.0204 in Q16.16
    parameter signed [31:0] P_INIT       = 32'sd65536,  // 1.0 in Q16.16
    parameter signed [31:0] K_ENV_INIT   = 32'sd65536,  // 1.0 initial guess
    parameter signed [31:0] Q_DEADZONE   = 32'sd65,     // 0.001 rad dead-zone
    parameter signed [31:0] KS_MIN       = 32'sd6554,   // 0.1 minimum stiffness
    parameter signed [31:0] KS_MAX       = 32'sd6553600  // 100.0 maximum stiffness
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // External (contact) torque per joint: τ_ext = τ_measured - τ_cmd (Q16.16 Nm)
    input  wire signed [31:0] tau_ext_0, tau_ext_1, tau_ext_2,
    input  wire signed [31:0] tau_ext_3, tau_ext_4, tau_ext_5,

    // Joint position error: q_d - q (Q16.16 rad)
    input  wire signed [31:0] q_err_0, q_err_1, q_err_2,
    input  wire signed [31:0] q_err_3, q_err_4, q_err_5,

    // Nominal stiffness per joint (Q16.16, from host)
    input  wire signed [31:0] Ks_nom_0, Ks_nom_1, Ks_nom_2,
    input  wire signed [31:0] Ks_nom_3, Ks_nom_4, Ks_nom_5,

    // Nominal damping per joint (Q16.16, from host)
    input  wire signed [31:0] Ds_nom_0, Ds_nom_1, Ds_nom_2,
    input  wire signed [31:0] Ds_nom_3, Ds_nom_4, Ds_nom_5,

    // Enable adaptive mode (when 0, output = nominal)
    input  wire        adaptive_enable,

    // Adapted output: stiffness per joint (Q16.16)
    output reg  signed [31:0] Ks_out_0, Ks_out_1, Ks_out_2,
    output reg  signed [31:0] Ks_out_3, Ks_out_4, Ks_out_5,

    // Adapted output: damping per joint (Q16.16)
    output reg  signed [31:0] Ds_out_0, Ds_out_1, Ds_out_2,
    output reg  signed [31:0] Ds_out_3, Ds_out_4, Ds_out_5,

    // Debug: estimated environment stiffness per joint (Q16.16)
    output wire signed [31:0] K_env_est_0, K_env_est_1, K_env_est_2,
    output wire signed [31:0] K_env_est_3, K_env_est_4, K_env_est_5,

    output reg         done
);

    // ── Q16.16 Multiply ─────────────────────────────────────────────────────
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

    // ── Q16.16 Divide ───────────────────────────────────────────────────────
    function signed [31:0] fdiv;
        input signed [31:0] numer;
        input signed [31:0] denom;
        reg signed [63:0] n64;
        begin
            if (denom == 0)
                fdiv = 32'sh7FFF_FFFF;
            else begin
                n64 = $signed(numer) <<< 16;
                fdiv = n64 / $signed(denom);
            end
        end
    endfunction

    // ── Absolute value ──────────────────────────────────────────────────────
    function signed [31:0] fabs;
        input signed [31:0] v;
        begin
            fabs = (v < 0) ? -v : v;
        end
    endfunction

    // ── Clamp ───────────────────────────────────────────────────────────────
    function signed [31:0] fclamp;
        input signed [31:0] v;
        input signed [31:0] lo;
        input signed [31:0] hi;
        begin
            if (v < lo) fclamp = lo;
            else if (v > hi) fclamp = hi;
            else fclamp = v;
        end
    endfunction

    // ── RLS state per joint ─────────────────────────────────────────────────
    reg signed [31:0] K_env [0:5];
    reg signed [31:0] P     [0:5];

    assign K_env_est_0 = K_env[0];
    assign K_env_est_1 = K_env[1];
    assign K_env_est_2 = K_env[2];
    assign K_env_est_3 = K_env[3];
    assign K_env_est_4 = K_env[4];
    assign K_env_est_5 = K_env[5];

    // ── Input latching ──────────────────────────────────────────────────────
    reg signed [31:0] tau_lat [0:5];   // τ_ext per joint
    reg signed [31:0] qe_lat  [0:5];   // q_error per joint
    reg signed [31:0] Ks_nom_lat [0:5];
    reg signed [31:0] Ds_nom_lat [0:5];

    // ── State Machine ───────────────────────────────────────────────────────
    localparam S_IDLE      = 4'd0;
    localparam S_LATCH     = 4'd1;
    localparam S_DEADZONE  = 4'd2;
    localparam S_RLS_GAIN  = 4'd3;
    localparam S_RLS_KENV  = 4'd4;
    localparam S_RLS_P     = 4'd5;
    localparam S_ADAPT     = 4'd6;
    localparam S_NEXT      = 4'd7;
    localparam S_OUTPUT    = 4'd8;
    localparam S_DONE      = 4'd9;

    reg [3:0] state;
    reg [2:0] axis;

    reg signed [31:0] cur_gain;
    reg signed [31:0] cur_qe;

    reg signed [31:0] Ks_adapted [0:5];
    reg signed [31:0] Ds_adapted [0:5];

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done  <= 1'b0;
            axis  <= 3'd0;
            cur_gain <= 0;
            for (i = 0; i < 6; i = i + 1) begin
                K_env[i]      <= K_ENV_INIT;
                P[i]          <= P_INIT;
                Ks_adapted[i] <= 0;
                Ds_adapted[i] <= 0;
            end
            Ks_out_0 <= 0; Ks_out_1 <= 0; Ks_out_2 <= 0;
            Ks_out_3 <= 0; Ks_out_4 <= 0; Ks_out_5 <= 0;
            Ds_out_0 <= 0; Ds_out_1 <= 0; Ds_out_2 <= 0;
            Ds_out_3 <= 0; Ds_out_4 <= 0; Ds_out_5 <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) state <= S_LATCH;
                end

                S_LATCH: begin
                    tau_lat[0] <= tau_ext_0; tau_lat[1] <= tau_ext_1; tau_lat[2] <= tau_ext_2;
                    tau_lat[3] <= tau_ext_3; tau_lat[4] <= tau_ext_4; tau_lat[5] <= tau_ext_5;
                    qe_lat[0]  <= q_err_0;  qe_lat[1]  <= q_err_1;  qe_lat[2]  <= q_err_2;
                    qe_lat[3]  <= q_err_3;  qe_lat[4]  <= q_err_4;  qe_lat[5]  <= q_err_5;
                    Ks_nom_lat[0] <= Ks_nom_0; Ks_nom_lat[1] <= Ks_nom_1; Ks_nom_lat[2] <= Ks_nom_2;
                    Ks_nom_lat[3] <= Ks_nom_3; Ks_nom_lat[4] <= Ks_nom_4; Ks_nom_lat[5] <= Ks_nom_5;
                    Ds_nom_lat[0] <= Ds_nom_0; Ds_nom_lat[1] <= Ds_nom_1; Ds_nom_lat[2] <= Ds_nom_2;
                    Ds_nom_lat[3] <= Ds_nom_3; Ds_nom_lat[4] <= Ds_nom_4; Ds_nom_lat[5] <= Ds_nom_5;
                    axis  <= 3'd0;
                    state <= S_DEADZONE;
                end

                // Dead-zone check
                S_DEADZONE: begin
                    cur_qe <= qe_lat[axis];
                    if (!adaptive_enable || fabs(qe_lat[axis]) < Q_DEADZONE) begin
                        Ks_adapted[axis] <= Ks_nom_lat[axis];
                        Ds_adapted[axis] <= Ds_nom_lat[axis];
                        state <= S_NEXT;
                    end else begin
                        state <= S_RLS_GAIN;
                    end
                end

                // RLS Step 1: gain = P*q_err / (λ + q_err*P*q_err)
                S_RLS_GAIN: begin
                    cur_gain <= fdiv(
                        fmult(P[axis], cur_qe),
                        LAMBDA + fmult(cur_qe, fmult(P[axis], cur_qe))
                    );
                    state <= S_RLS_KENV;
                end

                // RLS Step 2: K_env += gain * (τ_ext - K_env * q_err)
                S_RLS_KENV: begin
                    K_env[axis] <= K_env[axis] + fmult(cur_gain,
                        tau_lat[axis] - fmult(K_env[axis], cur_qe));
                    state <= S_RLS_P;
                end

                // RLS Step 3: P = (1/λ) * (P - gain*q_err*P)
                S_RLS_P: begin
                    P[axis] <= fmult(INV_LAMBDA,
                        P[axis] - fmult(fmult(cur_gain, cur_qe), P[axis]));
                    state <= S_ADAPT;
                end

                // Compute adapted Ks
                S_ADAPT: begin
                    if (K_env[axis] > 0)
                        Ks_adapted[axis] <= fclamp(
                            fdiv(fmult(Ks_nom_lat[axis], Ks_nom_lat[axis]), K_env[axis]),
                            KS_MIN, KS_MAX);
                    else
                        Ks_adapted[axis] <= Ks_nom_lat[axis];
                    state <= S_NEXT;
                end

                // Compute Ds and advance
                S_NEXT: begin
                    if (Ks_nom_lat[axis] != 0)
                        Ds_adapted[axis] <= fmult(Ds_nom_lat[axis],
                            fdiv(Ks_adapted[axis], Ks_nom_lat[axis]));
                    else
                        Ds_adapted[axis] <= Ds_nom_lat[axis];

                    if (axis == 3'd5)
                        state <= S_OUTPUT;
                    else begin
                        axis  <= axis + 3'd1;
                        state <= S_DEADZONE;
                    end
                end

                S_OUTPUT: begin
                    Ks_out_0 <= Ks_adapted[0]; Ks_out_1 <= Ks_adapted[1]; Ks_out_2 <= Ks_adapted[2];
                    Ks_out_3 <= Ks_adapted[3]; Ks_out_4 <= Ks_adapted[4]; Ks_out_5 <= Ks_adapted[5];
                    Ds_out_0 <= Ds_adapted[0]; Ds_out_1 <= Ds_adapted[1]; Ds_out_2 <= Ds_adapted[2];
                    Ds_out_3 <= Ds_adapted[3]; Ds_out_4 <= Ds_adapted[4]; Ds_out_5 <= Ds_adapted[5];
                    state <= S_DONE;
                end

                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
