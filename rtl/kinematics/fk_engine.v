`timescale 1ns / 1ps

// fk_engine.v - Forward Kinematics Engine (Verilog-2001 compatible)
// Computes end-effector pose from joint angles using DH convention

module fk_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    // Joint angles (Q16.16 radians) - expanded
    input  wire signed [31:0] q0, q1, q2, q3, q4, q5,
    // Output transformation matrix (4x4, row-major, Q16.16) - expanded
    output reg  signed [31:0] T00, T01, T02, T03,
    output reg  signed [31:0] T10, T11, T12, T13,
    output reg  signed [31:0] T20, T21, T22, T23,
    output reg  signed [31:0] T30, T31, T32, T33,
    // Output pose (x, y, z, rx, ry, rz in Q16.16) - expanded
    output reg  signed [31:0] pose_x, pose_y, pose_z,
    output reg  signed [31:0] pose_rx, pose_ry, pose_rz,
    output reg         done
);

    // State machine
    localparam IDLE        = 4'd0;
    localparam LOAD_DH     = 4'd1;
    localparam CALC_TRIG   = 4'd2;
    localparam WAIT_TRIG   = 4'd3;
    localparam CALC_Ti     = 4'd4;
    localparam MULT_MATRIX = 4'd5;
    localparam NEXT_JOINT  = 4'd6;
    localparam EXTRACT_POSE= 4'd7;
    localparam WAIT_ATAN1  = 4'd8;
    localparam WAIT_ATAN2  = 4'd9;
    localparam WAIT_ATAN3  = 4'd10;
    localparam DONE_ST     = 4'd11;

    reg [3:0] state;
    reg [2:0] joint_idx;
    reg [3:0] mult_step;

    // Joint angle storage
    reg signed [31:0] q_arr [0:5];

    // DH parameters for current joint
    reg signed [31:0] dh_d, dh_a, dh_alpha;
    wire signed [31:0] dh_d_w, dh_a_w, dh_alpha_w;
    wire signed [31:0] q_min_w, q_max_w;

    // CORDIC interface
    reg cordic_start;
    reg signed [31:0] cordic_angle;
    wire signed [31:0] cordic_sin, cordic_cos;
    wire cordic_done;

    // Trigonometric values
    reg signed [31:0] sin_q, cos_q;
    reg signed [31:0] sin_alpha, cos_alpha;
    reg trig_phase;

    // Individual transformation matrix Ti (4x4)
    reg signed [31:0] Ti00, Ti01, Ti02, Ti03;
    reg signed [31:0] Ti10, Ti11, Ti12, Ti13;
    reg signed [31:0] Ti20, Ti21, Ti22, Ti23;
    reg signed [31:0] Ti30, Ti31, Ti32, Ti33;

    // Accumulated transformation matrix T (4x4)
    reg signed [31:0] Ta00, Ta01, Ta02, Ta03;
    reg signed [31:0] Ta10, Ta11, Ta12, Ta13;
    reg signed [31:0] Ta20, Ta21, Ta22, Ta23;
    reg signed [31:0] Ta30, Ta31, Ta32, Ta33;

    // Temporary matrix for multiplication
    reg signed [31:0] Tt00, Tt01, Tt02, Tt03;
    reg signed [31:0] Tt10, Tt11, Tt12, Tt13;
    reg signed [31:0] Tt20, Tt21, Tt22, Tt23;
    reg signed [31:0] Tt30, Tt31, Tt32, Tt33;

    // CORDIC atan2 for pose extraction
    reg atan_start;
    reg signed [31:0] atan_x, atan_y;
    wire signed [31:0] atan_angle, atan_mag;
    wire atan_done;

    // DH configuration module
    part1_config_comb dh_config (
        .joint_idx(joint_idx),
        .dh_d(dh_d_w),
        .dh_a(dh_a_w),
        .dh_alpha(dh_alpha_w),
        .q_min(q_min_w),
        .q_max(q_max_w)
    );

    // CORDIC sincos module
    cordic_sincos cordic_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(cordic_start),
        .angle(cordic_angle),
        .sin_out(cordic_sin),
        .cos_out(cordic_cos),
        .done(cordic_done)
    );

    // CORDIC atan2 module
    cordic_atan2 atan2_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(atan_start),
        .x_in(atan_x),
        .y_in(atan_y),
        .angle(atan_angle),
        .magnitude(atan_mag),
        .done(atan_done)
    );

    // Fixed point multiplication
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

    // Constants
    localparam signed [31:0] FIXED_ONE  = 32'sd65536;
    localparam signed [31:0] FIXED_ZERO = 32'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0;
            state <= IDLE;
            joint_idx <= 3'd0;
            cordic_start <= 1'b0;
            atan_start <= 1'b0;
            trig_phase <= 1'b0;
            mult_step <= 4'd0;

            // Initialize T_acc to identity
            Ta00 <= FIXED_ONE;  Ta01 <= FIXED_ZERO; Ta02 <= FIXED_ZERO; Ta03 <= FIXED_ZERO;
            Ta10 <= FIXED_ZERO; Ta11 <= FIXED_ONE;  Ta12 <= FIXED_ZERO; Ta13 <= FIXED_ZERO;
            Ta20 <= FIXED_ZERO; Ta21 <= FIXED_ZERO; Ta22 <= FIXED_ONE;  Ta23 <= FIXED_ZERO;
            Ta30 <= FIXED_ZERO; Ta31 <= FIXED_ZERO; Ta32 <= FIXED_ZERO; Ta33 <= FIXED_ONE;

            // Initialize outputs
            T00 <= FIXED_ZERO; T01 <= FIXED_ZERO; T02 <= FIXED_ZERO; T03 <= FIXED_ZERO;
            T10 <= FIXED_ZERO; T11 <= FIXED_ZERO; T12 <= FIXED_ZERO; T13 <= FIXED_ZERO;
            T20 <= FIXED_ZERO; T21 <= FIXED_ZERO; T22 <= FIXED_ZERO; T23 <= FIXED_ZERO;
            T30 <= FIXED_ZERO; T31 <= FIXED_ZERO; T32 <= FIXED_ZERO; T33 <= FIXED_ZERO;
            pose_x <= FIXED_ZERO; pose_y <= FIXED_ZERO; pose_z <= FIXED_ZERO;
            pose_rx <= FIXED_ZERO; pose_ry <= FIXED_ZERO; pose_rz <= FIXED_ZERO;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Store joint angles
                        q_arr[0] <= q0; q_arr[1] <= q1; q_arr[2] <= q2;
                        q_arr[3] <= q3; q_arr[4] <= q4; q_arr[5] <= q5;
                        joint_idx <= 3'd0;
                        // Initialize T_acc to identity
                        Ta00 <= FIXED_ONE;  Ta01 <= FIXED_ZERO; Ta02 <= FIXED_ZERO; Ta03 <= FIXED_ZERO;
                        Ta10 <= FIXED_ZERO; Ta11 <= FIXED_ONE;  Ta12 <= FIXED_ZERO; Ta13 <= FIXED_ZERO;
                        Ta20 <= FIXED_ZERO; Ta21 <= FIXED_ZERO; Ta22 <= FIXED_ONE;  Ta23 <= FIXED_ZERO;
                        Ta30 <= FIXED_ZERO; Ta31 <= FIXED_ZERO; Ta32 <= FIXED_ZERO; Ta33 <= FIXED_ONE;
                        state <= LOAD_DH;
                    end
                end

                LOAD_DH: begin
                    dh_d <= dh_d_w;
                    dh_a <= dh_a_w;
                    dh_alpha <= dh_alpha_w;
                    trig_phase <= 1'b0;
                    state <= CALC_TRIG;
                end

                CALC_TRIG: begin
                    if (!trig_phase) begin
                        cordic_angle <= q_arr[joint_idx];
                    end else begin
                        cordic_angle <= dh_alpha;
                    end
                    cordic_start <= 1'b1;
                    state <= WAIT_TRIG;
                end

                WAIT_TRIG: begin
                    cordic_start <= 1'b0;
                    if (cordic_done) begin
                        if (!trig_phase) begin
                            sin_q <= cordic_sin;
                            cos_q <= cordic_cos;
                            trig_phase <= 1'b1;
                            state <= CALC_TRIG;
                        end else begin
                            sin_alpha <= cordic_sin;
                            cos_alpha <= cordic_cos;
                            state <= CALC_Ti;
                        end
                    end
                end

                CALC_Ti: begin
                    // Build DH transformation matrix Ti
                    Ti00 <= cos_q;
                    Ti01 <= -fmult(sin_q, cos_alpha);
                    Ti02 <= fmult(sin_q, sin_alpha);
                    Ti03 <= fmult(dh_a, cos_q);

                    Ti10 <= sin_q;
                    Ti11 <= fmult(cos_q, cos_alpha);
                    Ti12 <= -fmult(cos_q, sin_alpha);
                    Ti13 <= fmult(dh_a, sin_q);

                    Ti20 <= FIXED_ZERO;
                    Ti21 <= sin_alpha;
                    Ti22 <= cos_alpha;
                    Ti23 <= dh_d;

                    Ti30 <= FIXED_ZERO;
                    Ti31 <= FIXED_ZERO;
                    Ti32 <= FIXED_ZERO;
                    Ti33 <= FIXED_ONE;

                    mult_step <= 4'd0;
                    state <= MULT_MATRIX;
                end

                MULT_MATRIX: begin
                    case (mult_step)
                        4'd0: begin
                            Tt00 <= fmult(Ta00, Ti00) + fmult(Ta01, Ti10) + fmult(Ta02, Ti20) + fmult(Ta03, Ti30);
                            Tt01 <= fmult(Ta00, Ti01) + fmult(Ta01, Ti11) + fmult(Ta02, Ti21) + fmult(Ta03, Ti31);
                            Tt02 <= fmult(Ta00, Ti02) + fmult(Ta01, Ti12) + fmult(Ta02, Ti22) + fmult(Ta03, Ti32);
                            Tt03 <= fmult(Ta00, Ti03) + fmult(Ta01, Ti13) + fmult(Ta02, Ti23) + fmult(Ta03, Ti33);
                            mult_step <= 4'd1;
                        end
                        4'd1: begin
                            Tt10 <= fmult(Ta10, Ti00) + fmult(Ta11, Ti10) + fmult(Ta12, Ti20) + fmult(Ta13, Ti30);
                            Tt11 <= fmult(Ta10, Ti01) + fmult(Ta11, Ti11) + fmult(Ta12, Ti21) + fmult(Ta13, Ti31);
                            Tt12 <= fmult(Ta10, Ti02) + fmult(Ta11, Ti12) + fmult(Ta12, Ti22) + fmult(Ta13, Ti32);
                            Tt13 <= fmult(Ta10, Ti03) + fmult(Ta11, Ti13) + fmult(Ta12, Ti23) + fmult(Ta13, Ti33);
                            mult_step <= 4'd2;
                        end
                        4'd2: begin
                            Tt20 <= fmult(Ta20, Ti00) + fmult(Ta21, Ti10) + fmult(Ta22, Ti20) + fmult(Ta23, Ti30);
                            Tt21 <= fmult(Ta20, Ti01) + fmult(Ta21, Ti11) + fmult(Ta22, Ti21) + fmult(Ta23, Ti31);
                            Tt22 <= fmult(Ta20, Ti02) + fmult(Ta21, Ti12) + fmult(Ta22, Ti22) + fmult(Ta23, Ti32);
                            Tt23 <= fmult(Ta20, Ti03) + fmult(Ta21, Ti13) + fmult(Ta22, Ti23) + fmult(Ta23, Ti33);
                            mult_step <= 4'd3;
                        end
                        4'd3: begin
                            Tt30 <= fmult(Ta30, Ti00) + fmult(Ta31, Ti10) + fmult(Ta32, Ti20) + fmult(Ta33, Ti30);
                            Tt31 <= fmult(Ta30, Ti01) + fmult(Ta31, Ti11) + fmult(Ta32, Ti21) + fmult(Ta33, Ti31);
                            Tt32 <= fmult(Ta30, Ti02) + fmult(Ta31, Ti12) + fmult(Ta32, Ti22) + fmult(Ta33, Ti32);
                            Tt33 <= fmult(Ta30, Ti03) + fmult(Ta31, Ti13) + fmult(Ta32, Ti23) + fmult(Ta33, Ti33);
                            mult_step <= 4'd4;
                        end
                        4'd4: begin
                            Ta00 <= Tt00; Ta01 <= Tt01; Ta02 <= Tt02; Ta03 <= Tt03;
                            Ta10 <= Tt10; Ta11 <= Tt11; Ta12 <= Tt12; Ta13 <= Tt13;
                            Ta20 <= Tt20; Ta21 <= Tt21; Ta22 <= Tt22; Ta23 <= Tt23;
                            Ta30 <= Tt30; Ta31 <= Tt31; Ta32 <= Tt32; Ta33 <= Tt33;
                            state <= NEXT_JOINT;
                        end
                        default: mult_step <= 4'd0;
                    endcase
                end

                NEXT_JOINT: begin
                    if (joint_idx == 3'd5) begin
                        // Copy to output
                        T00 <= Ta00; T01 <= Ta01; T02 <= Ta02; T03 <= Ta03;
                        T10 <= Ta10; T11 <= Ta11; T12 <= Ta12; T13 <= Ta13;
                        T20 <= Ta20; T21 <= Ta21; T22 <= Ta22; T23 <= Ta23;
                        T30 <= Ta30; T31 <= Ta31; T32 <= Ta32; T33 <= Ta33;
                        state <= EXTRACT_POSE;
                    end else begin
                        joint_idx <= joint_idx + 1;
                        state <= LOAD_DH;
                    end
                end

                EXTRACT_POSE: begin
                    // Extract position directly
                    pose_x <= Ta03;
                    pose_y <= Ta13;
                    pose_z <= Ta23;
                    // Start atan2 for rz = atan2(r10, r00)
                    atan_x <= Ta00;
                    atan_y <= Ta10;
                    atan_start <= 1'b1;
                    state <= WAIT_ATAN1;
                end

                WAIT_ATAN1: begin
                    atan_start <= 1'b0;
                    if (atan_done) begin
                        pose_rz <= atan_angle;
                        // atan2 for ry = atan2(-r20, sqrt(r00^2+r10^2))
                        atan_x <= atan_mag;
                        atan_y <= -Ta20;
                        atan_start <= 1'b1;
                        state <= WAIT_ATAN2;
                    end
                end

                WAIT_ATAN2: begin
                    atan_start <= 1'b0;
                    if (atan_done) begin
                        pose_ry <= atan_angle;
                        // atan2 for rx = atan2(r21, r22)
                        atan_x <= Ta22;
                        atan_y <= Ta21;
                        atan_start <= 1'b1;
                        state <= WAIT_ATAN3;
                    end
                end

                WAIT_ATAN3: begin
                    atan_start <= 1'b0;
                    if (atan_done) begin
                        pose_rx <= atan_angle;
                        state <= DONE_ST;
                    end
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
