`timescale 1ns / 1ps

// velocity_estimator.v - Task-Space Velocity Estimator
// Estimates 6DOF velocity using first-order backward difference:
//   x_dot = (x[n] - x[n-1]) * INV_DT
// Includes optional first-order IIR low-pass filter:
//   x_dot_filt = alpha * x_dot_new + (1-alpha) * x_dot_old
//
// Q16.16 format throughout. Latency: ~12 clock cycles.

module velocity_estimator (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,       // Triggers velocity computation

    // Current pose (Q16.16)
    input  wire signed [31:0] x,     // position x (m)
    input  wire signed [31:0] y,     // position y (m)
    input  wire signed [31:0] z,     // position z (m)
    input  wire signed [31:0] rx,    // orientation rx (rad)
    input  wire signed [31:0] ry,    // orientation ry (rad)
    input  wire signed [31:0] rz,    // orientation rz (rad)

    // Estimated velocity (Q16.16)
    output reg  signed [31:0] vx,    // linear velocity x (m/s)
    output reg  signed [31:0] vy,    // linear velocity y (m/s)
    output reg  signed [31:0] vz,    // linear velocity z (m/s)
    output reg  signed [31:0] wx,    // angular velocity x (rad/s)
    output reg  signed [31:0] wy,    // angular velocity y (rad/s)
    output reg  signed [31:0] wz,    // angular velocity z (rad/s)

    output reg         done,
    output reg         valid         // High after first update (velocity is valid)
);

    // Configuration parameters
    // INV_DT: 1/dt in Q16.16
    // For 10kHz control rate: dt=0.0001s, 1/dt=10000 -> 10000*65536 = 655360000
    // For 100kHz control rate: dt=0.00001s, 1/dt=100000 -> would overflow Q16.16
    // Use a more moderate rate or scale differently
    // For 20kHz: dt=0.00005s, 1/dt=20000 -> 20000*65536 = 1310720000 (fits in 32-bit)
    parameter signed [31:0] INV_DT = 32'sd655360000; // 10kHz default

    // IIR filter coefficient: alpha in Q16.16
    // alpha = 0.3 -> 0.3 * 65536 = 19661
    // Higher alpha = less filtering, more responsive
    parameter signed [31:0] ALPHA = 32'sd19661;       // 0.3
    parameter signed [31:0] ONE_MINUS_ALPHA = 32'sd45875; // 0.7

    // State machine
    localparam IDLE  = 3'd0;
    localparam DIFF  = 3'd1;
    localparam SCALE = 3'd2;
    localparam FILTER= 3'd3;
    localparam DONE  = 3'd4;

    reg [2:0] state;

    // Previous pose
    reg signed [31:0] x_prev, y_prev, z_prev;
    reg signed [31:0] rx_prev, ry_prev, rz_prev;

    // Difference
    reg signed [31:0] dx, dy, dz, drx, dry, drz;

    // Raw velocity (before filtering)
    reg signed [31:0] vx_raw, vy_raw, vz_raw;
    reg signed [31:0] wx_raw, wy_raw, wz_raw;

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
            valid <= 1'b0;
            vx <= 0; vy <= 0; vz <= 0;
            wx <= 0; wy <= 0; wz <= 0;
            x_prev <= 0; y_prev <= 0; z_prev <= 0;
            rx_prev <= 0; ry_prev <= 0; rz_prev <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (enable) begin
                        // Compute differences
                        dx  <= x  - x_prev;
                        dy  <= y  - y_prev;
                        dz  <= z  - z_prev;
                        drx <= rx - rx_prev;
                        dry <= ry - ry_prev;
                        drz <= rz - rz_prev;
                        state <= DIFF;
                    end
                end

                DIFF: begin
                    // Scale by 1/dt to get velocity
                    // v = dx * INV_DT
                    // Note: dx is in Q16.16 meters, INV_DT is in Q16.16 (1/s)
                    // Product is Q32.32, take [47:16] for Q16.16 result
                    vx_raw <= fmult(dx, INV_DT);
                    vy_raw <= fmult(dy, INV_DT);
                    vz_raw <= fmult(dz, INV_DT);
                    wx_raw <= fmult(drx, INV_DT);
                    wy_raw <= fmult(dry, INV_DT);
                    wz_raw <= fmult(drz, INV_DT);
                    state <= FILTER;
                end

                FILTER: begin
                    if (!valid) begin
                        // First update: no filtering, just use raw value
                        vx <= vx_raw; vy <= vy_raw; vz <= vz_raw;
                        wx <= wx_raw; wy <= wy_raw; wz <= wz_raw;
                    end else begin
                        // IIR filter: v_filt = alpha * v_raw + (1-alpha) * v_old
                        vx <= fmult(ALPHA, vx_raw) + fmult(ONE_MINUS_ALPHA, vx);
                        vy <= fmult(ALPHA, vy_raw) + fmult(ONE_MINUS_ALPHA, vy);
                        vz <= fmult(ALPHA, vz_raw) + fmult(ONE_MINUS_ALPHA, vz);
                        wx <= fmult(ALPHA, wx_raw) + fmult(ONE_MINUS_ALPHA, wx);
                        wy <= fmult(ALPHA, wy_raw) + fmult(ONE_MINUS_ALPHA, wy);
                        wz <= fmult(ALPHA, wz_raw) + fmult(ONE_MINUS_ALPHA, wz);
                    end

                    // Update previous values
                    x_prev  <= x;  y_prev  <= y;  z_prev  <= z;
                    rx_prev <= rx; ry_prev <= ry; rz_prev <= rz;

                    valid <= 1'b1;
                    state <= DONE;
                end

                DONE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
