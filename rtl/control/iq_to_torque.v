`timescale 1ns / 1ps

// iq_to_torque.v - Convert measured motor current to joint torque
//
// τ[i] = Kt[i] * iq_actual[i]
//
// iq_actual: signed 16-bit from FOC ADC measurement
// Kt: torque constant per motor (Q16.16, Nm/A)
// τ: joint torque (Q16.16, Nm)
//
// Latency: 1 cycle (combinational multiply + register)

module iq_to_torque (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // Measured iq from FOC (signed 16-bit)
    input  wire signed [15:0] iq_actual_0, iq_actual_1, iq_actual_2,
    input  wire signed [15:0] iq_actual_3, iq_actual_4, iq_actual_5,

    // Torque constants (Q16.16, Nm/A)
    input  wire signed [31:0] Kt_0, Kt_1, Kt_2,
    input  wire signed [31:0] Kt_3, Kt_4, Kt_5,

    // Output: measured joint torques (Q16.16, Nm)
    output reg  signed [31:0] tau_meas_0, tau_meas_1, tau_meas_2,
    output reg  signed [31:0] tau_meas_3, tau_meas_4, tau_meas_5,

    output reg         done
);

    // Q16.16 multiply: Kt (Q16.16) × iq (int16 treated as Q16.0)
    // Result: Q32.16, take [47:16] → Q16.16
    function signed [31:0] kt_mult;
        input signed [31:0] kt;
        input signed [15:0] iq;
        reg signed [63:0] temp;
        begin
            temp = $signed(kt) * $signed({{16{iq[15]}}, iq});
            kt_mult = temp[47:16];
        end
    endfunction

    reg state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 1'b0;
            done  <= 1'b0;
            tau_meas_0 <= 0; tau_meas_1 <= 0; tau_meas_2 <= 0;
            tau_meas_3 <= 0; tau_meas_4 <= 0; tau_meas_5 <= 0;
        end else begin
            case (state)
                1'b0: begin
                    done <= 1'b0;
                    if (start) begin
                        tau_meas_0 <= kt_mult(Kt_0, iq_actual_0);
                        tau_meas_1 <= kt_mult(Kt_1, iq_actual_1);
                        tau_meas_2 <= kt_mult(Kt_2, iq_actual_2);
                        tau_meas_3 <= kt_mult(Kt_3, iq_actual_3);
                        tau_meas_4 <= kt_mult(Kt_4, iq_actual_4);
                        tau_meas_5 <= kt_mult(Kt_5, iq_actual_5);
                        state <= 1'b1;
                    end
                end

                1'b1: begin
                    done  <= 1'b1;
                    state <= 1'b0;
                end
            endcase
        end
    end

endmodule
