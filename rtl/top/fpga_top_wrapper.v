`timescale 1ns / 1ps

// fpga_top_wrapper.v - FPGA Top-Level Wrapper with Minimal I/O
//
// Alveo U200 is a PCIe accelerator card — all external I/O goes through
// PCIe/AXI. This wrapper internalizes ALL wide signals (config params,
// ADC data, encoder data, PWM outputs, debug) as memory-mapped registers,
// exposing only a simple parallel register bus (proxy for AXI-Lite).
//
// The FOC array and full control pipeline remain intact internally,
// preserving all LUT/FF/DSP utilization in synthesis reports.
//
// Physical I/O:
//   sys_clk, foc_clk, rst_n          = 3 pins
//   reg_addr[7:0]                     = 8 pins
//   reg_wdata[31:0]                   = 32 pins
//   reg_rdata[31:0]                   = 32 pins
//   reg_wr_en, reg_rd_en             = 2 pins
//   status (foc_init_done, pipeline_active) = 2 pins
//   ─────────────────────────────────────────
//   Total: 79 pins (~12% of 676 bonded IOBs)

module fpga_top_wrapper #(
    parameter signed [31:0] VEL_INV_DT = 32'sd655360000,
    parameter signed [31:0] VEL_ALPHA  = 32'sd19661,
    parameter signed [31:0] VEL_ONE_MINUS_ALPHA = 32'sd45875
) (
    // ====== Physical I/O (minimal) ======
    input  wire        sys_clk,
    input  wire        foc_clk,
    input  wire        rst_n,

    // Register bus (proxy for AXI-Lite over PCIe)
    input  wire [7:0]  reg_addr,
    input  wire [31:0] reg_wdata,
    input  wire        reg_wr_en,
    input  wire        reg_rd_en,
    output reg  [31:0] reg_rdata,

    // Status
    output wire        foc_all_init_done,
    output wire        pipeline_active_out
);

    // =====================================================================
    // Internal Configuration Registers (host writes via reg bus)
    // =====================================================================

    // Control enable
    reg ctrl_enable;

    // Joint angles (Q16.16 radians)
    reg signed [31:0] q0, q1, q2, q3, q4, q5;

    // Desired pose (Q16.16)
    reg signed [31:0] xd_x,  xd_y,  xd_z;
    reg signed [31:0] xd_rx, xd_ry, xd_rz;

    // Desired velocity (Q16.16)
    reg signed [31:0] xd_dot_x,  xd_dot_y,  xd_dot_z;
    reg signed [31:0] xd_dot_rx, xd_dot_ry, xd_dot_rz;

    // Stiffness gains (Q16.16)
    reg signed [31:0] Ks_x,  Ks_y,  Ks_z;
    reg signed [31:0] Ks_rx, Ks_ry, Ks_rz;

    // Damping gains (Q16.16)
    reg signed [31:0] Ds_x,  Ds_y,  Ds_z;
    reg signed [31:0] Ds_rx, Ds_ry, Ds_rz;

    // FOC PI parameters
    reg [30:0] foc_Kp, foc_Ki;

    // Encoder phi for FOC (12-bit each, packed into 32-bit regs)
    reg [11:0] phi_0, phi_1, phi_2, phi_3, phi_4, phi_5;

    // ADC data registers (host writes sampled ADC values)
    reg [11:0] adc_a_0, adc_b_0, adc_c_0;
    reg [11:0] adc_a_1, adc_b_1, adc_c_1;
    reg [11:0] adc_a_2, adc_b_2, adc_c_2;
    reg [11:0] adc_a_3, adc_b_3, adc_c_3;
    reg [11:0] adc_a_4, adc_b_4, adc_c_4;
    reg [11:0] adc_a_5, adc_b_5, adc_c_5;

    // ADC enable strobes (host triggers ADC valid)
    reg en_adc_0, en_adc_1, en_adc_2;
    reg en_adc_3, en_adc_4, en_adc_5;

    // =====================================================================
    // Internal wires from core → readable via reg bus
    // =====================================================================

    // ADC sample triggers from FOC (readable by host to know when to sample)
    wire sn_adc_0, sn_adc_1, sn_adc_2;
    wire sn_adc_3, sn_adc_4, sn_adc_5;

    // PWM outputs (captured internally, readable via reg bus)
    wire pwm_en_0, pwm_a_0, pwm_b_0, pwm_c_0;
    wire pwm_en_1, pwm_a_1, pwm_b_1, pwm_c_1;
    wire pwm_en_2, pwm_a_2, pwm_b_2, pwm_c_2;
    wire pwm_en_3, pwm_a_3, pwm_b_3, pwm_c_3;
    wire pwm_en_4, pwm_a_4, pwm_b_4, pwm_c_4;
    wire pwm_en_5, pwm_a_5, pwm_b_5, pwm_c_5;

    // Debug outputs from control pipeline
    wire signed [31:0] dbg_pose_x, dbg_pose_y, dbg_pose_z;
    wire signed [31:0] dbg_pose_rx, dbg_pose_ry, dbg_pose_rz;
    wire signed [31:0] dbg_F_x, dbg_F_y, dbg_F_z;
    wire signed [31:0] dbg_tau0, dbg_tau1, dbg_tau2;
    wire signed [31:0] dbg_tau3, dbg_tau4, dbg_tau5;
    wire [31:0] dbg_cycle_count;
    wire pipeline_active;
    wire foc_init_done;

    assign pipeline_active_out = pipeline_active;
    assign foc_all_init_done   = foc_init_done;

    // =====================================================================
    // Register Address Map
    // =====================================================================
    //
    // === Write Registers (host → FPGA) ===
    // 0x00       : ctrl_enable [0]
    // 0x01-0x06  : q0-q5                 (joint angles, Q16.16)
    // 0x10-0x15  : xd_x/y/z/rx/ry/rz    (desired pose)
    // 0x18-0x1D  : xd_dot_x~rz           (desired velocity)
    // 0x20-0x25  : Ks_x~rz               (stiffness)
    // 0x28-0x2D  : Ds_x~rz               (damping)
    // 0x30       : foc_Kp [30:0]
    // 0x31       : foc_Ki [30:0]
    // 0x32-0x34  : phi_0/1 [11:0|27:16], phi_2/3, phi_4/5  (packed pairs)
    // 0x38-0x3D  : adc_ch0{a[11:0],b[27:16]} ... adc_ch5   (packed ADC)
    // 0x3E       : en_adc_all [5:0]       (ADC valid strobes)
    //
    // === Read Registers (FPGA → host) ===
    // 0x40-0x45  : dbg_pose_x~rz
    // 0x48-0x4A  : dbg_F_x/y/z
    // 0x50-0x55  : dbg_tau0-5
    // 0x58       : dbg_cycle_count
    // 0x59       : {pipeline_active, foc_init_done}
    // 0x5A       : sn_adc_all [5:0]       (ADC sample triggers)
    // 0x5B-0x5D  : pwm_ch0/1 {en,a,b,c}, pwm_ch2/3, pwm_ch4/5

    // =====================================================================
    // Register Write Logic
    // =====================================================================
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_enable <= 0;
            q0 <= 0; q1 <= 0; q2 <= 0; q3 <= 0; q4 <= 0; q5 <= 0;
            xd_x <= 0; xd_y <= 0; xd_z <= 0;
            xd_rx <= 0; xd_ry <= 0; xd_rz <= 0;
            xd_dot_x <= 0; xd_dot_y <= 0; xd_dot_z <= 0;
            xd_dot_rx <= 0; xd_dot_ry <= 0; xd_dot_rz <= 0;
            Ks_x <= 0; Ks_y <= 0; Ks_z <= 0;
            Ks_rx <= 0; Ks_ry <= 0; Ks_rz <= 0;
            Ds_x <= 0; Ds_y <= 0; Ds_z <= 0;
            Ds_rx <= 0; Ds_ry <= 0; Ds_rz <= 0;
            foc_Kp <= 0; foc_Ki <= 0;
            phi_0 <= 0; phi_1 <= 0; phi_2 <= 0;
            phi_3 <= 0; phi_4 <= 0; phi_5 <= 0;
            adc_a_0 <= 0; adc_b_0 <= 0; adc_c_0 <= 0;
            adc_a_1 <= 0; adc_b_1 <= 0; adc_c_1 <= 0;
            adc_a_2 <= 0; adc_b_2 <= 0; adc_c_2 <= 0;
            adc_a_3 <= 0; adc_b_3 <= 0; adc_c_3 <= 0;
            adc_a_4 <= 0; adc_b_4 <= 0; adc_c_4 <= 0;
            adc_a_5 <= 0; adc_b_5 <= 0; adc_c_5 <= 0;
            en_adc_0 <= 0; en_adc_1 <= 0; en_adc_2 <= 0;
            en_adc_3 <= 0; en_adc_4 <= 0; en_adc_5 <= 0;
        end else begin
            // Auto-clear ADC enable strobes after 1 cycle
            en_adc_0 <= 0; en_adc_1 <= 0; en_adc_2 <= 0;
            en_adc_3 <= 0; en_adc_4 <= 0; en_adc_5 <= 0;

            if (reg_wr_en) begin
                case (reg_addr)
                    8'h00: ctrl_enable <= reg_wdata[0];
                    // Joint angles
                    8'h01: q0 <= $signed(reg_wdata);
                    8'h02: q1 <= $signed(reg_wdata);
                    8'h03: q2 <= $signed(reg_wdata);
                    8'h04: q3 <= $signed(reg_wdata);
                    8'h05: q4 <= $signed(reg_wdata);
                    8'h06: q5 <= $signed(reg_wdata);
                    // Desired pose
                    8'h10: xd_x  <= $signed(reg_wdata);
                    8'h11: xd_y  <= $signed(reg_wdata);
                    8'h12: xd_z  <= $signed(reg_wdata);
                    8'h13: xd_rx <= $signed(reg_wdata);
                    8'h14: xd_ry <= $signed(reg_wdata);
                    8'h15: xd_rz <= $signed(reg_wdata);
                    // Desired velocity
                    8'h18: xd_dot_x  <= $signed(reg_wdata);
                    8'h19: xd_dot_y  <= $signed(reg_wdata);
                    8'h1A: xd_dot_z  <= $signed(reg_wdata);
                    8'h1B: xd_dot_rx <= $signed(reg_wdata);
                    8'h1C: xd_dot_ry <= $signed(reg_wdata);
                    8'h1D: xd_dot_rz <= $signed(reg_wdata);
                    // Stiffness
                    8'h20: Ks_x  <= $signed(reg_wdata);
                    8'h21: Ks_y  <= $signed(reg_wdata);
                    8'h22: Ks_z  <= $signed(reg_wdata);
                    8'h23: Ks_rx <= $signed(reg_wdata);
                    8'h24: Ks_ry <= $signed(reg_wdata);
                    8'h25: Ks_rz <= $signed(reg_wdata);
                    // Damping
                    8'h28: Ds_x  <= $signed(reg_wdata);
                    8'h29: Ds_y  <= $signed(reg_wdata);
                    8'h2A: Ds_z  <= $signed(reg_wdata);
                    8'h2B: Ds_rx <= $signed(reg_wdata);
                    8'h2C: Ds_ry <= $signed(reg_wdata);
                    8'h2D: Ds_rz <= $signed(reg_wdata);
                    // FOC PI
                    8'h30: foc_Kp <= reg_wdata[30:0];
                    8'h31: foc_Ki <= reg_wdata[30:0];
                    // Encoder phi (packed pairs: [11:0]=even, [27:16]=odd)
                    8'h32: begin phi_0 <= reg_wdata[11:0]; phi_1 <= reg_wdata[27:16]; end
                    8'h33: begin phi_2 <= reg_wdata[11:0]; phi_3 <= reg_wdata[27:16]; end
                    8'h34: begin phi_4 <= reg_wdata[11:0]; phi_5 <= reg_wdata[27:16]; end
                    // ADC data (packed: [11:0]=phase_a, [27:16]=phase_b)
                    // Phase C written separately via next address
                    8'h38: begin adc_a_0 <= reg_wdata[11:0]; adc_b_0 <= reg_wdata[27:16]; end
                    8'h39: begin adc_c_0 <= reg_wdata[11:0]; en_adc_0 <= 1'b1; end
                    8'h3A: begin adc_a_1 <= reg_wdata[11:0]; adc_b_1 <= reg_wdata[27:16]; end
                    8'h3B: begin adc_c_1 <= reg_wdata[11:0]; en_adc_1 <= 1'b1; end
                    8'h3C: begin adc_a_2 <= reg_wdata[11:0]; adc_b_2 <= reg_wdata[27:16]; end
                    8'h3D: begin adc_c_2 <= reg_wdata[11:0]; en_adc_2 <= 1'b1; end
                    8'h3E: begin adc_a_3 <= reg_wdata[11:0]; adc_b_3 <= reg_wdata[27:16]; end
                    8'h3F: begin adc_c_3 <= reg_wdata[11:0]; en_adc_3 <= 1'b1; end
                    8'h70: begin adc_a_4 <= reg_wdata[11:0]; adc_b_4 <= reg_wdata[27:16]; end
                    8'h71: begin adc_c_4 <= reg_wdata[11:0]; en_adc_4 <= 1'b1; end
                    8'h72: begin adc_a_5 <= reg_wdata[11:0]; adc_b_5 <= reg_wdata[27:16]; end
                    8'h73: begin adc_c_5 <= reg_wdata[11:0]; en_adc_5 <= 1'b1; end
                    default: ;
                endcase
            end
        end
    end

    // =====================================================================
    // Register Read Logic
    // =====================================================================
    always @(posedge sys_clk) begin
        if (reg_rd_en) begin
            case (reg_addr)
                // Read back config
                8'h00: reg_rdata <= {31'd0, ctrl_enable};
                8'h01: reg_rdata <= q0;
                8'h02: reg_rdata <= q1;
                8'h03: reg_rdata <= q2;
                8'h04: reg_rdata <= q3;
                8'h05: reg_rdata <= q4;
                8'h06: reg_rdata <= q5;
                // Debug: pose
                8'h40: reg_rdata <= dbg_pose_x;
                8'h41: reg_rdata <= dbg_pose_y;
                8'h42: reg_rdata <= dbg_pose_z;
                8'h43: reg_rdata <= dbg_pose_rx;
                8'h44: reg_rdata <= dbg_pose_ry;
                8'h45: reg_rdata <= dbg_pose_rz;
                // Debug: force
                8'h48: reg_rdata <= dbg_F_x;
                8'h49: reg_rdata <= dbg_F_y;
                8'h4A: reg_rdata <= dbg_F_z;
                // Debug: torque
                8'h50: reg_rdata <= dbg_tau0;
                8'h51: reg_rdata <= dbg_tau1;
                8'h52: reg_rdata <= dbg_tau2;
                8'h53: reg_rdata <= dbg_tau3;
                8'h54: reg_rdata <= dbg_tau4;
                8'h55: reg_rdata <= dbg_tau5;
                // Debug: cycle count & status
                8'h58: reg_rdata <= dbg_cycle_count;
                8'h59: reg_rdata <= {30'd0, pipeline_active, foc_init_done};
                // FOC: ADC sample triggers
                8'h5A: reg_rdata <= {26'd0, sn_adc_5, sn_adc_4, sn_adc_3,
                                     sn_adc_2, sn_adc_1, sn_adc_0};
                // FOC: PWM outputs (packed: [3:0]=ch0{en,a,b,c}, [7:4]=ch1...)
                8'h5B: reg_rdata <= {24'd0,
                                     pwm_en_1, pwm_a_1, pwm_b_1, pwm_c_1,
                                     pwm_en_0, pwm_a_0, pwm_b_0, pwm_c_0};
                8'h5C: reg_rdata <= {24'd0,
                                     pwm_en_3, pwm_a_3, pwm_b_3, pwm_c_3,
                                     pwm_en_2, pwm_a_2, pwm_b_2, pwm_c_2};
                8'h5D: reg_rdata <= {24'd0,
                                     pwm_en_5, pwm_a_5, pwm_b_5, pwm_c_5,
                                     pwm_en_4, pwm_a_4, pwm_b_4, pwm_c_4};
                default: reg_rdata <= 32'hDEAD_BEEF;
            endcase
        end
    end

    // =====================================================================
    // Core Instance — impedance_control_top (complete pipeline + FOC)
    // =====================================================================
    impedance_control_top #(
        .VEL_INV_DT(VEL_INV_DT),
        .VEL_ALPHA(VEL_ALPHA),
        .VEL_ONE_MINUS_ALPHA(VEL_ONE_MINUS_ALPHA)
    ) core_inst (
        .sys_clk        (sys_clk),
        .rst_n          (rst_n),
        .foc_clk        (foc_clk),
        .ctrl_enable    (ctrl_enable),
        // Desired pose
        .xd_x(xd_x), .xd_y(xd_y), .xd_z(xd_z),
        .xd_rx(xd_rx), .xd_ry(xd_ry), .xd_rz(xd_rz),
        // Desired velocity
        .xd_dot_x(xd_dot_x), .xd_dot_y(xd_dot_y), .xd_dot_z(xd_dot_z),
        .xd_dot_rx(xd_dot_rx), .xd_dot_ry(xd_dot_ry), .xd_dot_rz(xd_dot_rz),
        // Stiffness
        .Ks_x(Ks_x), .Ks_y(Ks_y), .Ks_z(Ks_z),
        .Ks_rx(Ks_rx), .Ks_ry(Ks_ry), .Ks_rz(Ks_rz),
        // Damping
        .Ds_x(Ds_x), .Ds_y(Ds_y), .Ds_z(Ds_z),
        .Ds_rx(Ds_rx), .Ds_ry(Ds_ry), .Ds_rz(Ds_rz),
        // Joint angles
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        // FOC PI
        .foc_Kp(foc_Kp), .foc_Ki(foc_Ki),
        // Encoder phi (internal registers)
        .phi_0(phi_0), .phi_1(phi_1), .phi_2(phi_2),
        .phi_3(phi_3), .phi_4(phi_4), .phi_5(phi_5),
        // ADC (internal registers, not physical pins)
        .sn_adc_0(sn_adc_0), .sn_adc_1(sn_adc_1), .sn_adc_2(sn_adc_2),
        .sn_adc_3(sn_adc_3), .sn_adc_4(sn_adc_4), .sn_adc_5(sn_adc_5),
        .en_adc_0(en_adc_0), .en_adc_1(en_adc_1), .en_adc_2(en_adc_2),
        .en_adc_3(en_adc_3), .en_adc_4(en_adc_4), .en_adc_5(en_adc_5),
        .adc_a_0(adc_a_0), .adc_b_0(adc_b_0), .adc_c_0(adc_c_0),
        .adc_a_1(adc_a_1), .adc_b_1(adc_b_1), .adc_c_1(adc_c_1),
        .adc_a_2(adc_a_2), .adc_b_2(adc_b_2), .adc_c_2(adc_c_2),
        .adc_a_3(adc_a_3), .adc_b_3(adc_b_3), .adc_c_3(adc_c_3),
        .adc_a_4(adc_a_4), .adc_b_4(adc_b_4), .adc_c_4(adc_c_4),
        .adc_a_5(adc_a_5), .adc_b_5(adc_b_5), .adc_c_5(adc_c_5),
        // PWM (internal wires, not physical pins)
        .pwm_en_0(pwm_en_0), .pwm_a_0(pwm_a_0), .pwm_b_0(pwm_b_0), .pwm_c_0(pwm_c_0),
        .pwm_en_1(pwm_en_1), .pwm_a_1(pwm_a_1), .pwm_b_1(pwm_b_1), .pwm_c_1(pwm_c_1),
        .pwm_en_2(pwm_en_2), .pwm_a_2(pwm_a_2), .pwm_b_2(pwm_b_2), .pwm_c_2(pwm_c_2),
        .pwm_en_3(pwm_en_3), .pwm_a_3(pwm_a_3), .pwm_b_3(pwm_b_3), .pwm_c_3(pwm_c_3),
        .pwm_en_4(pwm_en_4), .pwm_a_4(pwm_a_4), .pwm_b_4(pwm_b_4), .pwm_c_4(pwm_c_4),
        .pwm_en_5(pwm_en_5), .pwm_a_5(pwm_a_5), .pwm_b_5(pwm_b_5), .pwm_c_5(pwm_c_5),
        // Debug
        .dbg_pose_x(dbg_pose_x), .dbg_pose_y(dbg_pose_y), .dbg_pose_z(dbg_pose_z),
        .dbg_pose_rx(dbg_pose_rx), .dbg_pose_ry(dbg_pose_ry), .dbg_pose_rz(dbg_pose_rz),
        .dbg_F_x(dbg_F_x), .dbg_F_y(dbg_F_y), .dbg_F_z(dbg_F_z),
        .dbg_tau0(dbg_tau0), .dbg_tau1(dbg_tau1), .dbg_tau2(dbg_tau2),
        .dbg_tau3(dbg_tau3), .dbg_tau4(dbg_tau4), .dbg_tau5(dbg_tau5),
        .dbg_cycle_count(dbg_cycle_count),
        .foc_all_init_done(foc_init_done),
        .pipeline_active(pipeline_active)
    );

endmodule
