# synth_report.tcl - Vivado Synthesis Script for Resource Utilization Report
# Synthesizes impedance_control_top (without FOC) for resource counting
# Usage: vivado -mode batch -source synth_report.tcl

# Create in-memory project
create_project -in_memory -part xcu200-fsgd2104-2-e

# Add source files (control pipeline only, no FOC for clean resource count)
set rtl_dir "D:/robot/impedance_control/rtl"

# Common modules
read_verilog "$rtl_dir/common/cordic_sincos.v"
read_verilog "$rtl_dir/common/cordic_atan2.v"
read_verilog "$rtl_dir/common/fixed_mult.v"
read_verilog "$rtl_dir/common/format_converter.v"

# Kinematics
read_verilog "$rtl_dir/kinematics/part1_config.v"
read_verilog "$rtl_dir/kinematics/fk_engine_ext.v"
read_verilog "$rtl_dir/kinematics/analytical_jacobian.v"

# Control pipeline
read_verilog "$rtl_dir/control/velocity_estimator.v"
read_verilog "$rtl_dir/control/impedance_controller.v"
read_verilog "$rtl_dir/control/matrix_transpose.v"
read_verilog "$rtl_dir/control/matrix_vector_mult.v"
read_verilog "$rtl_dir/control/jt_force_mapper.v"

# FOC integration
read_verilog "$rtl_dir/foc/torque_to_iq.v"

# Set top module - we synthesize analytical_jacobian + control pipeline separately
# to get clean per-module resource counts

# First: synthesize analytical_jacobian alone
set_property top analytical_jacobian [current_fileset]
synth_design -top analytical_jacobian -part xcu200-fsgd2104-2-e -mode out_of_context
report_utilization -file "D:/robot/impedance_control/reports/utilization_jacobian.txt"
report_timing_summary -file "D:/robot/impedance_control/reports/timing_jacobian.txt"

puts "=== Analytical Jacobian Resource Utilization ==="
report_utilization

# Reset and synthesize impedance control pipeline modules
# Create a wrapper for just the control pipeline
close_design

# Synthesize velocity_estimator
set_property top velocity_estimator [current_fileset]
synth_design -top velocity_estimator -part xcu200-fsgd2104-2-e -mode out_of_context
puts "\n=== Velocity Estimator Resource Utilization ==="
report_utilization
report_utilization -file "D:/robot/impedance_control/reports/utilization_velocity_est.txt"
close_design

# Synthesize impedance_controller
set_property top impedance_controller [current_fileset]
synth_design -top impedance_controller -part xcu200-fsgd2104-2-e -mode out_of_context
puts "\n=== Impedance Controller Resource Utilization ==="
report_utilization
report_utilization -file "D:/robot/impedance_control/reports/utilization_impedance_ctrl.txt"
close_design

# Synthesize jt_force_mapper
set_property top jt_force_mapper [current_fileset]
synth_design -top jt_force_mapper -part xcu200-fsgd2104-2-e -mode out_of_context
puts "\n=== J^T Force Mapper Resource Utilization ==="
report_utilization
report_utilization -file "D:/robot/impedance_control/reports/utilization_jt_mapper.txt"
close_design

# Synthesize torque_to_iq
set_property top torque_to_iq [current_fileset]
synth_design -top torque_to_iq -part xcu200-fsgd2104-2-e -mode out_of_context
puts "\n=== Torque to Iq Resource Utilization ==="
report_utilization
report_utilization -file "D:/robot/impedance_control/reports/utilization_torque_to_iq.txt"
close_design

puts "\n=== All synthesis reports generated ==="
puts "Reports saved to D:/robot/impedance_control/reports/"
exit
