# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 FPGA 的 6-DOF 机械臂（EPDobot）任务空间阻抗控制器。包含 Verilog RTL、仿真 testbench、Python 分析/绘图脚本，以及中文学位论文章节草稿（`第*章_*.txt`）。目标器件为 Xilinx UltraScale+ (xcu200-fsgd2104-2-e / Alveo U200)。

所有运算使用 Q16.16 有符号定点数（32 位）。机器人 DH 参数定义在 `rtl/kinematics/part1_config.v`。

## 控制流水线架构

双时钟域设计：
- `sys_clk`（100 MHz）— 控制流水线，由 `impedance_control_top` 中的 12 状态 FSM（`S_IDLE` → `S_FK_START` → ... → `S_DONE`）顺序调度
- `foc_clk`（73.728 MHz）— FOC 电机控制，异步时钟域（`set_clock_groups -asynchronous`）

流水线各级及其延迟：

1. **正运动学 + 解析雅可比** (`rtl/kinematics/`) — ~63 周期。6 路并行 CORDIC sin/cos 预计算(20cyc) → 4 行并行 DH 矩阵链乘(18cyc) → 3 路并行 atan2 欧拉角提取(22cyc) → 6 列并行雅可比(2cyc)。
2. **速度估计** (`rtl/control/velocity_estimator.v`) — ~4 周期。IIR 滤波差分。
3. **阻抗控制** (`rtl/control/impedance_controller.v`) — ~3 周期。F = Ks·(xd−x) + Ds·(ẋd−ẋ)。
4. **J^T 力映射** (`rtl/control/jt_force_mapper.v`) — ~2 周期。τ = J^T · F，6 行并行。
5. **力矩→Iq 转换** (`rtl/foc/torque_to_iq.v`) — ~3 周期。
6. **FOC 阵列** (`rtl/foc/foc_array.v`) — 6 路独立电机通道。Clarke/Park 变换、PI 电流环、SVPWM。

总流水线延迟 ~75 周期 @ 100MHz ≈ 0.75μs → ~1.33MHz 控制带宽。

附加模块：
- `rtl/control/adaptive_impedance.v` — 基于逐关节 RLS 的在线环境刚度估计，自适应调整 Ks/Ds 增益。
- `rtl/common/` — 公共原语：CORDIC sin/cos、CORDIC atan2、定点乘法器、格式转换器。

## 顶层封装

`rtl/top/fpga_top_wrapper.v` 将所有宽信号内化为寄存器总线（AXI-Lite 代理），仅暴露 79 引脚物理 I/O。寄存器地址映射：
- 写寄存器 `0x00`-`0x3E`：ctrl_enable、关节角、期望位姿/速度、Ks/Ds 增益、FOC PI 参数、编码器 phi、ADC 数据
- 读寄存器 `0x40`-`0x5D`：调试位姿、力、力矩、周期计数、状态、PWM 输出
- 未映射地址返回 `0xDEAD_BEEF`

## 仿真

目标仿真器为 Vivado xsim。所有 TCL 脚本使用 `xvlog` → `xelab` → `xsim` 三步流程。

```bash
# 全流水线仿真（含 FOC）
vivado -mode batch -source sim/run_full_pipeline_sim.tcl

# 消融实验（串行/并行 FK + 自适应阻抗）
vivado -mode batch -source sim/run_ablation_sim.tcl
```

仿真工作目录在 `sim_work/` 下按实验分子目录（`full_pipeline/`、`ablation_pipeline/`、`ablation_adaptive/`）。仿真完成后 VCD 波形文件位于对应目录下。

主要 testbench（`sim/testbench/`）：
- `tb_full_pipeline.v` — 全流水线含 FOC 的端到端仿真
- `tb_impedance_control_top.v` — 含刚体动力学模型 (`rigid_body_plant.v`) 的闭环仿真
- `tb_analytical_jacobian.v` — FK/雅可比精度验证，对比 Python 参考向量
- `tb_impedance_step_response.v` — 阶跃响应，输出 CSV 到 `data/csv/`
- `tb_latency_benchmark.v` — 流水线周期计数
- `tb_ablation_pipeline.v` — 串行 vs 并行 FK 消融
- `tb_ablation_adaptive.v` — 自适应阻抗消融（输出 config A/C CSV）

FK/雅可比验证用的测试向量在 `sim/test_vectors/`，由 `scripts/fk_reference_generator.py` 和 `scripts/jacobian_reference.py` 生成。格式为每行一个 Q16.16 十六进制值。

## 综合

```bash
vivado -mode batch -source scripts/synth_report.tcl
```

**注意**：`synth_report.tcl` 中路径硬编码为 `D:/robot/impedance_control/rtl`，在本仓库运行前需将其改为 `D:/papercode/rtl`。报告输出到 `reports/`。

时序约束在 `constraints/timing.xdc`，DSP48 推断策略设为 `full`（乘法器强制映射到 DSP 块以降低功耗）。

## Python 脚本

依赖 numpy、matplotlib、pandas。所有脚本使用 `matplotlib.use('Agg')`（非交互后端）。

论文图表生成：
- `scripts/gen_ch5_figures.py` — 第五章全部实验图，读取 `data/csv/`，输出到 `figures/ch5/`。
- `scripts/gen_ablation_comparison.py` — 消融实验对比图（42BYGH48 电机参数）。
- `scripts/plot_results.py` — 论文主图生成器。支持 `--figure N` 和 `--format pdf`。

```bash
python scripts/gen_ch5_figures.py          # 生成第五章全部图
python scripts/plot_results.py --figure 4  # 生成单张图
python scripts/plot_results.py --format pdf
```

架构/时序图脚本：`plot_fpga_architecture.py`、`plot_hw_dataflow.py`、`plot_system_overview.py`、`plot_timing_diagram.py`。

参考向量生成：
```bash
python scripts/fk_reference_generator.py   # → sim/test_vectors/fk_*.txt
python scripts/jacobian_reference.py       # → sim/test_vectors/jacobian_*.txt
```

其他：`cpu_benchmark.py`（FPGA vs CPU 延迟对比）、`workspace_analysis.py` / `arm_workspace_overlay.py`（工作空间分析）。

## 数据目录

- `data/csv/` — 仿真输出 CSV（阶跃响应、延迟基准、雅可比精度、消融实验 config A/C）
- `data/vcd/` — 波形文件（VCD 格式）
- `figures/` — 生成的图表（PNG/PDF）；`figures/ch5/` 为第五章专用
- `reports/` — Vivado 综合资源利用率报告（逐模块：jacobian、velocity_est、impedance_ctrl、jt_mapper、torque_to_iq）

## 编码约定

- Verilog 模块间使用 `start`/`done` 握手信号驱动流水线。
- 全局使用低有效异步复位 (`rst_n`)。
- 信号命名：`q0..q5` = 关节角，`tau0..tau5` = 关节力矩，`iq0..iq5` = 电机 q 轴电流，`F_x/y/z/rx/ry/rz` = 任务空间力/力矩。
- Q16.16 常量示例：`PI_2 = 32'sd102944`（π/2）。
- TCL 仿真脚本中路径硬编码为 `D:/papercode/`，仿真工作目录在 `sim_work/` 下按实验分子目录。
