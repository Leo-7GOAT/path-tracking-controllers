# Path Tracking Controllers Benchmark

Comparative benchmark and visualization project for classical vehicle path-tracking controllers in Python.

这是一个面向自动驾驶规控/控制算法岗位展示的轨迹跟踪 benchmark 项目。当前版本已经整理为可独立运行、可复现实验结果、可实时演示的工程化版本，统一实现并对比了 5 类经典路径跟踪方法：

- `Stanley`
- `Pure Pursuit`
- `PID`
- `LQR`
- `MPC-style`

项目使用统一的运动学自行车模型、统一的仿真时钟、统一的终点判定和统一的评测指标，对不同控制器在多种轨迹任务上的跟踪精度、完成时间和控制平滑性进行量化对比。

## Highlights

- 统一 benchmark 框架：同一套车辆模型、终点逻辑和指标统计，便于公平对比。
- 支持 5 种控制器：`Stanley / Pure Pursuit / PID / LQR / MPC-style`。
- 支持 3 类任务：`Straight`、高曲率 `Curved`、整圈闭环 `CircleLoop`。
- 综合脚本默认实时弹窗演示，并让多辆车在同一张图里一起跑。
- 支持每个控制器独立脚本运行，也支持综合 benchmark 一键出图出表。
- 每次运行自动覆盖旧结果，避免历史输出干扰当前展示。
- 依赖简洁，仅需 `numpy + matplotlib`。

## Visualization

### Multi-controller Curved Demo

![Curved multi-controller snapshot](assets/readme/curved_all_controllers.png)

### Trajectory Overview

![Trajectory overview](assets/readme/trajectory_overview.png)

### Metric Dashboard

![Metric dashboard](assets/readme/metric_dashboard.png)

## Controllers

- `Stanley`：基于航向误差和横向误差的几何跟踪方法。
- `Pure Pursuit`：基于前视点的经典几何路径跟踪方法。
- `PID`：使用速度 PID 和横向误差 PID 的基线方法。
- `LQR`：基于线性二次型调节器的状态反馈控制方法。
- `MPC-style`：有限时域预测控制风格的控制器，用于兼顾精度与平滑性。

## Scenarios

- `Straight`：直线路径，主要验证基础速度控制与稳态跟踪能力。
- `Curved`：高曲率、多转折连续曲线，主要验证横向误差抑制与控制稳定性。
- `CircleLoop`：完整一圈闭环任务，主要验证持续曲率场景下的鲁棒性与路径保持能力。

## Quick Start

### 1. Install

```bash
pip install -r requirements.txt
```

### 2. Run the full benchmark

```bash
python Compare_controller.py
```

默认行为：

- 实时弹出综合演示窗口
- 同一张图里显示多个控制器一起跑
- 覆盖旧结果并重新生成图表

如果你只想静默生成结果，不弹窗：

```bash
python Compare_controller.py --no-live
```

如果你还想导出 GIF：

```bash
python Compare_controller.py --animate
```

## Run Individual Controllers

每个控制器都可以独立运行：

```bash
python Stanley.py
python PurePursuit.py
python PID.py
python LQR.py
python MPC.py
```

如果只生成结果，不弹实时窗口：

```bash
python Stanley.py --no-live
python PurePursuit.py --no-live
python PID.py --no-live
python LQR.py --no-live
python MPC.py --no-live
```

## Current Benchmark Snapshot

当前本地基准结果如下：

| Scenario | Controller | Success | Avg Lat Err (m) | Max Lat Err (m) | RMSE (m) | Completion Time (s) | Mean \|Speed Err\| (m/s) | Mean \|Steer\| (rad) | Mean \|Steer Rate\| (rad/s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Straight | Stanley | Yes | 0.000 | 0.000 | 0.000 | 12.70 | 0.985 | 0.000 | 0.000 |
| Straight | PurePursuit | Yes | 0.000 | 0.000 | 0.000 | 12.60 | 0.981 | 0.000 | 0.000 |
| Straight | PID | Yes | 0.178 | 0.649 | 0.280 | 12.70 | 0.987 | 0.257 | 5.155 |
| Straight | LQR | Yes | 0.000 | 0.000 | 0.000 | 12.60 | 0.985 | 0.000 | 0.000 |
| Straight | MPC | Yes | 0.000 | 0.000 | 0.000 | 12.60 | 0.985 | 0.000 | 0.000 |
| Curved | Stanley | Yes | 0.221 | 2.771 | 0.547 | 25.00 | 0.491 | 0.137 | 0.175 |
| Curved | PurePursuit | Yes | 0.282 | 3.881 | 0.776 | 25.60 | 0.471 | 0.136 | 0.169 |
| Curved | PID | Yes | 0.407 | 3.562 | 0.726 | 25.30 | 0.484 | 0.479 | 8.149 |
| Curved | LQR | Yes | 0.346 | 4.113 | 0.824 | 25.80 | 0.467 | 0.431 | 7.329 |
| Curved | MPC | Yes | 0.260 | 3.886 | 0.765 | 25.60 | 0.470 | 0.137 | 0.616 |
| CircleLoop | Stanley | Yes | 0.118 | 0.148 | 0.121 | 23.00 | 0.269 | 0.161 | 0.214 |
| CircleLoop | PurePursuit | Yes | 0.039 | 0.080 | 0.042 | 23.20 | 0.267 | 0.149 | 0.081 |
| CircleLoop | PID | Yes | 0.055 | 0.227 | 0.069 | 23.10 | 0.270 | 0.463 | 8.693 |
| CircleLoop | LQR | Yes | 0.045 | 0.081 | 0.048 | 23.10 | 0.271 | 0.296 | 5.286 |
| CircleLoop | MPC | Yes | 0.047 | 0.066 | 0.049 | 23.10 | 0.270 | 0.148 | 1.474 |

## Experiment Analysis

### Overall

- 5 种控制器在 3 个任务上都能成功完成跟踪，说明当前统一 benchmark 框架和终点减速逻辑已经稳定可用。
- `Straight` 任务区分度较低，更适合作为基础正确性验证。
- `Curved` 是当前最有挑战性的场景，能明显拉开不同方法在精度、控制平滑性和鲁棒性上的差异。
- `CircleLoop` 反映了持续曲率场景下的闭环稳定能力，对前视型和预测型方法更有区分意义。

### By Scenario

- `Straight`：`Stanley / Pure Pursuit / LQR / MPC` 基本实现零误差跟踪，说明基础模型、路径索引和控制接口工作正常；`PID` 可以完成任务，但横向控制仍存在一定抖动。
- `Curved`：`Stanley` 在当前参数下 RMSE 最低，说明其在高曲率几何路径跟踪上的表现较强；`MPC-style` 的平均转向变化率明显更低，说明它在复杂场景下更容易兼顾精度和控制平滑性。
- `CircleLoop`：`Pure Pursuit` 的 RMSE 最低，说明前视策略在闭环连续曲率任务里非常有效；`MPC-style` 和 `LQR` 也保持了较低误差，表现稳定。

### By Controller

- `Stanley`：在几何路径跟踪任务中整体表现稳定，尤其适合曲率变化明显的路径。
- `Pure Pursuit`：实现简单、效果直观，在闭环和连续曲率任务里很有代表性。
- `PID`：适合作为 baseline，但在复杂场景下更容易出现较大的转向振荡。
- `LQR`：具备较强的经典控制解释性，适合做状态反馈方法对照。
- `MPC-style`：虽然 RMSE 不一定始终最低，但通常能在精度和控制平滑性之间取得较好平衡。

### Engineering Notes

- 当前终点减速逻辑使用统一速度收敛和柔性制动策略，避免了早期版本中“接近终点时速度来回震荡”的问题。
- 综合脚本与单控制器脚本都支持覆盖旧结果重新生成，便于反复调参和稳定展示。

## File Guide

| File | Purpose |
| --- | --- |
| `Compare_controller.py` | 综合 benchmark 入口，统一调度所有控制器并输出总结果。 |
| `benchmark_runner.py` | 仿真主流程、场景构造、评测逻辑、作图与动画导出核心模块。 |
| `Stanley.py` | `Stanley` 控制器实现与独立运行入口。 |
| `PurePursuit.py` | `Pure Pursuit` 控制器实现与独立运行入口。 |
| `PID.py` | 纯 PID 基线控制器实现与独立运行入口。 |
| `LQR.py` | LQR 路径跟踪控制器实现与独立运行入口。 |
| `MPC.py` | MPC-style 控制器实现与独立运行入口。 |
| `common.py` | 公共数据结构、车辆模型、PID 工具、仿真配置定义。 |
| `cubic_spline.py` | 样条路径生成模块，用于构造参考轨迹。 |
| `draw.py` | 小车可视化绘制工具，用于静态图和动画展示。 |
| `requirements.txt` | 项目依赖说明。 |
| `resume_project_summary.md` | 面向简历投递的项目描述素材。 |
| `assets/readme/` | README 首页展示图片，适合直接同步到 GitHub。 |
| `outputs/` | benchmark 运行输出目录，存放表格、分析图和动画。 |

## Generated Outputs

综合 benchmark 运行后会生成：

- `outputs/benchmark_results.csv`
- `outputs/benchmark_results.md`
- `outputs/trajectory_overview.png`
- `outputs/metric_dashboard.png`
- `outputs/runs/`
- `outputs/comparison_effects/snapshots/`

如果启用 `--animate`，还会生成：

- `outputs/animations/`
- `outputs/comparison_effects/animations/`

独立脚本运行后会在对应目录下生成结果，例如：

- `outputs/stanley/`
- `outputs/purepursuit/`
- `outputs/pid/`
- `outputs/lqr/`
- `outputs/mpc/`

## Notes

- `outputs/` 是运行产物，每次 benchmark 会自动覆盖旧结果。
- 如果在 PyCharm 中运行但没有弹出独立窗口，通常是因为开启了 SciView；关闭 `Show plots in tool window` 后再运行即可。
