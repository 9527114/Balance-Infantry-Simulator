# Balance Infantry Simulator

基于 Webots 和 MATLAB 的双轮自平衡小车仿真项目。项目围绕轮式倒立摆动态平衡、腿轮复合结构运动控制和复杂地形通过能力展开，完成了 Webots 仿真环境搭建、控制器编写、MATLAB 建模分析以及控制参数调试。

![Webots 仿真截图](worlds/sim3_cyy.png)

## 项目简介

本项目搭建了一个双轮自平衡小车的 Webots 仿真系统。小车通过 IMU、陀螺仪、轮速反馈和关节位置反馈获取自身运动状态，并利用轮毂电机力矩控制与腿部关节角度控制实现动态平衡。

在 MATLAB 中建立双轮平衡小车状态空间模型，进行系统可控性分析、稳定性分析和 LQR 参数计算；在 Webots 控制器中读取传感器数据并输出轮毂电机力矩和腿部关节目标角度，用于验证小车在平地、上坡、上楼梯、飞坡和落地等场景下的平衡性能。

## 技术栈

- Webots
- MATLAB / Live Script
- Python
- NumPy
- PID 控制
- LQR 控制
- 状态空间建模
- IMU / Gyro 传感器仿真
- 轮毂电机力矩控制
- 腿部关节位置控制

## 项目结构

```text
.
|-- controllers/
|   |-- my_controller/
|   |   `-- my_controller.py          # 主控制器：传感器读取、姿态反馈、轮毂电机和腿部关节控制
|   |-- simplified_controller/
|   |   `-- simplified_controller.py  # 简化控制器
|   `-- naiive_controller/
|       `-- naiive_controller.py      # 基础控制器
|-- LQR_control/
|   |-- two_wheel_model.m             # 双轮平衡小车状态空间建模与 LQR 参数计算
|   |-- LQR_SJ1.mlx                   # MATLAB Live Script 控制参数调试文件
|   |-- LQR_SJ1_EarlierEdition.mlx    # 早期版本调试文件
|   |-- generate_cpp_formatMatrix.py  # 控制矩阵格式转换脚本
|   |-- const_set.mat                 # MATLAB 参数数据
|   |-- data*.xlsx                    # 测试/调试数据
|   `-- modify.md                     # 模型参数说明与推导记录
`-- worlds/
    |-- sim3_cyy.wbt                  # Webots 仿真世界
    `-- sim3_cyy.png                  # 仿真场景截图
```

## 功能特点

- 搭建 Webots 双轮自平衡小车仿真环境。
- 配置轮毂电机、腿部关节电机、IMU、陀螺仪、摄像头和显示模块。
- 基于键盘输入实现前进、后退、转向和姿态模式切换。
- 基于 MATLAB 建立轮式倒立摆状态空间模型。
- 进行系统可控性判断和特征值稳定性分析。
- 通过 PID/LQR 参数调试提升小车平衡性能。
- 支持平地稳定、上坡、上楼梯、飞坡和落地等复杂地形测试。
- 在 Webots display 中实时绘制轮毂力矩、pitch 角和关节力矩反馈。

## 控制器说明

主控制器位于：

```text
controllers/my_controller/my_controller.py
```

控制器主要完成以下任务：

1. 初始化 Webots 机器人实例和仿真步长。
2. 获取腿部关节电机、轮毂电机、IMU、陀螺仪、摄像头和显示模块。
3. 读取键盘输入，实现运动方向和模式切换。
4. 根据腿部关节角度和车体 pitch 计算腿部等效长度与姿态角。
5. 根据控制矩阵和状态反馈计算轮毂电机输出力矩。
6. 输出腿部关节目标角度，实现小车姿态调整。
7. 通过 display 模块可视化轮毂力矩、车体 pitch 和关节力矩变化。

## MATLAB 建模说明

MATLAB 建模文件位于：

```text
LQR_control/two_wheel_model.m
```

该文件完成了双轮平衡小车的核心建模流程：

- 定义车轮质量、机体质量、轮半径、摆长、转动惯量、重力加速度等物理参数。
- 构建状态空间矩阵 `A` 和输入矩阵 `B`。
- 使用 `ctrb(A, B)` 判断系统可控性。
- 计算系统特征值，分析开环稳定性。
- 设计 LQR 控制器权重矩阵 `Q` 和 `R`。
- 使用 `lqr(A, B, Q, R)` 计算反馈增益矩阵。

## 运行方式

### 1. 打开 Webots 仿真世界

使用 Webots 打开：

```text
worlds/sim3_cyy.wbt
```

### 2. 设置控制器

在 Webots 中将机器人控制器设置为：

```text
my_controller
```

### 3. 运行仿真

点击 Webots 的运行按钮，观察小车在仿真环境中的平衡状态和运动效果。

### 4. MATLAB 参数调试

在 MATLAB 中打开并运行：

```text
LQR_control/two_wheel_model.m
```

或使用 Live Script 文件：

```text
LQR_control/LQR_SJ1.mlx
```

根据仿真反馈调整控制参数，并同步到 Webots 控制器中验证效果。

## 操作说明

在 Webots 仿真窗口中可通过键盘控制小车运动：

| 按键 | 功能 |
| --- | --- |
| `Ctrl + W` | 前进 |
| `Ctrl + S` | 后退 |
| `Ctrl + A` | 左转 |
| `Ctrl + D` | 右转 |
| `Ctrl + X` | 切换站立/放松模式 |

## 项目成果

- 完成双轮自平衡小车 Webots 仿真环境搭建。
- 实现基于 Python 的 Webots 主控制器。
- 完成 MATLAB 中的状态空间建模、可控性分析和 LQR 参数计算。
- 通过反复调试 PID/LQR 控制参数，使小车能够保持稳定平衡。
- 实现小车在平地、坡道、台阶、飞坡和落地过程中的连续运动测试。
- 最终达到小车稳定平衡并完成复杂地形完整行程的效果。

## 项目亮点

- 将 MATLAB 控制参数设计与 Webots 物理仿真闭环结合，形成建模、调参、验证的完整流程。
- 同时考虑轮毂电机力矩控制和腿部关节角度控制，适用于腿轮复合式平衡机器人仿真。
- 使用 IMU、陀螺仪、轮速和关节反馈构建多传感器状态反馈。
- 面向复杂地形进行测试，验证控制策略在动态冲击和姿态扰动下的鲁棒性。

## 后续优化方向

- 将 PID/LQR 参数整理为独立配置文件，便于快速切换不同控制策略。
- 增加数据记录与曲线绘制模块，自动保存 pitch、yaw、轮速和电机力矩变化。
- 对复杂地形测试结果进行量化分析，例如最大 pitch 偏差、恢复时间和通过成功率。
- 进一步优化腿部虚拟模型控制，提高落地缓冲和台阶通过稳定性。

