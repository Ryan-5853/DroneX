# DroneX 飞控固件子系统

基于 HAL 库的飞控固件与外围驱动，面向 DroneX 推力矢量可回收火箭式无人机。

## 简介

本子系统基于 **STM32H743** 与 **STM32CubeMX/HAL**，从零编写飞控固件，实现：

- 实时控制循环（200–500 Hz）
- IMU 姿态估计（互补滤波或 EKF）
- 电调与云台电机驱动
- 遥控接收与串口遥测
- 安全逻辑（低电压、失控、急停）

## 目录结构

```
firmware/
├── docs/                              # 飞控设计文档
│   └── FLIGHT_CONTROL_DESIGN.md       # 飞控设计主文档
├── DroneX_STM32H743_Firmware/         # CubeMX + Keil 工程（主工程）
│   ├── Core/                          # 主控逻辑、HAL 配置
│   ├── Drivers/                       # HAL、CMSIS
│   ├── MDK-ARM/                       # Keil 工程文件
│   └── *.ioc                           # CubeMX 配置
├── hal_project/                       # 空白 HAL 工程导入位置（可选）
└── README.md                          # 本文件
```

- **DroneX_STM32H743_Firmware/**：STM32CubeMX 生成的 HAL 工程，含 Keil MDK-ARM 项目文件
- **docs/**：飞控设计、驱动规划、控制流程等文档

## 硬件平台

| 组件       | 选型                     |
| ---------- | ------------------------ |
| 主控       | STM32H743                |
| IMU        | MPU6050 或 BMI088        |
| 电调       | BLHeli 或 ESC32          |
| 云台舵机   | 高端无刷舵机，PWM / SBUS 等舵机协议 |

## 开发阶段

对应项目主计划 [PLAN.md](../docs/PLAN.md) 中：

- **P2：下位机基础** — 飞控框架、IMU、电调、云台驱动，可手动操控的固件
- **P3：控制闭环** — 姿态闭环、悬停、简单轨迹

## 相关文档

- [飞控设计文档](docs/FLIGHT_CONTROL_DESIGN.md) — 软件架构、控制流程、驱动规划、安全逻辑
- [项目主计划](../docs/PLAN.md) — DroneX 总体规划、系统架构
- [动力学模型](../sim/docs/dynamics_model.md) — 坐标系、运动方程
- [飞行控制文献](../docs/literature/04-flight-control-and-estimation.md) — 姿态控制、状态估计
- [机械设计计划](../mechanical/docs/DESIGN_PLAN.md) — 云台舵机选型、限位
