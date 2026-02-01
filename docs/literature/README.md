# 文献摘编目录

本目录按技术主题对 [REFERENCES.md](../REFERENCES.md) 中的文献做要点摘编，并与 [PLAN.md](../PLAN.md) 的工程描述对应，便于建模、控制与仿真时查阅。

| 文档 | 主题 | 与 DroneX 的关联 |
|------|------|------------------|
| [01-TVC-and-rocket-landing.md](01-TVC-and-rocket-landing.md) | 推力矢量控制与火箭垂直着陆 | 二轴云台 TVC、推力线不通过重心、力矩与力分配 |
| [02-inverted-pendulum-and-underactuated.md](02-inverted-pendulum-and-underactuated.md) | 倒立摆与欠驱动系统 | 重心高于推力点、开环不稳定、串级控制与 LQR |
| [03-coaxial-rotor-and-propulsion.md](03-coaxial-rotor-and-propulsion.md) | 共轴旋翼与推进器建模 | 共轴反桨、推力–转速关系、干扰系数 |
| [04-flight-control-and-estimation.md](04-flight-control-and-estimation.md) | 飞行控制与状态估计 | 姿态/位置 PID 或 LQR、控制分配、IMU 互补滤波或 EKF |
| [05-simulation-and-dynamics.md](05-simulation-and-dynamics.md) | 仿真与动力学建模 | 坐标系、平动/转动方程、Simulink/CIL、云台与推进器模型 |

**使用建议**：先阅 [REFERENCES.md](../REFERENCES.md) 获取完整书目与分类，再按需打开上表对应摘编文档查看公式与实现要点。
