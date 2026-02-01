# DroneX 项目参考文献与文献索引

本文档按技术主题整理与 DroneX（推力矢量可回收无人机）相关的学术论文、书籍、标准与在线资源，便于建模、控制、仿真与实现时查阅。

---

## 一、推力矢量控制与火箭垂直着陆

| 类型 | 文献 | 说明 |
|------|------|------|
| 论文 | Blackmore, L., *et al.*, "Guidance and Control for Lunar Descent and Landing," *AIAA Guidance, Navigation and Control Conference*, 2008 | 月球着陆段制导与控制，与 TVC 着陆段问题相近 |
| 论文 | Ebrahimi, B., *et al.*, "Modeling and Control of a Single-Axis Thrust-Vectoring UAV," *Journal of Guidance, Control, and Dynamics*, 2018 | 单轴推力矢量 UAV 建模与控制，可直接类比本项目的俯仰/滚转 TVC |
| 论文 | Slegers, N., Kyle, J., "Nonlinear Model Predictive Control for a Thrust-Vectored Flying Wing," *AIAA Guidance, Navigation and Control Conference*, 2009 | 推力矢量飞行器非线性模型预测控制 |
| 技术报告 | SpaceX, Falcon 9 Landing Technology (公开资料与发布会) | 垂直着陆姿态与 TVC 工程实现参考 |
| 书籍 | Tewari, A., *Atmospheric and Space Flight Dynamics*, Birkhäuser, 2007 | 大气/空间飞行动力学，含推力矢量与姿态动力学 |

---

## 二、倒立摆与欠驱动系统控制

| 类型 | 文献 | 说明 |
|------|------|------|
| 书籍 | Åström, K.J., Murray, R.M., *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed., Princeton, 2021 | 反馈系统与稳定性，倒立摆经典例题 |
| 论文 | Fantoni, I., Lozano, R., *Non-linear Control for Underactuated Mechanical Systems*, Springer, 2002 | 欠驱动机械系统非线性控制，含摆类系统 |
| 论文 | Spong, M.W., "The Swing Up Control Problem for the Acrobot," *IEEE Control Systems Magazine*, 1995 | 欠驱动摆的起摆与稳定，控制思路可借鉴 |
| 论文 | Bouabdallah, S., *et al.*, "Design and Control of an Indoor Micro Quadrotor," *ICRA*, 2004 | 微四旋翼姿态控制，与重心在推力之上的稳定性问题相关 |
| 标准/教程 | MathWorks, "Inverted Pendulum" (Control System Toolbox / Simulink 示例) | 倒立摆建模与 LQR/PID 仿真参考 |

---

## 三、共轴旋翼与推进器建模

| 类型 | 文献 | 说明 |
|------|------|------|
| 论文 | Leishman, J.G., "Principles of Helicopter Aerodynamics," 2nd ed., Cambridge, 2006 | 旋翼气动基础，共轴双桨干扰与拉力/扭矩 |
| 论文 | Coleman, C.P., "A Survey of Theoretical and Experimental Coaxial Rotor Aerodynamic Research," NASA TM 3675, 1997 | 共轴旋翼理论与实验综述 |
| 论文 | Kim, S.K., *et al.*, "Design and Implementation of a Coaxial Quadrotor with Tiltable Rotors," *ICUAS*, 2016 | 共轴 + 倾转机构，推力矢量与旋翼布局 |
| 论文 | Prouty, R.W., *Helicopter Performance, Stability, and Control*, Krieger, 1995 | 直升机性能与稳定性，旋翼推力/扭矩建模 |
| 在线 | UIUC Airfoil Data, NACA/APC 桨叶数据 | 螺旋桨升力/阻力系数与雷诺数修正 |

---

## 四、飞行控制与状态估计

| 类型 | 文献 | 说明 |
|------|------|------|
| 书籍 | Stevens, B.L., *et al.*, *Aircraft Control and Simulation*, 3rd ed., Wiley, 2016 | 飞行器控制与仿真，欧拉角/四元数、PID 与 LQR |
| 书籍 | Tewari, A., *Advanced Control of Aircraft, Spacecraft and Rockets*, Wiley, 2011 | 飞行器/航天器先进控制，含推力矢量与姿态回路 |
| 论文 | Mahony, R., *et al.*, "Nonlinear Complementary Filters on the Special Orthogonal Group," *IEEE TAC*, 2008 | 姿态互补滤波在 SO(3) 上的严格形式，适合 IMU 姿态估计 |
| 论文 | Simon, D., *Optimal State Estimation*, Wiley, 2006 | 最优状态估计，EKF 与姿态四元数 EKF |
| 论文 | Bresciani, T., "Modelling, Identification and Control of a Quadrotor Helicopter," *Lund University*, 2008 | 四旋翼建模、辨识与控制，流程可迁移到 TVC 无人机 |

---

## 五、仿真与动力学建模

| 类型 | 文献 | 说明 |
|------|------|------|
| 书籍 | Zipfel, P.H., *Modeling and Simulation of Aerospace Vehicle Dynamics*, 3rd ed., AIAA, 2007 | 飞行器动力学建模与仿真，坐标系与欧拉方程 |
| 书籍 | Jazar, R.N., *Advanced Dynamics*, Wiley, 2020 | 刚体动力学、多体系统与欧拉角奇异点 |
| 文档 | MathWorks, *Aerospace Blockset* / *Simscape Multibody* 用户手册 | 六自由度刚体、推进器与多体机械仿真 |
| 论文 | Luukkonen, T., "Modelling and Control of Quadcopter," *Aalto University*, 2011 | 四旋翼 Simulink 建模与控制器在环仿真 |
| 标准 | IEEE/ISO 坐标系与姿态表示 (例如 NED/ENU) | 惯性系与机体系定义，与 PLAN.md 中约定一致 |

---

## 六、嵌入式飞控与硬件实现

| 类型 | 文献 | 说明 |
|------|------|------|
| 手册 | STM32H743 Reference Manual & CubeMX/HAL 文档 | 主控、定时器、ADC、通信与实时循环 |
| 论文 | Premerlani, W., Bizard, P., "Direction Cosine Matrix IMU: Theory," 2009 (开源笔记) | DCM/互补滤波在嵌入式上的实现 |
| 文档 | Betaflight / PX4 架构与状态机说明 (GitHub/Wiki) | 飞控任务调度、安全逻辑与参数配置思路 |
| 手册 | BLHeli_S / ESC32 电调协议与 PWM/OneShot | 电机转速与推力指令接口 |
| 书籍 | Valvano, J.W., *Embedded Systems*, 4th ed., 2019 | 嵌入式实时系统与外设驱动 |

---

## 七、垂直起降与降落逻辑

| 类型 | 文献 | 说明 |
|------|------|------|
| 论文 | Polack, P., *et al.*, "The MIT Duckietown Autonomous Driving Project," (可类比地面触地与安全逻辑) | 自主系统的状态机与安全约束 |
| 论文 | 各类 VTOL 与 eVTOL 降落轨迹规划文献 | 减速段、接地检测与缓冲策略 |
| 标准 | DO-178C / DO-331 (机载软件) | 高可靠性飞控软件开发参考（若需合规） |

---

## 使用建议

- **建模与仿真**：优先参阅 *Zipfel*、*Stevens*、*Tewari (Atmospheric)* 与 MATLAB Aerospace/Simulink 文档。  
- **控制算法**：从 *Åström*、*Stevens*、*Tewari (Advanced)* 与 *Ebrahimi (TVC UAV)* 入手，再结合倒立摆与欠驱动文献。  
- **共轴推进**：*Leishman*、*Coleman* 与 *Prouty* 用于推力/扭矩与干扰系数建模。  
- **嵌入式实现**：STM32 HAL、Betaflight/PX4 架构与 *Premerlani* 的 DCM/互补滤波实现。  

如需某主题的详细摘编或公式推导，可参见 `docs/literature/` 下对应主题的文献摘编文档。
