# 推力矢量控制与火箭垂直着陆 — 文献摘编

与 DroneX 的**二轴云台推力矢量**、**重心高于推力点**的力学特性直接相关。

---

## 1. 核心概念对应

| DroneX 设计 | 文献中的对应概念 |
|-------------|------------------|
| 云台偏转 (θ_pitch, θ_roll) | Thrust vectoring / gimbal deflection |
| 推力线不通过重心 → 姿态力矩 | Thrust vector moment / TVC authority |
| 垂直着陆段减速与姿态稳定 | Powered descent / terminal landing phase |

---

## 2. 推荐文献要点

### 2.1 单轴推力矢量 UAV（Ebrahimi 等）

- **题目**：Modeling and Control of a Single-Axis Thrust-Vectoring UAV  
- **与项目关系**：单轴 TVC 可视为本项目俯仰或滚转一维简化，建模思路（推力幅值 + 偏转角 → 力/力矩）可直接推广到二轴。
- **可借鉴**：  
  - 推力矢量产生的力矩表达式：τ = r × F_thrust，其中 r 为推力作用点到重心的矢量。  
  - 控制分配：期望力矩 → 推力大小 T 与云台角 α（或 β）。

### 2.2 火箭/着陆器制导与控制（Blackmore 等）

- **题目**：Guidance and Control for Lunar Descent and Landing  
- **与项目关系**：月球着陆的“动力下降段”同样面临：大推力、短时燃烧、姿态与轨迹耦合、终端约束（垂直速度、位置）。
- **可借鉴**：  
  - 制导律（如多项式轨迹、ZEM/ZEV）给出期望加速度/推力方向。  
  - 姿态控制器跟踪该推力方向，与 DroneX 中“位置环输出期望推力方向 + 姿态环跟踪”一致。

### 2.3 飞行动力学中的 TVC（Tewari, Atmospheric and Space Flight Dynamics）

- **与项目关系**：系统讲述推力矢量在体轴系下的分量、对转动方程的影响，以及欧拉角/四元数姿态方程。
- **常用公式**：  
  - 推力在机体系下：F_b = T · [sin(α)cos(β), sin(β), cos(α)cos(β)]^T（随云台角定义略作调整）。  
  - 力矩：τ = r_cp × F_b，r_cp 为推力作用点相对重心在体轴系下的坐标。

---

## 3. 与 PLAN.md 的衔接

- **平动**：`m·a = F_thrust + m·g + F_disturbance`，其中 `F_thrust` 由 T 与云台角 (θ_pitch, θ_roll) 确定，见上文 F_b。  
- **转动**：`I·ω̇ + ω×(I·ω) = τ_thrust + τ_disturbance`，τ_thrust 由 r_cp × F_thrust 得到。  
- **控制分配**：期望力 F_d、期望力矩 τ_d → 求解 (T, θ_pitch, θ_roll)，需考虑推力限幅与云台机械限位。

---

## 4. 延伸阅读

- Slegers, N., Kyle, J., "Nonlinear Model Predictive Control for a Thrust-Vectored Flying Wing" — 推力矢量 + NMPC，若后续做约束与最优轨迹可参考。  
- SpaceX 公开的 Falcon 9 着陆视频与解说：直观理解大推力、短时、高精度姿态与位置协同。
