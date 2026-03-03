# DroneX 动力学模型 — 定义性参考文档

本文档是仿真子系统的唯一数学标准。所有 MATLAB/Simulink 代码必须与本文档中的坐标系、符号、方程保持严格一致。

对应项目主计划 [PLAN.md](../../docs/PLAN.md) 第二章，以及仿真计划 [SIMULATION_PLAN.md](SIMULATION_PLAN.md)。

---

## 0. 约定总表（Quick Reference）

| 项目 | 约定 |
|------|------|
| 惯性系 {I} | 右手系，$Z_I$ 向上（ENU 类） |
| 机体系 {B} | 右手系，原点在 CG，$X_B$ 前，$Y_B$ 左，$Z_B$ 上 |
| 推力系 {T} | 云台坐标系，零偏转时与 {B} 对齐 |
| 重力 | $\mathbf{g}_I = [0;\ 0;\ -g]^T$，$g = 9.81\ \text{m/s}^2$ |
| 欧拉角序 | ZYX 内旋（yaw $\psi$ → pitch $\theta$ → roll $\varphi$） |
| 四元数 | Hamilton，标量在前 $q = [q_w;\ q_x;\ q_y;\ q_z]$ |
| 旋转矩阵 $R_{IB}$ | 机体系到惯性系：$\mathbf{v}_I = R_{IB}\,\mathbf{v}_B$ |
| 云台角 $\alpha$ | 俯仰，正值使推力偏向 $+X_B$ |
| 云台角 $\beta$ | 滚转，正值使推力偏向 $+Y_B$ |
| 推力点 | 在 CG 下方，$\mathbf{r}_T = [0;\ 0;\ {-l_T}]^T_B$ |
| 状态向量 | $\mathbf{x} = [\mathbf{p}_I;\ \mathbf{v}_I;\ \mathbf{q};\ \boldsymbol{\omega}_B]$，13 维 |
| 输入向量 | $\mathbf{u} = [T;\ \alpha;\ \beta]$，3 维 |

---

## 1. 坐标系定义

### 1.1 惯性系 {I}

| 属性 | 定义 |
|------|------|
| 原点 | 地面固定点 |
| $X_I$ | 水平，前方（任意参考方向） |
| $Y_I$ | 水平，指向左方（$Z_I \times X_I$ 的右手补集） |
| $Z_I$ | 向上 |
| 用途 | 描述位置、速度、重力方向 |

重力向量：

$$\mathbf{g}_I = \begin{bmatrix} 0 \\ 0 \\ -g \end{bmatrix},\quad g = 9.81\ \text{m/s}^2$$

### 1.2 机体系 {B}

| 属性 | 定义 |
|------|------|
| 原点 | 机体重心 (CG) |
| $X_B$ | 前方（机头方向） |
| $Y_B$ | 左方 |
| $Z_B$ | 上方（推力默认方向） |
| 用途 | 描述角速度、力矩、机载力 |

悬停时姿态为单位四元数 $q = [1;0;0;0]$，此时 {B} 与 {I} 重合。

### 1.3 推力/云台系 {T}

| 属性 | 定义 |
|------|------|
| 原点 | 推力作用点 |
| 轴 | 云台零偏转时与 {B} 对齐 |
| 偏转 | 由云台角 $(\alpha, \beta)$ 定义，见 §3.5 |

推力在 {T} 中恒沿 $+Z_T$：$\mathbf{F}^{thrust}_T = [0;\ 0;\ T]^T$。

---

## 2. 旋转约定

### 2.1 欧拉角 — ZYX 内旋

采用 **ZYX 内旋**（intrinsic）顺序，即先绕 $Z$ 旋转 yaw $\psi$，再绕新 $Y'$ 旋转 pitch $\theta$，最后绕新 $X''$ 旋转 roll $\varphi$。

旋转矩阵（机体系到惯性系）：

$$R_{IB}(\varphi, \theta, \psi) = R_z(\psi)\,R_y(\theta)\,R_x(\varphi)$$

其中基本旋转矩阵（右手正方向）：

$$R_x(\varphi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\varphi & -\sin\varphi \\ 0 & \sin\varphi & \cos\varphi \end{bmatrix}$$

$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

$$R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**物理含义**（小角度近似）：

- $\varphi > 0$（roll）：机体向右倾（$Y_B$ 端下降）
- $\theta > 0$（pitch）：机头抬起（$X_B$ 端上升）
- $\psi > 0$（yaw）：机头向左偏

### 2.2 四元数 — Hamilton 标量在前

$$q = \begin{bmatrix} q_w \\ q_x \\ q_y \\ q_z \end{bmatrix}, \quad |q| = 1$$

四元数乘法遵循 Hamilton 约定（$ijk = -1$）。

四元数 $q$ 表示**机体系到惯性系**的旋转。旋转矩阵：

$$R_{IB}(q) = \begin{bmatrix}
1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_w q_z) & 2(q_x q_z + q_w q_y) \\
2(q_x q_y + q_w q_z) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_w q_x) \\
2(q_x q_z - q_w q_y) & 2(q_y q_z + q_w q_x) & 1 - 2(q_x^2 + q_y^2)
\end{bmatrix}$$

向量变换：$\mathbf{v}_I = R_{IB}\,\mathbf{v}_B$。

### 2.3 欧拉角与四元数的转换

**欧拉角 → 四元数** (ZYX)：

$$q = q_z(\psi) \otimes q_y(\theta) \otimes q_x(\varphi)$$

展开：

$$\begin{aligned}
q_w &= c_\varphi c_\theta c_\psi + s_\varphi s_\theta s_\psi \\
q_x &= s_\varphi c_\theta c_\psi - c_\varphi s_\theta s_\psi \\
q_y &= c_\varphi s_\theta c_\psi + s_\varphi c_\theta s_\psi \\
q_z &= c_\varphi c_\theta s_\psi - s_\varphi s_\theta c_\psi
\end{aligned}$$

其中 $c_\varphi = \cos(\varphi/2)$，$s_\varphi = \sin(\varphi/2)$，余同。

**四元数 → 欧拉角**：

$$\begin{aligned}
\varphi &= \text{atan2}\!\big(2(q_w q_x + q_y q_z),\ 1 - 2(q_x^2 + q_y^2)\big) \\
\theta &= \arcsin\!\big(2(q_w q_y - q_z q_x)\big) \\
\psi &= \text{atan2}\!\big(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2)\big)
\end{aligned}$$

### 2.4 欧拉角速率与机体角速度

$$\begin{bmatrix} \dot\varphi \\ \dot\theta \\ \dot\psi \end{bmatrix}
= W(\varphi, \theta)
\begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

$$W = \begin{bmatrix}
1 & \sin\varphi\tan\theta & \cos\varphi\tan\theta \\
0 & \cos\varphi & -\sin\varphi \\
0 & \sin\varphi / \cos\theta & \cos\varphi / \cos\theta
\end{bmatrix}$$

在悬停点 $(\varphi = \theta = 0)$：$W = I_3$，即 $\dot\varphi \approx \omega_x$，$\dot\theta \approx \omega_y$，$\dot\psi \approx \omega_z$。

> **万向锁警告**：$\theta = \pm 90°$ 时 $W$ 奇异。仿真中使用四元数积分避免此问题，欧拉角仅用于显示和小角度线性化。

---

## 3. 非线性动力学方程

### 3.1 状态与输入定义

**状态向量**（13 维）：

$$\mathbf{x} = \begin{bmatrix} \mathbf{p}_I \\ \mathbf{v}_I \\ \mathbf{q} \\ \boldsymbol{\omega}_B \end{bmatrix}
= \begin{bmatrix} p_x \\ p_y \\ p_z \\ v_x \\ v_y \\ v_z \\ q_w \\ q_x \\ q_y \\ q_z \\ \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

| 分量 | 含义 | 坐标系 | 单位 |
|------|------|--------|------|
| $\mathbf{p}_I$ | 位置 | 惯性系 | m |
| $\mathbf{v}_I$ | 速度 | 惯性系 | m/s |
| $\mathbf{q}$ | 姿态四元数 | — | — |
| $\boldsymbol{\omega}_B$ | 角速度 | 机体系 | rad/s |

**输入向量**（3 维）：

$$\mathbf{u} = \begin{bmatrix} T \\ \alpha \\ \beta \end{bmatrix}$$

| 分量 | 含义 | 单位 | 范围 |
|------|------|------|------|
| $T$ | 推力幅值 | N | $[T_{min},\ T_{max}]$ |
| $\alpha$ | 云台俯仰角 | rad | $[-\alpha_{lim},\ \alpha_{lim}]$ |
| $\beta$ | 云台滚转角 | rad | $[-\beta_{lim},\ \beta_{lim}]$ |

### 3.2 平动方程（牛顿第二定律）

$$m\,\dot{\mathbf{v}}_I = R_{IB}\,\mathbf{F}^{thrust}_B + m\,\mathbf{g}_I + \mathbf{F}^{dist}_I$$

$$\dot{\mathbf{p}}_I = \mathbf{v}_I$$

其中 $\mathbf{F}^{thrust}_B$ 为机体系下的推力矢量（见 §3.5），$\mathbf{F}^{dist}_I$ 为惯性系下的扰动力（风等，默认为零）。

### 3.3 转动方程（欧拉方程）

$$I_B\,\dot{\boldsymbol{\omega}}_B + \boldsymbol{\omega}_B \times (I_B\,\boldsymbol{\omega}_B) = \boldsymbol{\tau}^{thrust}_B + \boldsymbol{\tau}^{dist}_B$$

其中：

- $I_B = \text{diag}(I_{xx},\, I_{yy},\, I_{zz})$：绕 CG 的转动惯量（主轴，惯量积为零）
- $\boldsymbol{\tau}^{thrust}_B$：推力产生的力矩（见 §3.5）
- $\boldsymbol{\tau}^{dist}_B$：扰动力矩

展开：

$$\dot{\boldsymbol{\omega}}_B = I_B^{-1}\left(\boldsymbol{\tau}^{thrust}_B - \boldsymbol{\omega}_B \times (I_B\,\boldsymbol{\omega}_B)\right)$$

陀螺力矩展开形式（对角惯量）：

$$\boldsymbol{\omega}_B \times (I_B\,\boldsymbol{\omega}_B) = \begin{bmatrix}
(I_{zz} - I_{yy})\,\omega_y\,\omega_z \\
(I_{xx} - I_{zz})\,\omega_z\,\omega_x \\
(I_{yy} - I_{xx})\,\omega_x\,\omega_y
\end{bmatrix}$$

### 3.4 四元数运动学

$$\dot{\mathbf{q}} = \frac{1}{2}\,\Omega(\boldsymbol{\omega}_B)\,\mathbf{q}$$

其中 $\Omega$ 为 $4 \times 4$ 矩阵：

$$\Omega(\boldsymbol{\omega}) = \begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}$$

**推导**：此式等价于右乘形式 $\dot{q} = \frac{1}{2}\,q \otimes \bar{\omega}$，其中 $\bar{\omega} = [0;\ \omega_x;\ \omega_y;\ \omega_z]$ 为纯四元数。

**验证**：

- 当 $\boldsymbol{\omega} = \mathbf{0}$ 时，$\dot{\mathbf{q}} = \mathbf{0}$（静止无旋转）✓
- $\Omega$ 为反对称矩阵，保证 $\frac{d}{dt}|q|^2 = 2q^T\dot{q} = q^T \Omega q = 0$（范数守恒）✓

> 数值积分中仍需每步归一化 $q \leftarrow q / |q|$ 以抑制浮点误差累积。

### 3.5 推力矢量与力矩模型

#### 3.5.1 云台角定义与推力方向

云台角 $(\alpha, \beta)$ 将推力方向从 $+Z_B$ 偏转到空间方向。定义：

- $\alpha$ **（俯仰）**：推力在 $X_B\text{-}Z_B$ 平面内的偏转角。$\alpha > 0$ 使推力偏向 $+X_B$（前方）。
- $\beta$ **（滚转）**：推力在 $Y_B\text{-}Z_B$ 平面内的偏转角。$\beta > 0$ 使推力偏向 $+Y_B$（左方）。

推力方向单位向量（机体系）：

$$\boxed{\mathbf{d}_B(\alpha, \beta) = \begin{bmatrix} \sin\alpha \\ \sin\beta\,\cos\alpha \\ \cos\beta\,\cos\alpha \end{bmatrix}}$$

**验证**：

| 条件 | $\mathbf{d}_B$ | 物理含义 |
|------|----------|----------|
| $\alpha = \beta = 0$ | $[0;\ 0;\ 1]$ | 推力沿 $+Z_B$（正上方）✓ |
| $\alpha > 0,\ \beta = 0$ | $[\sin\alpha;\ 0;\ \cos\alpha]$ | 推力偏向 $+X_B$（前方）✓ |
| $\alpha = 0,\ \beta > 0$ | $[0;\ \sin\beta;\ \cos\beta]$ | 推力偏向 $+Y_B$（左方）✓ |
| 任意 $(\alpha, \beta)$ | $|\mathbf{d}_B| = 1$ | 单位向量 ✓ |

> **等价旋转矩阵表示**：$\mathbf{d}_B = R_{BT}\,\mathbf{e}_3$，其中 $R_{BT} = R_x(-\beta)\,R_y(\alpha)$，即先绕 $Y$ 轴旋转 $\alpha$，再绕 $X$ 轴旋转 $-\beta$。注意此处 $-\beta$ 是因为约定正 $\beta$ 偏向 $+Y_B$，而右手旋转 $R_x(\beta)$ 使 $+Z$ 偏向 $-Y$。

#### 3.5.2 推力矢量

机体系：

$$\mathbf{F}^{thrust}_B = T\,\mathbf{d}_B = T \begin{bmatrix} \sin\alpha \\ \sin\beta\,\cos\alpha \\ \cos\beta\,\cos\alpha \end{bmatrix}$$

惯性系（代入平动方程 §3.2）：

$$\mathbf{F}^{thrust}_I = R_{IB}\,\mathbf{F}^{thrust}_B$$

#### 3.5.3 推力产生的力矩

推力作用点相对 CG 的位置向量（机体系）：

$$\mathbf{r}_T = \begin{bmatrix} 0 \\ 0 \\ -l_T \end{bmatrix}_B$$

其中 $l_T > 0$ 为推力点到 CG 的距离。$\mathbf{r}_T$ 指向 $-Z_B$（推力点在 CG 下方），这是**倒立摆构型**的核心——重心在推力点之上。

力矩（机体系）：

$$\boldsymbol{\tau}^{thrust}_B = \mathbf{r}_T \times \mathbf{F}^{thrust}_B$$

展开叉乘：

$$\boldsymbol{\tau}^{thrust}_B = \begin{bmatrix} 0 \\ 0 \\ -l_T \end{bmatrix} \times T\begin{bmatrix} \sin\alpha \\ \sin\beta\cos\alpha \\ \cos\beta\cos\alpha \end{bmatrix}$$

$$\boxed{\boldsymbol{\tau}^{thrust}_B = T\,l_T \begin{bmatrix} \sin\beta\,\cos\alpha \\ -\sin\alpha \\ 0 \end{bmatrix}}$$

**验证**（小角度近似）：

$$\boldsymbol{\tau}^{thrust}_B \approx T\,l_T \begin{bmatrix} \beta \\ -\alpha \\ 0 \end{bmatrix}$$

| 输入 | 力矩 | 效果 |
|------|------|------|
| $\alpha > 0$ | $\tau_y < 0$ | 绕 $Y_B$ 负方向转矩 → 机头趋于下压 |
| $\beta > 0$ | $\tau_x > 0$ | 绕 $X_B$ 正方向转矩 → 机体趋于向右倾 |

> **偏航力矩为零**：$\tau_z = 0$，即单纯的云台偏转不产生偏航力矩。偏航控制需通过共轴双桨的转速差实现（本模型未包含，见 §3.7 备注）。

### 3.6 云台动力学（可选，一阶近似）

当需要仿真云台响应延迟时，引入一阶动力学：

$$\dot{\alpha}_{act} = \frac{\alpha_{cmd} - \alpha_{act}}{\tau_g}$$

$$\dot{\beta}_{act} = \frac{\beta_{cmd} - \beta_{act}}{\tau_g}$$

并施加限位：$|\alpha_{act}| \leq \alpha_{lim}$，$|\beta_{act}| \leq \beta_{lim}$。

当前仿真代码未启用此模块（假设云台响应为理想，$\alpha_{act} = \alpha_{cmd}$）。启用时需将状态向量扩展至 15 维。

### 3.7 完整 ODE 汇总

$$\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$$

$$\begin{cases}
\dot{\mathbf{p}}_I = \mathbf{v}_I \\[4pt]
\dot{\mathbf{v}}_I = \dfrac{1}{m}\Big(R_{IB}(q)\,T\,\mathbf{d}_B(\alpha,\beta) + m\,\mathbf{g}_I\Big) \\[4pt]
\dot{\mathbf{q}} = \dfrac{1}{2}\,\Omega(\boldsymbol{\omega}_B)\,\mathbf{q} \\[4pt]
\dot{\boldsymbol{\omega}}_B = I_B^{-1}\Big(T\,l_T\begin{bmatrix}\sin\beta\cos\alpha \\ -\sin\alpha \\ 0\end{bmatrix} - \boldsymbol{\omega}_B \times (I_B\,\boldsymbol{\omega}_B)\Big)
\end{cases}$$

> **关于偏航控制**：当前模型有 3 个输入 $(T, \alpha, \beta)$，无法控制偏航。完整系统需引入第 4 个输入（共轴桨差速产生的偏航力矩 $\tau_{z,cmd}$）。在当前仿真中，偏航通道为开环，初始偏航速率将保持不变。

---

## 4. 参数表

所有参数以 `matlab/scripts/params.m` 为准，下表为当前取值。

### 4.1 刚体参数

| 符号 | 含义 | 单位 | 取值 |
|------|------|------|------|
| $m$ | 质量 | kg | 1.5 |
| $I_{xx}$ | $X_B$ 轴转动惯量 | kg·m² | 0.02 |
| $I_{yy}$ | $Y_B$ 轴转动惯量 | kg·m² | 0.02 |
| $I_{zz}$ | $Z_B$ 轴转动惯量 | kg·m² | 0.03 |
| $g$ | 重力加速度 | m/s² | 9.81 |

### 4.2 几何参数

| 符号 | 含义 | 单位 | 取值 |
|------|------|------|------|
| $l_T$ | 推力点到 CG 距离 | m | 0.15 |
| $\mathbf{r}_T$ | 推力点位置（机体系） | m | $[0;\ 0;\ -0.15]$ |

### 4.3 推进器参数

| 符号 | 含义 | 单位 | 取值 |
|------|------|------|------|
| $T_{hover}$ | 悬停推力 $= mg$ | N | 14.715 |
| $T_{max}$ | 最大推力 | N | $5 \times T_{hover}$ |
| $T_{min}$ | 最小推力 | N | $0.1 \times T_{hover}$ |

### 4.4 云台参数

| 符号 | 含义 | 单位 | 取值 |
|------|------|------|------|
| $\alpha_{lim}$ | 俯仰角限位 | rad | $\pm 25°$ |
| $\beta_{lim}$ | 滚转角限位 | rad | $\pm 25°$ |
| $\tau_g$ | 云台响应时间常数 | s | 0.05 |

---

## 5. 悬停平衡点与线性化

### 5.1 悬停平衡点

令 $\dot{\mathbf{x}} = \mathbf{0}$，求解平衡态：

$$\mathbf{x}_0 = \begin{bmatrix} \mathbf{p}_0 \\ \mathbf{0} \\ [1;0;0;0] \\ \mathbf{0} \end{bmatrix}, \quad \mathbf{u}_0 = \begin{bmatrix} mg \\ 0 \\ 0 \end{bmatrix}$$

物理含义：

- 位置任意（$\mathbf{p}_0$ 为任意悬停点）
- 速度为零
- 姿态为单位四元数（机体系与惯性系对齐）
- 角速度为零
- 推力等于重力，云台居中

**验证**：代入 ODE

- $\dot{\mathbf{p}} = \mathbf{0}$ ✓
- $m\dot{\mathbf{v}} = I \cdot mg \cdot [0;0;1] + m[0;0;-g] = [0;0;0]$ ✓
- $\dot{q} = \frac{1}{2}\Omega(\mathbf{0})q = \mathbf{0}$ ✓
- $\dot{\omega} = I_B^{-1}(mg \cdot l_T \cdot [0;0;0] - \mathbf{0}) = \mathbf{0}$ ✓

### 5.2 小扰动状态定义

在悬停点附近，用**欧拉角**替代四元数（避免四元数约束 $|q|=1$ 带来的冗余），定义 12 维扰动状态：

$$\delta\mathbf{x} = \begin{bmatrix} \delta p_x \\ \delta p_y \\ \delta p_z \\ \delta v_x \\ \delta v_y \\ \delta v_z \\ \delta\varphi \\ \delta\theta \\ \delta\psi \\ \delta\omega_x \\ \delta\omega_y \\ \delta\omega_z \end{bmatrix}, \quad \delta\mathbf{u} = \begin{bmatrix} \delta T \\ \delta\alpha \\ \delta\beta \end{bmatrix}$$

其中 $\delta T = T - mg$，$\delta\alpha = \alpha$，$\delta\beta = \beta$（平衡点处 $\alpha_0 = \beta_0 = 0$）。

### 5.3 线性化推导

在悬停点对 $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ 做一阶 Taylor 展开：$\delta\dot{\mathbf{x}} = A\,\delta\mathbf{x} + B\,\delta\mathbf{u}$。

#### 5.3.1 位置运动学

$$\delta\dot{p}_x = \delta v_x, \quad \delta\dot{p}_y = \delta v_y, \quad \delta\dot{p}_z = \delta v_z$$

#### 5.3.2 平动方程线性化

原式：$m\dot{\mathbf{v}}_I = R_{IB}\,T\,\mathbf{d}_B + m\mathbf{g}_I$

小角度近似：

$$R_{IB} \approx I + [\boldsymbol{\epsilon}\times], \quad \boldsymbol{\epsilon} = \begin{bmatrix}\delta\varphi \\ \delta\theta \\ \delta\psi\end{bmatrix}$$

$$[\boldsymbol{\epsilon}\times] = \begin{bmatrix} 0 & -\delta\psi & \delta\theta \\ \delta\psi & 0 & -\delta\varphi \\ -\delta\theta & \delta\varphi & 0 \end{bmatrix}$$

平衡点处 $T_0\mathbf{d}_0 = mg\,[0;0;1]^T$，分别对姿态和输入求偏导：

**姿态变化的贡献**：

$$[\boldsymbol{\epsilon}\times] \cdot mg\begin{bmatrix}0\\0\\1\end{bmatrix} = mg\begin{bmatrix}\delta\theta \\ -\delta\varphi \\ 0\end{bmatrix}$$

**输入变化的贡献**（$\mathbf{d}_B$ 在平衡点的 Jacobian）：

$$\frac{\partial(T\mathbf{d}_B)}{\partial T}\bigg|_0 = \begin{bmatrix}0\\0\\1\end{bmatrix}, \quad \frac{\partial(T\mathbf{d}_B)}{\partial\alpha}\bigg|_0 = mg\begin{bmatrix}1\\0\\0\end{bmatrix}, \quad \frac{\partial(T\mathbf{d}_B)}{\partial\beta}\bigg|_0 = mg\begin{bmatrix}0\\1\\0\end{bmatrix}$$

合并（重力为常量不参与扰动）：

$$\boxed{\begin{aligned}
\delta\dot{v}_x &= g\,\delta\theta + g\,\delta\alpha \\
\delta\dot{v}_y &= -g\,\delta\varphi + g\,\delta\beta \\
\delta\dot{v}_z &= \frac{1}{m}\,\delta T
\end{aligned}}$$

**物理解读**：

- 机体前倾 ($\delta\theta > 0$，准确说是 pitch 指机头抬起，但此处 $R_{IB}$ 的偏导使 $\delta\theta$ 耦合到 $v_x$ 加速——这是因为 Z-up 惯性系下，机体 Z 轴偏离竖直后推力有水平分量) —— 更准确地说：当 pitch $\delta\theta$ 不为零时，推力有 $X_I$ 分量
- 机体左倾 ($\delta\varphi > 0$) 产生负 $Y_I$ 加速度
- 云台偏转 $\delta\alpha$、$\delta\beta$ 直接产生水平加速度

> **注意**：$\delta\dot{v}_y$ 中 $\delta\varphi$ 和 $\delta\beta$ 的符号反映了一个重要特性——机体倾斜和云台偏转对水平运动的贡献方向。当 $\delta\varphi > 0$（右倾），推力水平分量指向 $-Y_I$；当 $\delta\beta > 0$（推力偏向 $+Y_B$），水平力指向 $+Y_I$。

#### 5.3.3 姿态运动学线性化

在悬停点：$W(\varphi_0, \theta_0) = I_3$，故

$$\delta\dot\varphi = \delta\omega_x, \quad \delta\dot\theta = \delta\omega_y, \quad \delta\dot\psi = \delta\omega_z$$

#### 5.3.4 转动方程线性化

在悬停点 $\boldsymbol{\omega}_0 = \mathbf{0}$，陀螺项 $\omega \times (I\omega) = \mathbf{0}$。

力矩对输入的 Jacobian（由 §3.5.3）：

$$\frac{\partial\boldsymbol{\tau}^{thrust}_B}{\partial\alpha}\bigg|_0 = mg\,l_T\begin{bmatrix}0 \\ -1 \\ 0\end{bmatrix}, \quad \frac{\partial\boldsymbol{\tau}^{thrust}_B}{\partial\beta}\bigg|_0 = mg\,l_T\begin{bmatrix}1 \\ 0 \\ 0\end{bmatrix}, \quad \frac{\partial\boldsymbol{\tau}^{thrust}_B}{\partial T}\bigg|_0 = \begin{bmatrix}0\\0\\0\end{bmatrix}$$

推力幅值变化在 $\alpha_0 = \beta_0 = 0$ 时不改变力矩（力矩方向项为零）。

$$\boxed{\begin{aligned}
\delta\dot\omega_x &= \frac{mg\,l_T}{I_{xx}}\,\delta\beta \\
\delta\dot\omega_y &= -\frac{mg\,l_T}{I_{yy}}\,\delta\alpha \\
\delta\dot\omega_z &= 0
\end{aligned}}$$

### 5.4 状态空间矩阵

$$\delta\dot{\mathbf{x}} = A\,\delta\mathbf{x} + B\,\delta\mathbf{u}$$

状态排列：$\delta\mathbf{x} = [\delta p_x,\, \delta p_y,\, \delta p_z,\, \delta v_x,\, \delta v_y,\, \delta v_z,\, \delta\varphi,\, \delta\theta,\, \delta\psi,\, \delta\omega_x,\, \delta\omega_y,\, \delta\omega_z]^T$

输入排列：$\delta\mathbf{u} = [\delta T,\, \delta\alpha,\, \delta\beta]^T$

$$A = \begin{bmatrix}
\mathbf{0}_3 & I_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{0}_3 & \mathbf{0}_3 & A_{v\phi} & \mathbf{0}_3 \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & I_3 \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3
\end{bmatrix}$$

其中姿态-速度耦合子块：

$$A_{v\phi} = \begin{bmatrix} 0 & g & 0 \\ -g & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$

$$B = \begin{bmatrix}
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
B_{vT} & B_{v\alpha} & B_{v\beta} \\
\mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\
\mathbf{0}_3 & B_{\omega\alpha} & B_{\omega\beta}
\end{bmatrix}$$

展开为标量形式：

$$B = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & g & 0 \\
0 & 0 & g \\
1/m & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & \dfrac{mg\,l_T}{I_{xx}} \\[4pt]
0 & -\dfrac{mg\,l_T}{I_{yy}} & 0 \\[4pt]
0 & 0 & 0
\end{bmatrix}$$

### 5.5 通道解耦

线性化系统**自然解耦**为三个独立子系统：

#### (a) 纵向通道（X-Pitch）

状态 $[\delta p_x,\ \delta v_x,\ \delta\theta,\ \delta\omega_y]$，输入 $\delta\alpha$：

$$A_x = \begin{bmatrix} 0&1&0&0 \\ 0&0&g&0 \\ 0&0&0&1 \\ 0&0&0&0 \end{bmatrix}, \quad B_x = \begin{bmatrix} 0 \\ g \\ 0 \\ -mg\,l_T/I_{yy} \end{bmatrix}$$

#### (b) 横向通道（Y-Roll）

状态 $[\delta p_y,\ \delta v_y,\ \delta\varphi,\ \delta\omega_x]$，输入 $\delta\beta$：

$$A_y = \begin{bmatrix} 0&1&0&0 \\ 0&0&-g&0 \\ 0&0&0&1 \\ 0&0&0&0 \end{bmatrix}, \quad B_y = \begin{bmatrix} 0 \\ g \\ 0 \\ mg\,l_T/I_{xx} \end{bmatrix}$$

#### (c) 高度通道（Z-Altitude）

状态 $[\delta p_z,\ \delta v_z]$，输入 $\delta T$：

$$A_z = \begin{bmatrix} 0&1 \\ 0&0 \end{bmatrix}, \quad B_z = \begin{bmatrix} 0 \\ 1/m \end{bmatrix}$$

#### (d) 偏航通道（Yaw，不可控）

状态 $[\delta\psi,\ \delta\omega_z]$，无输入：

$$A_\psi = \begin{bmatrix} 0&1 \\ 0&0 \end{bmatrix}, \quad B_\psi = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$

偏航通道在当前 3 输入模型下**不可控**，需引入第 4 个输入（偏航力矩）。

---

## 6. 系统分析

### 6.1 开环特征值与稳定性

矩阵 $A$ 的特征值全部为零（$\det(sI - A) = s^{12}$）。

**物理解释**：

对自由飞行刚体，重力不在 CG 产生力矩，因此不存在类似经典固定支点倒立摆的指数不稳定（$e^{at}$ 型发散）。然而，系统是**边际不稳定**的：

1. 任何初始姿态偏差 $\delta\theta_0 \neq 0$ 会导致持续的水平加速度 $\delta\dot{v}_x = g\,\delta\theta_0$，水平速度线性增长
2. 任何初始角速度 $\delta\omega_{y,0} \neq 0$ 会导致姿态线性增长，进而水平加速度二次增长
3. 综合效果：无控制时，位置偏差随时间多项式增长（$t^2$ 到 $t^4$ 阶）

这种"多重积分器链"结构虽非指数不稳定，但在实际中**必须闭环控制**——任何微小扰动都会导致位置无界漂移。

> **与经典倒立摆的区别**：经典倒立摆（如杆铰接于小车上）的 $A$ 矩阵包含 $\sqrt{g/l}$ 的不稳定特征值，因为支点约束力产生了关于铰接点的重力力矩。本系统为自由飞行体，无此约束。

### 6.2 可控性分析

对 10 维可控子系统（排除偏航通道）检查可控性矩阵。

**纵向通道**：

$$\mathcal{C}_x = [B_x,\ A_x B_x,\ A_x^2 B_x,\ A_x^3 B_x]$$

$$= \begin{bmatrix}
0 & g & 0 & -mg^2 l_T/I_{yy} \\
g & 0 & -mg^2 l_T/I_{yy} & 0 \\
0 & -mgl_T/I_{yy} & 0 & 0 \\
-mgl_T/I_{yy} & 0 & 0 & 0
\end{bmatrix}$$

$$\det(\mathcal{C}_x) = \left(\frac{mgl_T}{I_{yy}}\right)^2 \cdot g^2 \cdot \left(\frac{mgl_T}{I_{yy}}\right)^2 \neq 0$$

**秩 = 4**，完全可控 ✓。

**横向通道**：对称结构，同理秩 = 4，完全可控 ✓。

**高度通道**：$\mathcal{C}_z = [B_z,\ A_z B_z] = \begin{bmatrix} 0 & 1/m \\ 1/m & 0 \end{bmatrix}$，秩 = 2 ✓。

**结论**：10 维子系统 $[\delta p_x, \delta v_x, \delta\theta, \delta\omega_y, \delta p_y, \delta v_y, \delta\varphi, \delta\omega_x, \delta p_z, \delta v_z]$ 由 $[\delta T, \delta\alpha, \delta\beta]$ 完全可控。偏航通道 $[\delta\psi, \delta\omega_z]$ 在当前模型下不可控。

### 6.3 数值验证

代入参数值：$m = 1.5$，$g = 9.81$，$l_T = 0.15$，$I_{xx} = I_{yy} = 0.02$，$I_{zz} = 0.03$。

$$\frac{mg\,l_T}{I_{xx}} = \frac{1.5 \times 9.81 \times 0.15}{0.02} = 110.36\ \text{rad/s}^2$$

$$\frac{mg\,l_T}{I_{yy}} = 110.36\ \text{rad/s}^2$$

纵向通道 $B_x = [0;\ 9.81;\ 0;\ -110.36]^T$。

高度通道 $B_z = [0;\ 0.667]^T$。

### 6.4 可观性（简要）

若传感器提供完整状态（IMU + 位置传感器），则系统完全可观。实际飞控中：

- IMU 直接测量 $\boldsymbol{\omega}_B$ 和加速度
- 姿态由 IMU 数据融合估计
- 位置/速度可由气压计（$z$）、光流/GPS（$x, y$）提供

可观性矩阵的详细分析留待状态估计器设计阶段。

---

## 7. 现有代码诊断

基于上述严格推导，逐一检查现有 MATLAB 代码的正确性。

### 7.1 `thrust_model.m` — 推力方向向量

**代码**（第 20-21 行）：

```matlab
% 俯仰 alpha 绕 Y，滚转 beta 绕 X：d_body = R_x(beta)*R_y(alpha)*[0;0;1]
d_body = [sa; sb*ca; cb*ca];
```

**诊断**：

- **公式正确**：`d_body = [sin(α); sin(β)cos(α); cos(β)cos(α)]` 与本文档 §3.5.1 定义一致 ✅
- **注释错误**：注释写 `R_x(beta)*R_y(alpha)`，但正确的旋转矩阵表示为 $R_x(-\beta) \cdot R_y(\alpha)$（注意 $-\beta$）。标准右手旋转 $R_x(\beta)$ 使 $+Z$ 偏向 $-Y$，而我们约定正 $\beta$ 偏向 $+Y$，故实际是 $R_x(-\beta)$。注释需修正。 ⚠️

### 7.2 `rigid_body_ode.m` — 四元数运动学

**代码**（第 29-33 行）：

```matlab
Om = [0,    -omega(1), -omega(2), -omega(3);
      omega(1), 0,       omega(3), -omega(2);
      omega(2), -omega(3), 0,       omega(1);
      omega(3),  omega(2), -omega(1), 0];
q_dot = 0.5 * Om * q;
```

**诊断**：与本文档 §3.4 中 $\Omega(\omega)$ 矩阵**完全一致** ✅。

### 7.3 `quat2rotm.m` — 旋转矩阵

**代码**注释 "body to inertial, v_I = R * v_B"，公式为标准 Hamilton 四元数旋转矩阵。

**诊断**：与本文档 §2.2 **完全一致** ✅。

### 7.4 `euler2quat.m` 与 `quat2euler.m`

**诊断**：两函数实现 ZYX 内旋约定的标准公式，与本文档 §2.3 **一致** ✅。

### 7.5 `attitude_controller.m` — 控制分配符号

**代码**（第 41-48 行）：

```matlab
d = abs(p.r_thrust(3));
T_use = max(T, 1e-6);
alpha = -tau_des(2) / (T_use * d);
beta  =  tau_des(1) / (T_use * d);
```

**诊断**：

由 §3.5.3 的小角度近似：

$$\tau_x \approx T\,l_T\,\beta \implies \beta = \frac{\tau_x}{T\,l_T}$$
$$\tau_y \approx -T\,l_T\,\alpha \implies \alpha = -\frac{\tau_y}{T\,l_T}$$

代码中 `d = abs(r_thrust(3)) = l_T`，所以：
- `alpha = -tau_des(2)/(T*l_T)` ↔ $\alpha = -\tau_y / (T\,l_T)$ ✅
- `beta = tau_des(1)/(T*l_T)` ↔ $\beta = \tau_x / (T\,l_T)$ ✅

控制分配符号**正确** ✅。

### 7.6 `thrust_dir_to_att.m` — 推力方向到期望姿态

**代码**（第 15-16 行）：

```matlab
pitch_des = atan2(dx, dz);   % 推力 +x 需 pitch>0
roll_des  = atan2(-dy, dz);  % 推力 +y 需 roll<0
```

**诊断**：

由 §2.1，机体 $Z_B$（推力默认方向）在惯性系中为：

$$\mathbf{z}_{B,I} = R_{IB}\begin{bmatrix}0\\0\\1\end{bmatrix} = \begin{bmatrix} \sin\theta\cos\varphi \\ -\sin\varphi \\ \cos\theta\cos\varphi \end{bmatrix}$$

若期望推力方向为 $\mathbf{d}_I = [d_x; d_y; d_z]$，则：

$$\theta = \text{atan2}(d_x,\, d_z) \quad (\text{精确需 } d_z = \cos\theta\cos\varphi, \text{ 此处近似 } \varphi \approx 0)$$

$$\varphi = \arcsin(-d_y) \quad (\text{精确值})$$

代码使用 `atan2(-dy, dz)` 而非 $\arcsin(-d_y)$。两者在 $\theta \approx 0$（即 $d_z \approx \cos\varphi$）时近似相等，但当 $\theta$ 较大时有偏差。

- **近悬停使用**：误差可忽略 ✅
- **大角度使用**：存在偏差，建议改用 $\varphi = \arcsin(\text{clamp}(-d_y, -1, 1))$ ⚠️

### 7.7 `position_controller.m` — 推力方向推导

**代码**（第 33 行）：

```matlab
F_des = p.m * (a_des - g_vec);
```

**诊断**：

由 $m\dot{\mathbf{v}}_I = \mathbf{F}^{thrust}_I + m\mathbf{g}_I$ 得 $\mathbf{F}^{thrust}_I = m(\mathbf{a}_{des} - \mathbf{g}_I)$。

代码中 `g_vec = [0; 0; -g]`，所以 `a_des - g_vec = a_des - [0;0;-g] = a_des + [0;0;g]`。

即 $\mathbf{F}_{des} = m(\mathbf{a}_{des} + [0;0;g])$。对于悬停（$a_{des} = 0$），$F_{des} = [0;0;mg]$，推力向上。✅

### 7.8 综合诊断总结

| 文件 | 状态 | 问题 |
|------|------|------|
| `rigid_body_ode.m` | ✅ 正确 | — |
| `thrust_model.m` | ⚠️ 注释错误 | 注释写 `R_x(beta)` 应为 `R_x(-beta)` |
| `quat2rotm.m` | ✅ 正确 | — |
| `euler2quat.m` | ✅ 正确 | — |
| `quat2euler.m` | ✅ 正确 | — |
| `attitude_controller.m` | ✅ 正确 | 控制分配符号一致 |
| `thrust_dir_to_att.m` | ⚠️ 近似 | 大角度下 roll 提取不精确 |
| `position_controller.m` | ✅ 正确 | — |

### 7.9 稳定性问题可能原因

代码本身的方程和符号在小角度范围内是一致且正确的。当前仿真出现发散/不合理响应的可能原因：

1. **PID 参数整定不当**：缺乏系统化设计方法，当前参数为手工试凑
   - 内环带宽：$\omega_{rate} \approx \sqrt{K_p^{rate} \cdot mg\,l_T / I} \approx \sqrt{0.08 \times 110.36} \approx 3.0\ \text{rad/s}$，对应 $\sim 0.5\ \text{Hz}$
   - 对于倒立摆系统，这个带宽可能不足以快速稳定姿态
   
2. **cascaded PID 带宽不匹配**：外环（姿态角 PID）和内环（角速度 PID）的带宽比应至少 3–5 倍，需验证
   
3. **积分项累积**：在大偏差场景下，积分项可能饱和或引起振荡
   
4. **大角度偏差**：`thrust_dir_to_att` 的近似在大角度下偏差，导致外环指令不准确

5. **无云台动力学**：理想云台（无延迟）与实际控制器设计不匹配，可能导致增益选择偏激进

**建议的后续行动**：

1. 基于 §5 的线性化模型设计 LQR 控制器，获得理论最优增益作为 baseline
2. 检查纵向/横向通道的频率响应和闭环带宽
3. 添加云台一阶动力学，评估其对稳定性裕度的影响
4. 从小扰动场景开始验证，逐步增大初始偏差

---

## 附录 A：符号总表

| 符号 | 含义 | 单位 | 所属坐标系 |
|------|------|------|------------|
| $\mathbf{p}_I = [p_x, p_y, p_z]^T$ | 位置 | m | {I} |
| $\mathbf{v}_I = [v_x, v_y, v_z]^T$ | 速度 | m/s | {I} |
| $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$ | 姿态四元数 | — | — |
| $\boldsymbol{\omega}_B = [\omega_x, \omega_y, \omega_z]^T$ | 角速度 | rad/s | {B} |
| $\varphi, \theta, \psi$ | 欧拉角（roll, pitch, yaw） | rad | — |
| $T$ | 推力幅值 | N | 标量 |
| $\alpha$ | 云台俯仰角 | rad | — |
| $\beta$ | 云台滚转角 | rad | — |
| $\mathbf{d}_B$ | 推力方向单位向量 | — | {B} |
| $\mathbf{r}_T$ | 推力点相对 CG 位置 | m | {B} |
| $l_T$ | 推力点到 CG 距离 | m | 标量 |
| $m$ | 机体质量 | kg | — |
| $I_B = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ | 转动惯量 | kg·m² | {B} |
| $R_{IB}$ | 机体到惯性旋转矩阵 | — | — |
| $\tau_g$ | 云台时间常数 | s | — |
| $A, B$ | 线性化状态空间矩阵 | — | — |
