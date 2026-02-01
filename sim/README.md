# DroneX 仿真子系统

在实机开发前验证动力学模型和控制算法的仿真环境。

## 简介

本子系统面向重心高于推力点的可回收火箭式无人机，通过 MATLAB/Simulink 实现：

- 刚体动力学仿真
- 推进器与云台模型
- 控制器在环仿真 (CIL)
- 控制参数与轨迹规划验证

## 目录结构

```
sim/
├── docs/                    # 仿真相关文档
│   ├── SIMULATION_PLAN.md   # 仿真计划（主文档）
│   ├── dynamics_model.md    # 动力学模型说明
│   └── verification.md      # 验证标准与用例
├── matlab/                  # MATLAB 脚本与函数
│   ├── init_path.m          # 路径初始化
│   ├── models/              # 动力学/推进器模型（rigid_body_ode、thrust_model）
│   ├── controllers/         # 姿态/位置控制器
│   ├── utils/               # 四元数、thrust_dir_to_att、animate_drone
│   └── scripts/             # params、run_basic_sim、run_verify_hover
├── simulink/                # Simulink 模型
│   ├── models/              # .slx 模型文件
│   └── lib/                 # 可复用库模块
├── data/                    # 仿真数据与结果
│   └── results/             # 运行输出、日志
└── README.md                # 本文件
```

## 环境要求

- **MATLAB**：R2021b 或更高版本
- **Simulink**：必选
- **Simscape Multibody**：可选，用于机械/接触仿真

## 快速入门

1. 打开 MATLAB，切换到项目目录
2. 运行基础仿真（自动添加路径）：
   ```matlab
   cd sim/matlab/scripts
   run_basic_sim
   ```
   或先初始化路径再运行：
   ```matlab
   cd sim/matlab
   init_path
   run_basic_sim
   ```
3. 将显示：时域曲线、3D 轨迹、姿态动画
4. 悬停验证：`run_verify_hover` 运行定点悬停 / 水平平移验证用例
5. 仿真结果可保存至 `data/results/`（在脚本中取消注释保存代码）

## 相关文档

- [仿真计划](docs/SIMULATION_PLAN.md) — 仿真目标、模块规划、流程与验证
- [动力学模型](docs/dynamics_model.md) — 坐标系、方程与参数
- [验证标准](docs/verification.md) — 验证指标与用例
- [项目主计划](../docs/PLAN.md) — DroneX 总体规划
