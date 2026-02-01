# DroneX

推力矢量可回收火箭式无人机 —— 重心高于推力点，通过共轴反桨与二轴云台实现姿态控制，动力学特性类似 SpaceX Falcon 9 垂直着陆。

## 项目概述

DroneX 采用**单一推力源 + 推力矢量偏转**的设计：

| 特性       | 描述                      |
| ---------- | ------------------------- |
| 推进方式   | 共轴反桨，向下产生推力，自然抵消反扭矩 |
| 重心位置   | 高于推力作用点（倒立摆模型） |
| 控制机构   | 二轴云台（俯仰+滚转），无刷减速电机驱动 |
| 飞行模式   | 悬停、平移、垂直降落（含自由落体段） |

与常规多旋翼不同，本机重心在推力之上，固有不稳定，需闭环控制保持姿态。

## 目录结构

```
DroneX/
├── docs/              # 项目文档
│   ├── PLAN.md        # 总体规划（动力学、架构、开发阶段）
│   ├── REFERENCES.md  # 参考文献索引
│   └── literature/    # 文献摘编
├── sim/               # 仿真（MATLAB/Simulink）
│   ├── matlab/        # 动力学模型、控制器、脚本、工具函数
│   ├── simulink/      # Simulink 模型
│   ├── data/          # 仿真输出
│   └── docs/          # 仿真计划、动力学模型、验证标准
├── mechanical/        # 机械设计
│   ├── docs/          # 设计计划、机架/推进/云台说明
│   ├── cad/           # CAD 源文件
│   ├── models/        # STL/STEP 导出
│   ├── drawings/      # 工程图
│   └── bom/           # 物料清单
└── firmware/          # 飞控固件（STM32H743 + HAL）
    ├── DroneX_STM32H743_Firmware/   # CubeMX + Keil 工程（主工程）
    ├── docs/          # 飞控设计文档
    └── hal_project/   # 空白 HAL 工程导入位置
```

## 快速入门

### 仿真

1. 安装 MATLAB R2021b+ 与 Simulink
2. 进入仿真目录并运行：
   ```matlab
   cd sim/matlab
   init_path
   run_basic_sim
   ```
   或直接运行：`cd sim/matlab/scripts; run_basic_sim`
3. 悬停验证：`run_verify_hover`（定点悬停 / 水平平移）
4. 详见 [sim/README.md](sim/README.md)

### 文档入口

- [总体规划](docs/PLAN.md) — 动力学模型、系统架构、子系统规划、开发阶段
- [参考文献](docs/REFERENCES.md) — TVC、倒立摆、共轴旋翼、飞行控制等文献
- [仿真计划](sim/docs/SIMULATION_PLAN.md) — 仿真目标、模块、验证标准
- [动力学模型](sim/docs/dynamics_model.md) — 坐标系、方程、参数（与 params.m 对齐）
- [机械设计计划](mechanical/docs/DESIGN_PLAN.md) — 管架、共轴产品、减速电机、材料与尺寸
- [飞控设计](firmware/docs/FLIGHT_CONTROL_DESIGN.md) — 软件架构、控制流程、驱动规划

## 技术栈

| 模块     | 工具 / 平台            |
| -------- | ---------------------- |
| 仿真     | MATLAB、Simulink               |
| 飞控     | STM32H743、CubeMX/HAL、Keil MDK-ARM |
| 机械     | 管架为主、3D 打印、现成共轴产品、减速电机直驱 |
| 文档     | Markdown               |

## 开发阶段

| 阶段 | 内容                     |
| ---- | ------------------------ |
| P0   | 建模与仿真               |
| P1   | 机械初版                 |
| P2   | 下位机基础               |
| P3   | 控制闭环、悬停           |
| P4   | 降落逻辑、鲁棒性优化     |

详见 [docs/PLAN.md](docs/PLAN.md)。

## 许可证

（待定）
