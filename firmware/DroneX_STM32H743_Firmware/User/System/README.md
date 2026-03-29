# System 层状态机规范（草案）

本文档定义 `User/System` 的宏观状态机接口语义，当前阶段以“接口与注释先行”为目标，复杂保护逻辑保留 TODO。

## 1. 状态定义

- `STANDBY`（待命）  
  动力单元未激活，允许进行解锁条件检查。

- `READY`（就绪）  
  动力单元已激活，但不输出动力。

- `ERROR`（错误）  
  上电自检失败进入，禁止再次激活动力单元，需重启恢复。

- `RC`（遥控）  
  手动模式，遥控通道可直接映射油门/云台。

- `RUN`（运行）  
  自主决策模式（飞行策略待实现）。

- `PROTECT`（保护）  
  急停、遥测异常或硬件异常触发后锁定。只能通过 CLI 重启命令或断电重启恢复。

## 2. 关键迁移规则

### 上电

- 上电后执行自检：
  - 自检通过 -> `STANDBY`
  - 自检失败 -> `ERROR`（锁定）

### 待命到就绪

- `STANDBY -> READY` 需同时满足：
  - `arm_condition_ok = true`（遥控器通道组合满足预设）
  - `cmd_to_ready = true`

### 激活态切换

- `READY <-> RC <-> RUN` 允许通过命令互相切换：
  - `cmd_to_rc`
  - `cmd_to_run`
  - `cmd_to_ready`
- 任一激活态可通过 `cmd_to_standby` 回到 `STANDBY`。

### 保护策略

- 任意状态收到以下事件之一，进入 `PROTECT`：
  - `estop`
  - `telemetry_abnormal`
  - `hardware_abnormal`
- `PROTECT` 为锁定态：不允许再次激活动力单元。

## 3. 恢复策略

- `ERROR`、`PROTECT` 均视为“需重启恢复”状态。
- 当前接口中通过 `cli_reboot_request` 触发状态机复位（等价于软件重启入口占位）。
- 实际产品建议最终走统一重启流程（命令行重启或硬件复位）。

## 4. 模块职责

- `System_StateMachine`  
  只负责状态迁移，不做硬件读写。

- `System_Safety`  
  汇总故障位图（当前已接 RC 丢失、ESC 更新异常，其他异常待实现）。

- `System_Telemetry`  
  打包系统状态快照，用于串口/地面站遥测。

- `System_Debug`  
  输出状态变化日志，便于联调。

- `System_Manager`  
  统一调度输入采样、安全判定、状态机更新和输出模块。

## 5. 当前占位与后续 TODO

- 自检输入当前由 `System_Manager` 占位为“通过”，需接入真实自检结果。
- 遥控器通道映射（解锁、模式切换）当前为占位逻辑，需按机型协议落地。
- `telemetry_abnormal`、`hardware_abnormal`、`estop` 仍需对接真实检测源。
- `RUN` 状态下自主飞行控制尚未实现。

