from __future__ import annotations

import random

from sim_py.config import ActuatorConfig
from sim_py.data_types import ActorOutput, CtrlCmd


class ActuatorModule:
    """根据控制指令生成带执行器特性的实际输出。"""

    def __init__(self, config: ActuatorConfig) -> None:
        self.config = config

    def apply(self, ctrl_cmd: CtrlCmd) -> ActorOutput:
        # 当前骨架使用简单映射，便于后续插入滞后、限幅和随机行为。
        force_xyz = (
            ctrl_cmd.servo_x,
            ctrl_cmd.servo_y,
            ctrl_cmd.motor_p + ctrl_cmd.motor_n,
        )
        torque_xyz = (
            ctrl_cmd.servo_x,
            -ctrl_cmd.servo_y,
            ctrl_cmd.motor_p - ctrl_cmd.motor_n,
        )
        return ActorOutput(
            force_xyz=self._add_noise(force_xyz, self.config.noise_std_force),
            torque_xyz=self._add_noise(torque_xyz, self.config.noise_std_torque),
        )

    @staticmethod
    def _add_noise(value: tuple[float, float, float], std: float) -> tuple[float, float, float]:
        if std <= 0.0:
            return value
        return tuple(component + random.gauss(0.0, std) for component in value)
