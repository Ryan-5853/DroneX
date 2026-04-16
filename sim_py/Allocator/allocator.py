from __future__ import annotations

from sim_py.config import AllocatorConfig
from sim_py.data_types import CtrlCmd, CtrlOutput
from sim_py.math_utils import clamp


class AllocatorModule:
    """将推力/力矩目标映射为具体执行器控制指令。"""

    def __init__(self, config: AllocatorConfig) -> None:
        self.config = config

    def allocate(self, ctrl_output: CtrlOutput) -> CtrlCmd:
        thrust_x, thrust_y, thrust_z = ctrl_output.thrust_xyz
        torque_x, _, torque_z = ctrl_output.torque_xyz
        return CtrlCmd(
            servo_x=clamp(thrust_x + torque_x, -self.config.servo_limit, self.config.servo_limit),
            servo_y=clamp(thrust_y, -self.config.servo_limit, self.config.servo_limit),
            motor_p=clamp(thrust_z + torque_z, 0.0, self.config.motor_limit),
            motor_n=clamp(thrust_z - torque_z, 0.0, self.config.motor_limit),
        )
