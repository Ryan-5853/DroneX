from __future__ import annotations

from sim_py.config import ControllerConfig
from sim_py.data_types import CtrlOutput, POSEst, SimulationTarget
from sim_py.math_utils import scale_vec3, sub_vec3


class ControllerModule:
    """将估计位姿与目标位姿比较，生成推力与力矩目标。"""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config

    def update(self, pos_est: POSEst, target: SimulationTarget) -> CtrlOutput:
        pos_error = sub_vec3(target.disp_xyz, pos_est.disp_xyz)
        angv_error = sub_vec3(target.angv_xyz, pos_est.angv_xyz)
        return CtrlOutput(
            thrust_xyz=scale_vec3(pos_error, self.config.position_kp),
            torque_xyz=scale_vec3(angv_error, self.config.attitude_kp),
        )
