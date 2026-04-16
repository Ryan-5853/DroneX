from __future__ import annotations

from sim_py.config import DynamicsConfig
from sim_py.data_types import ActorOutput, POSTruth
from sim_py.math_utils import add_vec3, scale_vec3


class DynamicsModule:
    """根据执行器实际输出推进真实物理状态。"""

    def __init__(self, config: DynamicsConfig, initial_state: POSTruth) -> None:
        self.config = config
        self.state = initial_state

    def step(self, actor_output: ActorOutput) -> POSTruth:
        dt = self.config.dt
        acc_xyz = scale_vec3(actor_output.force_xyz, 1.0 / self.config.mass)
        vel_xyz = add_vec3(self.state.vel_xyz, scale_vec3(acc_xyz, dt))
        disp_xyz = add_vec3(self.state.disp_xyz, scale_vec3(vel_xyz, dt))
        angv_xyz = add_vec3(self.state.angv_xyz, scale_vec3(actor_output.torque_xyz, dt))

        self.state = POSTruth(
            disp_xyz=disp_xyz,
            att_quat=self.state.att_quat,
            vel_xyz=vel_xyz,
            acc_xyz=acc_xyz,
            angv_xyz=angv_xyz,
            anga_xyz=actor_output.torque_xyz,
        )
        return self.state
