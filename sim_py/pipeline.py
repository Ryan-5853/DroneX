from __future__ import annotations

from dataclasses import dataclass

from sim_py.Actuator import ActuatorModule
from sim_py.Allocator import AllocatorModule
from sim_py.Controller import ControllerModule
from sim_py.Dynamics import DynamicsModule
from sim_py.Estimator import EstimatorModule
from sim_py.Sensors import SensorModule
from sim_py.config import SimulationConfig
from sim_py.data_types import ActorOutput, CtrlCmd, CtrlOutput, POSEst, POSTruth, SensorObs, SensorTruth


@dataclass(slots=True)
class SimulationSnapshot:
    pos_truth: POSTruth
    sensor_truth: SensorTruth
    sensor_obs: SensorObs
    pos_est: POSEst
    ctrl_output: CtrlOutput
    ctrl_cmd: CtrlCmd
    actor_output: ActorOutput


class SimulationSystem:
    """仿真系统主装配器。"""

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config or SimulationConfig()
        self.sensor = SensorModule(self.config.sensor)
        self.estimator = EstimatorModule(self.config.estimator)
        self.controller = ControllerModule(self.config.controller)
        self.allocator = AllocatorModule(self.config.allocator)
        self.actuator = ActuatorModule(self.config.actuator)
        self.dynamics = DynamicsModule(self.config.dynamics, self.config.initial_truth)

    @property
    def state(self) -> POSTruth:
        return self.dynamics.state

    def step(self) -> SimulationSnapshot:
        pos_truth = self.state
        sensor_truth, sensor_obs = self.sensor.observe(pos_truth)
        pos_est = self.estimator.estimate(sensor_obs)
        ctrl_output = self.controller.update(pos_est, self.config.target)
        ctrl_cmd = self.allocator.allocate(ctrl_output)
        actor_output = self.actuator.apply(ctrl_cmd)
        new_truth = self.dynamics.step(actor_output)
        return SimulationSnapshot(
            pos_truth=new_truth,
            sensor_truth=sensor_truth,
            sensor_obs=sensor_obs,
            pos_est=pos_est,
            ctrl_output=ctrl_output,
            ctrl_cmd=ctrl_cmd,
            actor_output=actor_output,
        )
