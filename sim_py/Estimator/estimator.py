from __future__ import annotations

from sim_py.config import EstimatorConfig
from sim_py.data_types import POSEst, SensorObs


class EstimatorModule:
    """根据传感器观测反解机体位姿。"""

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config

    def estimate(self, sensor_obs: SensorObs) -> POSEst:
        # 当前骨架仅透传惯导可直接观测到的量，完整姿态/位置估计算法后续补充。
        return POSEst(
            angv_xyz=sensor_obs.angv_xyz,
            acc_xyz=sensor_obs.acc_xyz,
        )
