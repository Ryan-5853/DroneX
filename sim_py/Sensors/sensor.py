from __future__ import annotations

import random

from sim_py.config import SensorConfig
from sim_py.data_types import POSTruth, SensorObs, SensorTruth


class SensorModule:
    """将真实状态转换为传感器参考系读数，并叠加观测噪声。"""

    def __init__(self, config: SensorConfig) -> None:
        self.config = config

    def compute_truth(self, pos_truth: POSTruth) -> SensorTruth:
        # 这里先保留最小骨架，后续接入安装误差与坐标变换模型。
        return SensorTruth(
            angv_xyz=pos_truth.angv_xyz,
            acc_xyz=pos_truth.acc_xyz,
        )

    def observe(self, pos_truth: POSTruth) -> tuple[SensorTruth, SensorObs]:
        sensor_truth = self.compute_truth(pos_truth)
        sensor_obs = SensorObs(
            angv_xyz=self._add_noise(sensor_truth.angv_xyz, self.config.noise_std_gyro),
            acc_xyz=self._add_noise(sensor_truth.acc_xyz, self.config.noise_std_acc),
        )
        return sensor_truth, sensor_obs

    @staticmethod
    def _add_noise(value: tuple[float, float, float], std: float) -> tuple[float, float, float]:
        if std <= 0.0:
            return value
        return tuple(component + random.gauss(0.0, std) for component in value)
