/**
 * @file Attitude_Est.h
 * @brief 姿态估计算法层：双正交 IMU 融合 + 姿态解算对外接口。
 */

#ifndef __ATTITUDE_EST_H__
#define __ATTITUDE_EST_H__

#include "Driver/IMU/IMU_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * 初始化与更新
 * ---------------------------------------------------------------------------- */

/**
 * @brief 初始化姿态估计算法（偏置清零、四元数单位化等）。
 */
void Attitude_Est_Init(void);

/**
 * @brief 姿态解算主入口：融合双 IMU 数据并更新姿态。
 * @param imu1  IMU1 原始数据（其坐标系由 R_imu1_to_body 定义）
 * @param imu2  IMU2 原始数据（其坐标系由 R_imu2_to_body 定义），可为 NULL 表示仅用 IMU1
 * @param dt_s  解算周期 [s]，如 0.01 表示 100Hz
 */
void Attitude_Est_Update(const IMU_Data_t *imu1, const IMU_Data_t *imu2, float dt_s);

/* ----------------------------------------------------------------------------
 * 姿态输出
 * ---------------------------------------------------------------------------- */

/**
 * @brief 获取当前姿态四元数 [qw, qx, qy, qz]。
 * @param q 输出，至少 4 个 float
 */
void Attitude_Est_GetQuat(float q[4]);

/**
 * @brief 获取欧拉角 [rad]，已按系统标准定义完成后处理（可直接使用）。
 * @param roll_rad  滚转角 [rad]，绕 Y 轴
 * @param pitch_rad 俯仰角 [rad]，绕 X 轴
 * @param yaw_rad   偏航角 [rad]，绕 Z 轴
 */
void Attitude_Est_GetEuler(float *roll_rad, float *pitch_rad, float *yaw_rad);

/**
 * @brief 获取主板系角速度 [rad/s]，[wx, wy, wz] 对应 X/Y/Z 轴。
 * @param omega 输出 [wx, wy, wz]
 */
void Attitude_Est_GetAngularRate(float omega[3]);

/**
 * @brief 获取融合后的主板系加速度 [m/s²]，[ax, ay, az] 对应 X/Y/Z 轴。
 * @param acc 输出 [ax, ay, az]
 */
void Attitude_Est_GetAccel(float acc[3]);

/**
 * @brief 获取单个 IMU 经过偏置修正和参考系变换后的六轴数据。
 * @param imu_id ATTITUDE_EST_IMU_1 或 ATTITUDE_EST_IMU_2
 * @param data   输出主板系数据：gyro [rad/s]，accel [m/s^2]
 * @return 0 成功，负数表示参数无效或该 IMU 尚无有效数据
 */
int Attitude_Est_GetImuBodyData(int imu_id, IMU_Data_t *data);

/* ----------------------------------------------------------------------------
 * 校准与偏置
 * ---------------------------------------------------------------------------- */

/** IMU 标识：1=IMU1, 2=IMU2 */
#define ATTITUDE_EST_IMU_1  1
#define ATTITUDE_EST_IMU_2 2

/**
 * @brief 设置指定 IMU 的偏置（校准完成后由应用层调用）。
 * @param imu_id ATTITUDE_EST_IMU_1 或 ATTITUDE_EST_IMU_2
 * @param bias   偏置值，NULL 表示清零
 * @return 0 成功，-1 参数无效
 */
int Attitude_Est_SetBias(int imu_id, const IMU_Bias_t *bias);

/**
 * @brief 获取当前使用的偏置（供调试或保存）。
 * @param imu_id ATTITUDE_EST_IMU_1 或 ATTITUDE_EST_IMU_2
 * @param bias   输出
 * @return 0 成功，-1 参数无效
 */
int Attitude_Est_GetBias(int imu_id, IMU_Bias_t *bias);

/* ----------------------------------------------------------------------------
 * 状态与使能（可选扩展）
 * ---------------------------------------------------------------------------- */

/** 姿态估计状态 */
typedef enum {
    ATTITUDE_EST_OK = 0,
    ATTITUDE_EST_ERR_PARAM = -1,
    ATTITUDE_EST_ERR_NOT_READY = -2,
} Attitude_Est_Status_t;

/**
 * @brief 是否已至少完成一次有效解算。
 */
int Attitude_Est_IsReady(void);

/**
 * @brief 重置姿态（如切换模式时归零）。
 */
void Attitude_Est_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __ATTITUDE_EST_H__ */
