/**
 * @file Attitude.h
 * @brief 姿态解算应用层：IMU 校准请求、姿态处理等。
 */

#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#ifdef __cplusplus
extern "C" {
#endif

/** IMU 选择：1=IMU1, 2=IMU2, 0=两个同时 */
typedef enum {
    ATTITUDE_IMU_1 = 1,
    ATTITUDE_IMU_2 = 2,
    ATTITUDE_IMU_BOTH = 0,
} Attitude_ImuSel_t;

/**
 * @brief 请求 IMU 校准（由 imu_cali 指令调用）。
 * @param imu     1=IMU1, 2=IMU2, 0=两个同时
 * @param time_s  校准时间（秒）
 * @param thresh  失败阈值（如方差阈值等，具体含义由实现定）
 * @return 0 成功，-1 参数无效
 */
int Attitude_RequestImuCali(int imu, float time_s, float thresh);

/**
 * @brief 姿态解算任务：应在定时器回调中周期调用（如 100Hz）。
 *        当前为占位实现，检测到校准请求时输出调试信息。
 */
void Attitude_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __ATTITUDE_H__ */
