/**
 * @file Att_Ctrl_PID.h
 * @brief 垂起（VTOL）姿态角 PID：滚转/俯仰 → 双舵机；偏航 → 双电机差速油门。
 *        角度 [rad] 与 Attitude_Est_GetEuler 一致（yaw 绕 Z）。
 */

#ifndef __ATT_CTRL_PID_H__
#define __ATT_CTRL_PID_H__

#include "Algorithm/Common/PID/PID.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATT_CTRL_PID_OK           0
#define ATT_CTRL_PID_ERR_NULL    -1
#define ATT_CTRL_PID_ERR_LIMIT   -2
#define ATT_CTRL_PID_ERR_TIME    -3
#define ATT_CTRL_PID_ERR_NOT_INIT -4

/**
 * @brief 初始化参数：三轴姿态 PID + 混控与饱和。
 *
 * 垂起混控约定：
 * - 偏航 u_y：电机1 += yaw_throttle_diff * u_y，电机2 -= 同上（航向由左右推力差产生）。
 * - 滚转 u_r、俯仰 u_p：舵机1 = trim0 + pitch_servo_common*u_p + roll_servo_diff*u_r，
 *   舵机2 = trim1 + pitch_servo_common*u_p - roll_servo_diff*u_r。
 * - pitch_throttle_common：两电机同时叠加（例如推力辅助俯仰，不需要则置 0）。
 */
typedef struct {
    PID_Gains_t gains_roll;
    PID_Gains_t gains_pitch;
    PID_Gains_t gains_yaw;

    float roll_out_min;
    float roll_out_max;
    float pitch_out_min;
    float pitch_out_max;
    float yaw_out_min;
    float yaw_out_max;

    uint8_t enable_roll_i_limit;
    float roll_i_min;
    float roll_i_max;
    uint8_t enable_pitch_i_limit;
    float pitch_i_min;
    float pitch_i_max;
    uint8_t enable_yaw_i_limit;
    float yaw_i_min;
    float yaw_i_max;

    float throttle_min;
    float throttle_max;
    float servo_min_rad;
    float servo_max_rad;

    float base_throttle;
    float yaw_throttle_diff;
    float pitch_throttle_common;
    float pitch_servo_common;
    float roll_servo_diff;
    float servo_trim_rad[2];
} Att_Ctrl_PID_Config_t;

/** 单周期输入：三轴姿态与期望、单调时间戳 [µs] */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float roll_sp_rad;
    float pitch_sp_rad;
    float yaw_sp_rad;
    uint32_t time_us;
} Att_Ctrl_PID_In_t;

/** 单周期输出：电机无量纲油门、舵机角 [rad] */
typedef struct {
    float throttle_motor[2];
    float servo_rad[2];
} Att_Ctrl_PID_Out_t;

/**
 * @brief 初始化：保存配置，创建滚转/俯仰/偏航角 PID，应用限幅。
 */
int Att_Ctrl_PID_Init(const Att_Ctrl_PID_Config_t *cfg);

/**
 * @brief 复位积分与微分状态（增益与配置不变）。
 */
void Att_Ctrl_PID_Reset(void);

/**
 * @brief 姿态控制更新。
 * @return ATT_CTRL_PID_OK；时间戳非法时返回 ATT_CTRL_PID_ERR_TIME 且不写 *out
 */
int Att_Ctrl_PID_Update(const Att_Ctrl_PID_In_t *in, Att_Ctrl_PID_Out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __ATT_CTRL_PID_H__ */
