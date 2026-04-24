/**
 * @file Att_Ctrl_PID.h
 * @brief 倒立摆飞行器两轴姿态位置式 PID：roll -> X 舵机，pitch -> Y 舵机。
 *        两轴完全解耦，不在控制器内做舵机差动混控。
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
 * @brief 初始化参数：两轴姿态 PID + 舵机归一化输出限幅。
 *
 * 控制约定：
 * - roll 误差只进入 X 轴舵机输出 servo_norm[0]。
 * - pitch 误差只进入 Y 轴舵机输出 servo_norm[1]。
 * - 机构/舵机安装方向差异放在驱动或最终输出层处理，不通过 PID 参数隐式修正。
 */
typedef struct {
    PID_Gains_t gains_roll_angle;
    PID_Gains_t gains_pitch_angle;
    PID_Gains_t gains_roll_rate;
    PID_Gains_t gains_pitch_rate;

    float roll_angle_out_min;
    float roll_angle_out_max;
    float pitch_angle_out_min;
    float pitch_angle_out_max;

    uint8_t enable_roll_angle_i_limit;
    float roll_angle_i_min;
    float roll_angle_i_max;
    uint8_t enable_pitch_angle_i_limit;
    float pitch_angle_i_min;
    float pitch_angle_i_max;

    float roll_rate_out_min;
    float roll_rate_out_max;
    float pitch_rate_out_min;
    float pitch_rate_out_max;

    uint8_t enable_roll_rate_i_limit;
    float roll_rate_i_min;
    float roll_rate_i_max;
    uint8_t enable_pitch_rate_i_limit;
    float pitch_rate_i_min;
    float pitch_rate_i_max;

    float gyro_lpf_tau_s;

    float servo_min_norm;
    float servo_max_norm;

    float roll_servo_gain;
    float pitch_servo_gain;
    float servo_trim_norm[2];
} Att_Ctrl_PID_Config_t;

/** 单周期输入：三轴姿态与期望、单调时间戳 [µs] */
typedef struct {
    float roll_rad;
    float pitch_rad;
    float roll_rate_fb_rad_s;
    float pitch_rate_fb_rad_s;
    float roll_sp_rad;
    float pitch_sp_rad;
    uint32_t time_us;
} Att_Ctrl_PID_In_t;

/** 单周期输出：舵机归一化命令 [-1, 1]，0=X轴，1=Y轴 */
typedef struct {
    float servo_norm[2];
} Att_Ctrl_PID_Out_t;

/**
 * @brief 初始化：保存配置，创建 roll/pitch 两个独立 PID，应用限幅。
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
