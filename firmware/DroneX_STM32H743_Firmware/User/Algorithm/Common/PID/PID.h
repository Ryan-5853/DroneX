/**
 * @file PID.h
 * @brief 通用位置式 PID：实例结构体 + 初始化、误差与时间戳更新、重置、参数读写（输出经指针返回）。
 */

#ifndef __ALGORITHM_COMMON_PID_H__
#define __ALGORITHM_COMMON_PID_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 返回值：成功 */
#define PID_OK              0
/** 空指针 */
#define PID_ERR_NULL       -1
/** 时间戳非单调递增（或 dt 无效） */
#define PID_ERR_TIME       -2
/** 限幅参数非法（如 min > max） */
#define PID_ERR_LIMIT      -3

/** PID 比例/积分/微分增益 */
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Gains_t;

/**
 * @brief PID 对象（可静态分配；内部状态勿直接改写，经 API 访问）。
 */
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;     /**< ∫e·dt */
    float prev_error;
    uint32_t prev_time_us;
    uint8_t has_time;    /**< 是否已有上一拍时间戳 */

    float out_min;
    float out_max;
    uint8_t output_limited;

    float i_min;
    float i_max;
    uint8_t integral_limited;
} PID_t;

/**
 * @brief 实例化/初始化：清零状态并设置增益。
 * @param pid   对象指针
 * @param gains 增益，不可为 NULL
 * @return PID_OK / PID_ERR_NULL
 */
int PID_Init(PID_t *pid, const PID_Gains_t *gains);

/**
 * @brief 重置内部状态（积分、上一拍误差与时间戳），增益与限幅保持不变。
 */
void PID_Reset(PID_t *pid);

/**
 * @brief 一拍更新：u = Kp*e + Ki*∫e·dt + Kd*de/dt。
 * @param pid      对象指针
 * @param error    当前误差 e（由应用定义：设定值−测量值等）
 * @param time_us  单调递增时间戳 [µs]，与上一拍差值作为采样周期
 * @param output   输出 u 写入地址，不可为 NULL
 * @return PID_OK；首次调用仅建立时间基准，输出为 Kp*e；失败时 *output 不写
 */
int PID_Update(PID_t *pid, float error, uint32_t time_us, float *output);

/**
 * @brief 修改增益。
 */
int PID_SetGains(PID_t *pid, const PID_Gains_t *gains);

/**
 * @brief 读取当前增益。
 */
int PID_GetGains(const PID_t *pid, PID_Gains_t *gains);

/**
 * @brief 设置输出限幅；若 min_out 或 max_out 为 NULL 则关闭输出限幅。
 * @return PID_OK / PID_ERR_NULL / PID_ERR_LIMIT
 */
int PID_SetOutputLimits(PID_t *pid, const float *min_out, const float *max_out);

/**
 * @brief 查询输出限幅是否使能及当前上下限（经指针写出）。
 * @param enabled 非 NULL 时写入：0 关闭，非 0 开启
 */
int PID_GetOutputLimits(const PID_t *pid, float *min_out, float *max_out, int *enabled);

/**
 * @brief 设置积分状态限幅（对 ∫e·dt）；任一为 NULL 则关闭积分限幅。
 */
int PID_SetIntegralLimits(PID_t *pid, const float *min_i, const float *max_i);

/**
 * @brief 查询积分限幅。
 */
int PID_GetIntegralLimits(const PID_t *pid, float *min_i, float *max_i, int *enabled);

#ifdef __cplusplus
}
#endif

#endif /* __ALGORITHM_COMMON_PID_H__ */
