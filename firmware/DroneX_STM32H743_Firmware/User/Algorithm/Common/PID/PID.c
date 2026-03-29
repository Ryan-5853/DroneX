/**
 * @file PID.c
 * @brief PID 实现：位置式，时间戳差分得到 dt。
 */

#include "Algorithm/Common/PID/PID.h"
#include <string.h>

static float pid_clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

int PID_Init(PID_t *pid, const PID_Gains_t *gains)
{
    if (pid == NULL || gains == NULL) {
        return PID_ERR_NULL;
    }
    memset(pid, 0, sizeof(*pid));
    pid->kp = gains->kp;
    pid->ki = gains->ki;
    pid->kd = gains->kd;
    return PID_OK;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_time_us = 0u;
    pid->has_time = 0u;
}

int PID_Update(PID_t *pid, float error, uint32_t time_us, float *output)
{
    float p_term;
    float i_term;
    float d_term;
    float dt_s;
    float u;

    if (pid == NULL || output == NULL) {
        return PID_ERR_NULL;
    }

    p_term = pid->kp * error;

    if (!pid->has_time) {
        pid->prev_time_us = time_us;
        pid->prev_error = error;
        pid->has_time = 1u;
        u = p_term;
        if (pid->output_limited) {
            u = pid_clampf(u, pid->out_min, pid->out_max);
        }
        *output = u;
        return PID_OK;
    }

    dt_s = (float)(time_us - pid->prev_time_us) * (1.0f / 1000000.0f);
    if (time_us < pid->prev_time_us || dt_s <= 0.0f) {
        return PID_ERR_TIME;
    }

    if (pid->ki != 0.0f) {
        pid->integral += error * dt_s;
        if (pid->integral_limited) {
            pid->integral = pid_clampf(pid->integral, pid->i_min, pid->i_max);
        }
    }
    i_term = pid->ki * pid->integral;

    if (pid->kd != 0.0f) {
        d_term = pid->kd * (error - pid->prev_error) / dt_s;
    } else {
        d_term = 0.0f;
    }

    u = p_term + i_term + d_term;

    if (pid->output_limited) {
        u = pid_clampf(u, pid->out_min, pid->out_max);
    }

    *output = u;

    pid->prev_time_us = time_us;
    pid->prev_error = error;

    return PID_OK;
}

int PID_SetGains(PID_t *pid, const PID_Gains_t *gains)
{
    if (pid == NULL || gains == NULL) {
        return PID_ERR_NULL;
    }
    pid->kp = gains->kp;
    pid->ki = gains->ki;
    pid->kd = gains->kd;
    return PID_OK;
}

int PID_GetGains(const PID_t *pid, PID_Gains_t *gains)
{
    if (pid == NULL || gains == NULL) {
        return PID_ERR_NULL;
    }
    gains->kp = pid->kp;
    gains->ki = pid->ki;
    gains->kd = pid->kd;
    return PID_OK;
}

int PID_SetOutputLimits(PID_t *pid, const float *min_out, const float *max_out)
{
    if (pid == NULL) {
        return PID_ERR_NULL;
    }
    if (min_out == NULL || max_out == NULL) {
        pid->output_limited = 0u;
        return PID_OK;
    }
    if (*min_out > *max_out) {
        return PID_ERR_LIMIT;
    }
    pid->out_min = *min_out;
    pid->out_max = *max_out;
    pid->output_limited = 1u;
    return PID_OK;
}

int PID_GetOutputLimits(const PID_t *pid, float *min_out, float *max_out, int *enabled)
{
    if (pid == NULL) {
        return PID_ERR_NULL;
    }
    if (min_out != NULL) {
        *min_out = pid->out_min;
    }
    if (max_out != NULL) {
        *max_out = pid->out_max;
    }
    if (enabled != NULL) {
        *enabled = pid->output_limited ? 1 : 0;
    }
    return PID_OK;
}

int PID_SetIntegralLimits(PID_t *pid, const float *min_i, const float *max_i)
{
    if (pid == NULL) {
        return PID_ERR_NULL;
    }
    if (min_i == NULL || max_i == NULL) {
        pid->integral_limited = 0u;
        return PID_OK;
    }
    if (*min_i > *max_i) {
        return PID_ERR_LIMIT;
    }
    pid->i_min = *min_i;
    pid->i_max = *max_i;
    pid->integral_limited = 1u;
    if (pid->ki != 0.0f) {
        pid->integral = pid_clampf(pid->integral, pid->i_min, pid->i_max);
    }
    return PID_OK;
}

int PID_GetIntegralLimits(const PID_t *pid, float *min_i, float *max_i, int *enabled)
{
    if (pid == NULL) {
        return PID_ERR_NULL;
    }
    if (min_i != NULL) {
        *min_i = pid->i_min;
    }
    if (max_i != NULL) {
        *max_i = pid->i_max;
    }
    if (enabled != NULL) {
        *enabled = pid->integral_limited ? 1 : 0;
    }
    return PID_OK;
}
