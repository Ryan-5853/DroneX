/**
 * @file Att_Ctrl_PID.c
 * @brief 垂起姿态 PID：偏航差速油门 + 舵机滚俯。
 */

#include "Algorithm/Att_Ctrl_PID/Att_Ctrl_PID.h"
#include <math.h>
#include <string.h>

#ifndef ATT_CTRL_PID_PI_F
#define ATT_CTRL_PID_PI_F 3.14159265358979323846f
#endif

typedef struct {
    uint8_t valid;
    Att_Ctrl_PID_Config_t cfg;
    PID_t pid_roll;
    PID_t pid_pitch;
    PID_t pid_yaw;
} Att_Ctrl_PID_State_t;

static Att_Ctrl_PID_State_t s_ac;

static float ac_clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/** 最短角差 wrap 到 (-π, π] */
static float angle_err_rad(float sp_rad, float meas_rad)
{
    float e = sp_rad - meas_rad;
    while (e > ATT_CTRL_PID_PI_F) {
        e -= 2.0f * ATT_CTRL_PID_PI_F;
    }
    while (e <= -ATT_CTRL_PID_PI_F) {
        e += 2.0f * ATT_CTRL_PID_PI_F;
    }
    return e;
}

static int pid_apply_limits(PID_t *pid, float out_min, float out_max,
    uint8_t i_en, float i_min, float i_max)
{
    int rc;

    rc = PID_SetOutputLimits(pid, &out_min, &out_max);
    if (rc != PID_OK) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    if (i_en) {
        rc = PID_SetIntegralLimits(pid, &i_min, &i_max);
    } else {
        rc = PID_SetIntegralLimits(pid, NULL, NULL);
    }
    if (rc != PID_OK) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    return ATT_CTRL_PID_OK;
}

static int validate_cfg(const Att_Ctrl_PID_Config_t *c)
{
    if (c->roll_out_min > c->roll_out_max || c->pitch_out_min > c->pitch_out_max
        || c->yaw_out_min > c->yaw_out_max) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    if (c->throttle_min > c->throttle_max || c->servo_min_rad > c->servo_max_rad) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    if (c->enable_roll_i_limit && c->roll_i_min > c->roll_i_max) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    if (c->enable_pitch_i_limit && c->pitch_i_min > c->pitch_i_max) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    if (c->enable_yaw_i_limit && c->yaw_i_min > c->yaw_i_max) {
        return ATT_CTRL_PID_ERR_LIMIT;
    }
    return ATT_CTRL_PID_OK;
}

int Att_Ctrl_PID_Init(const Att_Ctrl_PID_Config_t *cfg)
{
    int rc;

    if (cfg == NULL) {
        return ATT_CTRL_PID_ERR_NULL;
    }
    rc = validate_cfg(cfg);
    if (rc != ATT_CTRL_PID_OK) {
        return rc;
    }

    memset(&s_ac, 0, sizeof(s_ac));
    memcpy(&s_ac.cfg, cfg, sizeof(*cfg));

    rc = PID_Init(&s_ac.pid_roll, &cfg->gains_roll);
    if (rc != PID_OK) {
        return ATT_CTRL_PID_ERR_NULL;
    }
    rc = PID_Init(&s_ac.pid_pitch, &cfg->gains_pitch);
    if (rc != PID_OK) {
        return ATT_CTRL_PID_ERR_NULL;
    }
    rc = PID_Init(&s_ac.pid_yaw, &cfg->gains_yaw);
    if (rc != PID_OK) {
        return ATT_CTRL_PID_ERR_NULL;
    }

    rc = pid_apply_limits(&s_ac.pid_roll, cfg->roll_out_min, cfg->roll_out_max,
        cfg->enable_roll_i_limit, cfg->roll_i_min, cfg->roll_i_max);
    if (rc != ATT_CTRL_PID_OK) {
        return rc;
    }
    rc = pid_apply_limits(&s_ac.pid_pitch, cfg->pitch_out_min, cfg->pitch_out_max,
        cfg->enable_pitch_i_limit, cfg->pitch_i_min, cfg->pitch_i_max);
    if (rc != ATT_CTRL_PID_OK) {
        return rc;
    }
    rc = pid_apply_limits(&s_ac.pid_yaw, cfg->yaw_out_min, cfg->yaw_out_max,
        cfg->enable_yaw_i_limit, cfg->yaw_i_min, cfg->yaw_i_max);
    if (rc != ATT_CTRL_PID_OK) {
        return rc;
    }

    s_ac.valid = 1u;
    return ATT_CTRL_PID_OK;
}

void Att_Ctrl_PID_Reset(void)
{
    if (!s_ac.valid) {
        return;
    }
    PID_Reset(&s_ac.pid_roll);
    PID_Reset(&s_ac.pid_pitch);
    PID_Reset(&s_ac.pid_yaw);
}

int Att_Ctrl_PID_Update(const Att_Ctrl_PID_In_t *in, Att_Ctrl_PID_Out_t *out)
{
    float e_roll;
    float e_pitch;
    float e_yaw;
    float u_roll;
    float u_pitch;
    float u_yaw;
    float t_mid;
    int rc;

    if (!s_ac.valid || in == NULL || out == NULL) {
        return (in == NULL || out == NULL) ? ATT_CTRL_PID_ERR_NULL : ATT_CTRL_PID_ERR_NOT_INIT;
    }

    e_roll = angle_err_rad(in->roll_sp_rad, in->roll_rad);
    e_pitch = angle_err_rad(in->pitch_sp_rad, in->pitch_rad);
    e_yaw = angle_err_rad(in->yaw_sp_rad, in->yaw_rad);

    rc = PID_Update(&s_ac.pid_roll, e_roll, in->time_us, &u_roll);
    if (rc != PID_OK) {
        return (rc == PID_ERR_TIME) ? ATT_CTRL_PID_ERR_TIME : ATT_CTRL_PID_ERR_NULL;
    }
    rc = PID_Update(&s_ac.pid_pitch, e_pitch, in->time_us, &u_pitch);
    if (rc != PID_OK) {
        return (rc == PID_ERR_TIME) ? ATT_CTRL_PID_ERR_TIME : ATT_CTRL_PID_ERR_NULL;
    }
    rc = PID_Update(&s_ac.pid_yaw, e_yaw, in->time_us, &u_yaw);
    if (rc != PID_OK) {
        return (rc == PID_ERR_TIME) ? ATT_CTRL_PID_ERR_TIME : ATT_CTRL_PID_ERR_NULL;
    }

    {
        const Att_Ctrl_PID_Config_t *c = &s_ac.cfg;
        float m0;
        float m1;
        float s0;
        float s1;

        t_mid = c->base_throttle + c->pitch_throttle_common * u_pitch;
        m0 = t_mid + c->yaw_throttle_diff * u_yaw;
        m1 = t_mid - c->yaw_throttle_diff * u_yaw;
        out->throttle_motor[0] = ac_clampf(m0, c->throttle_min, c->throttle_max);
        out->throttle_motor[1] = ac_clampf(m1, c->throttle_min, c->throttle_max);

        s0 = c->servo_trim_rad[0] + c->pitch_servo_common * u_pitch
            + c->roll_servo_diff * u_roll;
        s1 = c->servo_trim_rad[1] + c->pitch_servo_common * u_pitch
            - c->roll_servo_diff * u_roll;
        out->servo_rad[0] = ac_clampf(s0, c->servo_min_rad, c->servo_max_rad);
        out->servo_rad[1] = ac_clampf(s1, c->servo_min_rad, c->servo_max_rad);
    }

    return ATT_CTRL_PID_OK;
}
