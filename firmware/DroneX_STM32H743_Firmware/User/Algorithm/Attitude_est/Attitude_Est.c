/**
 * @file Attitude_Est.c
 * @brief 姿态估计算法层：双正交 IMU 融合 + 姿态解算。
 *        坐标系变换使用 Attitude_Est_Config.h 中的 R_imu*_to_body，标定后硬编码填入。
 */

#include "Algorithm/Attitude_Est/Attitude_Est.h"
#include "Algorithm/Attitude_Est/Attitude_Est_Config.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ----------------------------------------------------------------------------
 * 内部：IMU 系到机体系的向量变换
 * ----------------------------------------------------------------------------
 * 使用 Attitude_Est_Config.h 中的 R_imu1_to_body / R_imu2_to_body。
 * 标定后只需修改 Config 中的矩阵，此处逻辑不变。
 */
static void apply_imu_to_body(const float R[3][3], const float v_imu[3], float v_body[3])
{
    v_body[0] = R[0][0] * v_imu[0] + R[0][1] * v_imu[1] + R[0][2] * v_imu[2];
    v_body[1] = R[1][0] * v_imu[0] + R[1][1] * v_imu[1] + R[1][2] * v_imu[2];
    v_body[2] = R[2][0] * v_imu[0] + R[2][1] * v_imu[1] + R[2][2] * v_imu[2];
}

/* ----------------------------------------------------------------------------
 * 内部状态
 * ---------------------------------------------------------------------------- */
static struct {
    float q[4];           /* 姿态四元数 [qw,qx,qy,qz] */
    float omega[3];       /* 融合后角速度 [rad/s] */
    float acc[3];         /* 融合后加速度 [m/s²] */
    IMU_Bias_t bias1;    /* IMU1 偏置 */
    IMU_Bias_t bias2;    /* IMU2 偏置 */
    uint8_t ready;        /* 是否已至少完成一次有效解算 */
} s_est;

/* ----------------------------------------------------------------------------
 * 接口实现
 * ---------------------------------------------------------------------------- */
void Attitude_Est_Init(void)
{
    memset(&s_est, 0, sizeof(s_est));
    s_est.q[0] = 1.0f;   /* 单位四元数 */
}

void Attitude_Est_Reset(void)
{
    s_est.q[0] = 1.0f;
    s_est.q[1] = s_est.q[2] = s_est.q[3] = 0.0f;
    s_est.omega[0] = s_est.omega[1] = s_est.omega[2] = 0.0f;
    s_est.acc[0] = s_est.acc[1] = s_est.acc[2] = 0.0f;
    /* 偏置保留，不重置 */
}

void Attitude_Est_Update(const IMU_Data_t *imu1, const IMU_Data_t *imu2, float dt_s)
{
    if (imu1 == NULL && imu2 == NULL) return;

    float gyro1_body[3], acc1_body[3];
    float gyro2_body[3], acc2_body[3];

    /* 1. 处理 IMU1（若有） */
    if (imu1 != NULL) {
        float g1[3] = {
            imu1->gyro[0] - s_est.bias1.gyro_bias[0],
            imu1->gyro[1] - s_est.bias1.gyro_bias[1],
            imu1->gyro[2] - s_est.bias1.gyro_bias[2],
        };
        float a1[3] = {
            imu1->accel[0] - s_est.bias1.accel_bias[0],
            imu1->accel[1] - s_est.bias1.accel_bias[1],
            imu1->accel[2] - s_est.bias1.accel_bias[2],
        };
        apply_imu_to_body(R_imu1_to_body, g1, gyro1_body);
        apply_imu_to_body(R_imu1_to_body, a1, acc1_body);
        acc1_body[0] = -acc1_body[0];
        acc1_body[1] = -acc1_body[1];
        acc1_body[2] = -acc1_body[2];
    }

    /* 2. 处理 IMU2（若有） */
    if (imu2 != NULL) {
        float g2[3] = {
            imu2->gyro[0] - s_est.bias2.gyro_bias[0],
            imu2->gyro[1] - s_est.bias2.gyro_bias[1],
            imu2->gyro[2] - s_est.bias2.gyro_bias[2],
        };
        float a2[3] = {
            imu2->accel[0] - s_est.bias2.accel_bias[0],
            imu2->accel[1] - s_est.bias2.accel_bias[1],
            imu2->accel[2] - s_est.bias2.accel_bias[2],
        };
        /* IMU2 系 → 主板系（使用 Config 中的 R_imu2_to_body） */
        apply_imu_to_body(R_imu2_to_body, g2, gyro2_body);
        apply_imu_to_body(R_imu2_to_body, a2, acc2_body);
        acc2_body[0] = -acc2_body[0];
        acc2_body[1] = -acc2_body[1];
        acc2_body[2] = -acc2_body[2];

        if (imu1 != NULL) {
            float w1 = ATTITUDE_EST_FUSION_WEIGHT_IMU1;
            float w2 = 1.0f - w1;
            s_est.omega[0] = w1 * gyro1_body[0] + w2 * gyro2_body[0];
            s_est.omega[1] = w1 * gyro1_body[1] + w2 * gyro2_body[1];
            s_est.omega[2] = w1 * gyro1_body[2] + w2 * gyro2_body[2];
            s_est.acc[0]   = w1 * acc1_body[0] + w2 * acc2_body[0];
            s_est.acc[1]   = w1 * acc1_body[1] + w2 * acc2_body[1];
            s_est.acc[2]   = w1 * acc1_body[2] + w2 * acc2_body[2];
        } else {
            memcpy(s_est.omega, gyro2_body, sizeof(gyro2_body));
            memcpy(s_est.acc, acc2_body, sizeof(acc2_body));
        }
    } else {
        memcpy(s_est.omega, gyro1_body, sizeof(gyro1_body));
        memcpy(s_est.acc, acc1_body, sizeof(acc1_body));
    }

    /* 4. 互补滤波姿态解算：陀螺积分 + 加速度计修正 */
    {
        float *q = s_est.q;
        float gx = s_est.omega[0], gy = s_est.omega[1], gz = s_est.omega[2];
        float ax = s_est.acc[0], ay = s_est.acc[1], az = s_est.acc[2];

        /* 加速度计归一化，用于重力方向修正 */
        float anorm = sqrtf(ax * ax + ay * ay + az * az);
        if (anorm > 0.5f) {
            ax /= anorm;
            ay /= anorm;
            az /= anorm;
            /* 期望重力 [0,0,-1] 在主板系：v = R*[0,0,-1] = [-R02,-R12,-R22] */
            float vx = -2.0f * (q[1] * q[3] + q[0] * q[2]);
            float vy = -2.0f * (q[2] * q[3] - q[0] * q[1]);
            float vz = 2.0f * (q[1] * q[1] + q[2] * q[2]) - 1.0f;
            /* 误差 = 测量 × 估计（叉积，修正轴） */
            float ex = ay * vz - az * vy;
            float ey = az * vx - ax * vz;
            float ez = ax * vy - ay * vx;
            float kp = 2.0f * (1.0f - ATTITUDE_EST_COMPL_FILTER_ALPHA) / dt_s;
            if (kp > 20.0f) kp = 20.0f;
            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
        }

        /* 四元数微分：q_dot = 0.5 * q ⊗ [0,ω] */
        float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
        q[0] += 0.5f * dt_s * (-qx * gx - qy * gy - qz * gz);
        q[1] += 0.5f * dt_s * ( qw * gx + qy * gz - qz * gy);
        q[2] += 0.5f * dt_s * ( qw * gy - qx * gz + qz * gx);
        q[3] += 0.5f * dt_s * ( qw * gz + qx * gy - qy * gx);

        /* 归一化 */
        float qnorm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (qnorm > 1e-6f) {
            q[0] /= qnorm;
            q[1] /= qnorm;
            q[2] /= qnorm;
            q[3] /= qnorm;
        }
    }
    s_est.ready = 1;
}

void Attitude_Est_GetQuat(float q[4])
{
    if (q) memcpy(q, s_est.q, 4 * sizeof(float));
}

void Attitude_Est_GetEuler(float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    float qw = s_est.q[0], qx = s_est.q[1], qy = s_est.q[2], qz = s_est.q[3];
    /* 四元数 → 旋转矩阵 R，按 ZXY(yaw-pitch-roll) 提取：R=Ry(roll)*Rx(pitch)*Rz(yaw) */
    float R02 = 2.0f * (qx * qz + qw * qy);
    float R22 = 1.0f - 2.0f * (qx * qx + qy * qy);
    float R12 = 2.0f * (qy * qz - qw * qx);
    float R10 = 2.0f * (qx * qy + qw * qz);
    float R11 = 1.0f - 2.0f * (qx * qx + qz * qz);

    if (roll_rad)  *roll_rad  = atan2f(R02, R22);
    if (pitch_rad) *pitch_rad = (fabsf(R12) >= 1.0f) ? copysignf(1.5707963267949f, R12) : asinf(-R12);
    if (yaw_rad)   *yaw_rad   = atan2f(R10, R11);
}

void Attitude_Est_GetAngularRate(float omega[3])
{
    if (omega) memcpy(omega, s_est.omega, 3 * sizeof(float));
}

void Attitude_Est_GetAccel(float acc[3])
{
    if (acc) memcpy(acc, s_est.acc, 3 * sizeof(float));
}

int Attitude_Est_SetBias(int imu_id, const IMU_Bias_t *bias)
{
    if (imu_id == ATTITUDE_EST_IMU_1) {
        if (bias) memcpy(&s_est.bias1, bias, sizeof(IMU_Bias_t));
        else      memset(&s_est.bias1, 0, sizeof(IMU_Bias_t));
        return 0;
    }
    if (imu_id == ATTITUDE_EST_IMU_2) {
        if (bias) memcpy(&s_est.bias2, bias, sizeof(IMU_Bias_t));
        else      memset(&s_est.bias2, 0, sizeof(IMU_Bias_t));
        return 0;
    }
    return -1;
}

int Attitude_Est_GetBias(int imu_id, IMU_Bias_t *bias)
{
    if (bias == NULL) return -1;
    if (imu_id == ATTITUDE_EST_IMU_1) { memcpy(bias, &s_est.bias1, sizeof(IMU_Bias_t)); return 0; }
    if (imu_id == ATTITUDE_EST_IMU_2) { memcpy(bias, &s_est.bias2, sizeof(IMU_Bias_t)); return 0; }
    return -1;
}

int Attitude_Est_IsReady(void)
{
    return s_est.ready ? 1 : 0;
}
