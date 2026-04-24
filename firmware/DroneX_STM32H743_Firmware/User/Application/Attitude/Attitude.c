/**
 * @file Attitude.c
 * @brief 姿态解算应用层：IMU 校准流程与持久化。
 */

#include "Application/Attitude/Attitude.h"
#include "Algorithm/Attitude_Est/Attitude_Est.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Flash/Flash_Param.h"
#include <math.h>
#include <string.h>

#define ATT_CALI_DURATION_MS      5000U
#define ATT_CAL_FLAG_IMU1         (1U << 0)
#define ATT_CAL_FLAG_IMU2         (1U << 1)

static struct {
    uint8_t active;
    int imu;
    uint32_t start_ms;
    uint32_t deadline_ms;
    uint32_t count1;
    uint32_t count2;
    float gyro_sum1[3];
    float gyro_sum2[3];
    float thresh;
} s_cali;

static void attitude_apply_saved_bias(const ParamFlash_Data_t *data)
{
    IMU_Bias_t bias;

    if ((data->flags & ATT_CAL_FLAG_IMU1) != 0U) {
        if (Attitude_Est_GetBias(ATTITUDE_EST_IMU_1, &bias) != 0) {
            memset(&bias, 0, sizeof(bias));
        }
        bias.gyro_bias[0] = data->imu1_gyro_bias[0];
        bias.gyro_bias[1] = data->imu1_gyro_bias[1];
        bias.gyro_bias[2] = data->imu1_gyro_bias[2];
        (void)Attitude_Est_SetBias(ATTITUDE_EST_IMU_1, &bias);
    }

    if ((data->flags & ATT_CAL_FLAG_IMU2) != 0U) {
        if (Attitude_Est_GetBias(ATTITUDE_EST_IMU_2, &bias) != 0) {
            memset(&bias, 0, sizeof(bias));
        }
        bias.gyro_bias[0] = data->imu2_gyro_bias[0];
        bias.gyro_bias[1] = data->imu2_gyro_bias[1];
        bias.gyro_bias[2] = data->imu2_gyro_bias[2];
        (void)Attitude_Est_SetBias(ATTITUDE_EST_IMU_2, &bias);
    }
}

static void attitude_save_bias_to_flash(const IMU_Bias_t *bias1, const IMU_Bias_t *bias2, int imu_sel)
{
    ParamFlash_Data_t data;
    HAL_StatusTypeDef st = Flash_Param_Load(&data);

    if (st != HAL_OK) {
        memset(&data, 0, sizeof(data));
    }

    if ((imu_sel == ATTITUDE_IMU_1) || (imu_sel == ATTITUDE_IMU_BOTH)) {
        data.flags |= ATT_CAL_FLAG_IMU1;
        data.imu1_gyro_bias[0] = bias1->gyro_bias[0];
        data.imu1_gyro_bias[1] = bias1->gyro_bias[1];
        data.imu1_gyro_bias[2] = bias1->gyro_bias[2];
    }

    if ((imu_sel == ATTITUDE_IMU_2) || (imu_sel == ATTITUDE_IMU_BOTH)) {
        data.flags |= ATT_CAL_FLAG_IMU2;
        data.imu2_gyro_bias[0] = bias2->gyro_bias[0];
        data.imu2_gyro_bias[1] = bias2->gyro_bias[1];
        data.imu2_gyro_bias[2] = bias2->gyro_bias[2];
    }

    st = Flash_Param_Save(&data);
    if (st == HAL_OK) {
        Debug_Printf("IMU cali: bias saved to flash\r\n");
    } else {
        Debug_Printf("IMU cali: flash save failed\r\n");
    }
}

static void attitude_finish_calibration(void)
{
    IMU_Bias_t bias1;
    IMU_Bias_t bias2;
    int fail = 0;

    if (((s_cali.imu == ATTITUDE_IMU_1) || (s_cali.imu == ATTITUDE_IMU_BOTH)) && (s_cali.count1 == 0U)) {
        fail = 1;
    }
    if (((s_cali.imu == ATTITUDE_IMU_2) || (s_cali.imu == ATTITUDE_IMU_BOTH)) && (s_cali.count2 == 0U)) {
        fail = 1;
    }

    if (fail) {
        Debug_Printf("IMU cali: failed, no valid sample\r\n");
        s_cali.active = 0U;
        return;
    }

    if (Attitude_Est_GetBias(ATTITUDE_EST_IMU_1, &bias1) != 0) {
        memset(&bias1, 0, sizeof(bias1));
    }
    if (Attitude_Est_GetBias(ATTITUDE_EST_IMU_2, &bias2) != 0) {
        memset(&bias2, 0, sizeof(bias2));
    }

    if ((s_cali.imu == ATTITUDE_IMU_1) || (s_cali.imu == ATTITUDE_IMU_BOTH)) {
        float inv_n1 = 1.0f / (float)s_cali.count1;
        bias1.gyro_bias[0] = s_cali.gyro_sum1[0] * inv_n1;
        bias1.gyro_bias[1] = s_cali.gyro_sum1[1] * inv_n1;
        bias1.gyro_bias[2] = s_cali.gyro_sum1[2] * inv_n1;
        (void)Attitude_Est_SetBias(ATTITUDE_EST_IMU_1, &bias1);
    }

    if ((s_cali.imu == ATTITUDE_IMU_2) || (s_cali.imu == ATTITUDE_IMU_BOTH)) {
        float inv_n2 = 1.0f / (float)s_cali.count2;
        bias2.gyro_bias[0] = s_cali.gyro_sum2[0] * inv_n2;
        bias2.gyro_bias[1] = s_cali.gyro_sum2[1] * inv_n2;
        bias2.gyro_bias[2] = s_cali.gyro_sum2[2] * inv_n2;
        (void)Attitude_Est_SetBias(ATTITUDE_EST_IMU_2, &bias2);
    }

    attitude_save_bias_to_flash(&bias1, &bias2, s_cali.imu);
    Debug_Printf(
        "IMU cali done: imu=%s n1=%lu n2=%lu b1=[%.5f %.5f %.5f] b2=[%.5f %.5f %.5f]\r\n",
        (s_cali.imu == ATTITUDE_IMU_BOTH) ? "both" : ((s_cali.imu == ATTITUDE_IMU_1) ? "1" : "2"),
        (unsigned long)s_cali.count1,
        (unsigned long)s_cali.count2,
        (double)bias1.gyro_bias[0],
        (double)bias1.gyro_bias[1],
        (double)bias1.gyro_bias[2],
        (double)bias2.gyro_bias[0],
        (double)bias2.gyro_bias[1],
        (double)bias2.gyro_bias[2]);

    s_cali.active = 0U;
}

void Attitude_Init(void)
{
    ParamFlash_Data_t data;
    memset(&s_cali, 0, sizeof(s_cali));

    if (Flash_Param_Load(&data) == HAL_OK) {
        if ((data.flags & (ATT_CAL_FLAG_IMU1 | ATT_CAL_FLAG_IMU2)) != 0U) {
            attitude_apply_saved_bias(&data);
            Debug_Printf("IMU cali: loaded from flash flags=0x%08lx\r\n", (unsigned long)data.flags);
        } else {
            Debug_Printf("IMU cali: flash has no valid bias, run imu_cali\r\n");
        }
    } else {
        Debug_Printf("IMU cali: no flash data, run imu_cali\r\n");
    }
}

int Attitude_RequestImuCali(int imu, float time_s, float thresh)
{
    uint32_t now_ms = HAL_GetTick();

    if (imu != 0 && imu != 1 && imu != 2)
        return -1;
    if (thresh < 0.f)
        return -1;
    if (s_cali.active != 0U)
        return -1;

    (void)time_s;

    memset(s_cali.gyro_sum1, 0, sizeof(s_cali.gyro_sum1));
    memset(s_cali.gyro_sum2, 0, sizeof(s_cali.gyro_sum2));
    s_cali.count1 = 0U;
    s_cali.count2 = 0U;
    s_cali.active = 1U;
    s_cali.start_ms = now_ms;
    s_cali.deadline_ms = now_ms + ATT_CALI_DURATION_MS;
    s_cali.imu = imu;
    s_cali.thresh = thresh;

    Debug_Printf("IMU cali start: imu=%s duration=5.0s\r\n",
                 (imu == ATTITUDE_IMU_BOTH) ? "both" : ((imu == ATTITUDE_IMU_1) ? "1" : "2"));

    return 0;
}

void Attitude_OnImuSample(const IMU_Data_t *imu1, int imu1_valid,
                          const IMU_Data_t *imu2, int imu2_valid)
{
    uint32_t now_ms;

    if (s_cali.active == 0U) {
        return;
    }

    if (((s_cali.imu == ATTITUDE_IMU_1) || (s_cali.imu == ATTITUDE_IMU_BOTH)) &&
        (imu1_valid != 0) && (imu1 != NULL)) {
        s_cali.gyro_sum1[0] += imu1->gyro[0];
        s_cali.gyro_sum1[1] += imu1->gyro[1];
        s_cali.gyro_sum1[2] += imu1->gyro[2];
        s_cali.count1++;
    }

    if (((s_cali.imu == ATTITUDE_IMU_2) || (s_cali.imu == ATTITUDE_IMU_BOTH)) &&
        (imu2_valid != 0) && (imu2 != NULL)) {
        s_cali.gyro_sum2[0] += imu2->gyro[0];
        s_cali.gyro_sum2[1] += imu2->gyro[1];
        s_cali.gyro_sum2[2] += imu2->gyro[2];
        s_cali.count2++;
    }

    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - s_cali.deadline_ms) >= 0) {
        attitude_finish_calibration();
    }
}

void Attitude_Task(void)
{
    (void)s_cali.thresh;
}
