/**
 * @file Attitude.c
 * @brief 姿态解算应用层：IMU 校准请求处理（占位实现）。
 */

#include "Application/Attitude/Attitude.h"
#include "Driver/Debug/Debug.h"
#include <string.h>

static struct {
    uint8_t pending;
    int imu;
    float time_s;
    float thresh;
} s_cali;

int Attitude_RequestImuCali(int imu, float time_s, float thresh)
{
    if (imu != 0 && imu != 1 && imu != 2)
        return -1;
    if (time_s <= 0.f || time_s > 300.f)  /* 限制 0~300 秒 */
        return -1;
    if (thresh < 0.f)
        return -1;

    s_cali.pending = 1;
    s_cali.imu = imu;
    s_cali.time_s = time_s;
    s_cali.thresh = thresh;
    return 0;
}

void Attitude_Task(void)
{
    if (!s_cali.pending) return;

    s_cali.pending = 0;
    const char *imu_str = (s_cali.imu == 0) ? "both" : (s_cali.imu == 1) ? "1" : "2";
    Debug_Printf("IMU cali: imu=%s time=%.1fs thresh=%.3f (stub)\r\n",
        imu_str, (double)s_cali.time_s, (double)s_cali.thresh);
}
