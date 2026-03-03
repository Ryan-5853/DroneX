/**
 * @file Cmd_handlers.c
 * @brief Cmd 模块内置命令实现：help、version、status、reset、imu_cali 等。
 */

#include "Debug/Cmd/Cmd.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "Application/Attitude/Attitude.h"
#include "Debug/ESC_configurer/ESC_configurer.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * 系统级命令
 * --------------------------------------------------------------------------- */
static void Cmd_Help(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    (void)params;
    (void)n;
    /* 合并为单条发送，避免多路 Debug_Printf 交错导致撕裂 */
    Debug_Printf("DroneX cmd: <cmd>:para1=xx;para2=xx;\r\n");
    Debug_Printf("help, version, status, reset, imu_cali, ESC_cfg\r\n");
}

static void Cmd_Version(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    (void)params;
    (void)n;
    Debug_Printf("DroneX v0.1\r\n");
}

static void Cmd_Status(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    (void)params;
    (void)n;
    Debug_Printf("status ok\r\n");
}

static void Cmd_Reset(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    (void)params;
    (void)n;
    Debug_Printf("OK reset\r\n");
    HAL_Delay(50);  /* 等待串口输出完成 */
    HAL_NVIC_SystemReset();
}

/* ---------------------------------------------------------------------------
 * 应用级命令
 * --------------------------------------------------------------------------- */
static void Cmd_ImuCali(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    const char *imu_s = Cmd_GetParam(params, n, "imu");
    const char *time_s = Cmd_GetParam(params, n, "time");
    const char *thresh_s = Cmd_GetParam(params, n, "thresh");
    if (!imu_s || !time_s || !thresh_s) {
        Debug_Printf("ERR imu_cali need imu,time,thresh\r\n");
        return;
    }
    int imu = 0;
    if (imu_s[0] == '1') imu = 1;
    else if (imu_s[0] == '2') imu = 2;
    else if (imu_s[0] == '0' || strcmp(imu_s, "both") == 0) imu = 0;
    else {
        Debug_Printf("ERR imu=1|2|0|both\r\n");
        return;
    }
    float t = 0.f, th = 0.f;
    if (sscanf(time_s, "%f", &t) != 1 || sscanf(thresh_s, "%f", &th) != 1) {
        Debug_Printf("ERR time,thresh must be number\r\n");
        return;
    }
    if (Attitude_RequestImuCali(imu, t, th) != 0) {
        Debug_Printf("ERR imu_cali param invalid\r\n");
        return;
    }
    Debug_Printf("OK imu_cali queued\r\n");
}

static void Cmd_EscCfg(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    const char *num_s = Cmd_GetParam(params, n, "num");
    if (!num_s) {
        Debug_Printf("ERR ESC_cfg need num\r\n");
        return;
    }
    int num = 0;
    if (sscanf(num_s, "%d", &num) != 1) {
        Debug_Printf("ERR num must be integer\r\n");
        return;
    }
    if (ESC_Config(num) != 0) {
        Debug_Printf("ERR ESC_cfg param invalid\r\n");
        return;
    }
    Debug_Printf("OK ESC_cfg done\r\n");
}

/* ---------------------------------------------------------------------------
 * 注册内置命令
 * --------------------------------------------------------------------------- */
void Cmd_RegisterBuiltins(void)
{
    Cmd_Register("help", Cmd_Help);
    Cmd_Register("version", Cmd_Version);
    Cmd_Register("status", Cmd_Status);
    Cmd_Register("reset", Cmd_Reset);
    Cmd_Register("imu_cali", Cmd_ImuCali);
    Cmd_Register("ESC_cfg", Cmd_EscCfg);
}
