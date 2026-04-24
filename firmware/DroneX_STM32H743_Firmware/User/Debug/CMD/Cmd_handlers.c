/**
 * @file Cmd_handlers.c
 * @brief Cmd 模块内置命令实现：help、version、status、reset、imu_cali 等。
 */

#include "Debug/Cmd/Cmd.h"
#include "Debug/Cmd/Cmd_cfg_table.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "Application/Attitude/Attitude.h"
#include "Debug/ESC_configurer/ESC_configurer.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
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
    Debug_Printf("help, version, status, reset, imu_cali, ESC_cfg, cfg:name=;val=, power:N(VOFA+)\r\n");
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
    Debug_BlockingPrintf("OK reset\r\n");
    __DSB();
    __ISB();
    NVIC_SystemReset();
    while (1) {
    }
}

/* ---------------------------------------------------------------------------
 * 应用级命令
 * --------------------------------------------------------------------------- */
static void Cmd_ImuCali(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    const char *imu_s = Cmd_GetParam(params, n, "imu");
    const char *thresh_s = Cmd_GetParam(params, n, "thresh");
    float t = 5.0f;
    float th = 0.0f;

    if (!imu_s) {
        Debug_Printf("ERR imu_cali need imu\r\n");
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
    if (thresh_s && (sscanf(thresh_s, "%f", &th) != 1)) {
        Debug_Printf("ERR thresh must be number\r\n");
        return;
    }
    if (Attitude_RequestImuCali(imu, t, th) != 0) {
        Debug_Printf("ERR imu_cali busy or invalid\r\n");
        return;
    }
    Debug_Printf("OK imu_cali queued (auto 5s)\r\n");
}

/**
 * cfg:name=<参数名>;val=<值>;
 * 整型：十进制或 0x 十六进制（strtol）；浮点： sscanf %f。
 */
static void Cmd_Cfg(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    const char *name_s = Cmd_GetParam(params, n, "name");
    const char *val_s = Cmd_GetParam(params, n, "val");
    if (!name_s || !val_s) {
        Debug_Printf("ERR cfg need name,val\r\n");
        return;
    }
    const Cmd_Cfg_Entry_t *e = Cmd_Cfg_Lookup(name_s);
    if (e == NULL || e->addr == NULL) {
        Debug_Printf("ERR cfg unknown name\r\n");
        return;
    }
    switch (e->type) {
    case CMD_CFG_TYPE_INT32: {
        char *end = NULL;
        errno = 0;
        long v = strtol(val_s, &end, 0);
        if (end == val_s || (end != NULL && *end != '\0')) {
            Debug_Printf("ERR cfg val not integer\r\n");
            return;
        }
        if (errno == ERANGE || v < (long)INT32_MIN || v > (long)INT32_MAX) {
            Debug_Printf("ERR cfg int32 range\r\n");
            return;
        }
        *(int32_t *)e->addr = (int32_t)v;
        break;
    }
    case CMD_CFG_TYPE_FLOAT: {
        float v = 0.f;
        if (sscanf(val_s, "%f", &v) != 1) {
            Debug_Printf("ERR cfg val not float\r\n");
            return;
        }
        *(float *)e->addr = v;
        break;
    }
    default:
        Debug_Printf("ERR cfg bad type\r\n");
        return;
    }
    Debug_Printf("OK cfg %s\r\n", name_s);
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

/* VOFA+ 控件发送 power:N 时，动力值已在 Debug_OnLine 中解析并更新，此处仅避免报 unknown */
static void Cmd_Power(const char *cmd, const Cmd_Param_t *params, int n)
{
    (void)cmd;
    (void)params;
    (void)n;
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
    Cmd_Register("cfg", Cmd_Cfg);
    Cmd_Register("power", Cmd_Power);
}
