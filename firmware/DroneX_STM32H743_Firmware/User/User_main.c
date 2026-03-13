/**
 * @file User_main.c
 * @brief 用户主程序：初始化与主循环逻辑，由 main.c 调用。
 */

#include "User_main.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "Driver/SD/SD.h"
#include "Debug/Cmd/Cmd.h"
#include "Application/Attitude/Attitude.h"
#include "Algorithm/Attitude_Est/Attitude_Est.h"
#include "Driver/IMU/IMU_interface.h"
#include "Driver/IMU/ICM42688.h"
#include "Driver/PIT/PIT.h"
#include "Driver/Timing/Timing.h"
#include "Driver/ESC/ESC.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 临时硬件自检：1=仅翻转 PA2，0=正常业务逻辑 */
#define PA2_TOGGLE_TEST_ONLY  0
/* 调试期原始 DShot 阶梯测试（建议拆桨后使用） */
#define ESC_RAMP_STEP0_RAW      300U
#define ESC_RAMP_STEP1_RAW      500U
#define ESC_RAMP_STEP2_RAW      700U
#define ESC_RAMP_STEP_MS       1000U
#define ESC_RAMP_ZERO_TAIL_MS  2000U
#define ESC_ARM_ZERO_MS         3000U

/* 陀螺仪1 句柄与配置 */
static IMU_Handle_t     s_imu1_handle;
static ICM42688_SPI_Config_t s_imu1_cfg;
/* 陀螺仪2 句柄与配置 */
static IMU_Handle_t     s_imu2_handle;
static ICM42688_SPI_Config_t s_imu2_cfg;
static volatile uint8_t s_esc_dbg_report_flag;
static uint32_t s_esc_arm_start_ms;
static uint8_t s_esc_test_state;      /* 0=arming, 1=ramp */
static uint16_t s_esc_test_raw;

/* VOFA+ 上位机动力参数：串口解析 "power:N" 更新，0=停转，1..1000=动力档，由 ESC_1kHz_Task 按原频率打包 DShot 发送 */
static volatile uint16_t s_power_from_uart = 0U;
#define POWER_UART_MAX  1000U

/* ESC 周期发送任务：发送频率不变，油门值来自串口解析的 s_power_from_uart（power:0..1000），打包 DShot 发送 */
static void ESC_1kHz_Task(void)
{
    uint16_t p = s_power_from_uart;
    uint16_t raw;

    if (p == 0U) {
        raw = 0U;
    } else {
        /* 1..1000 线性映射到 DShot 48..2047 */
        raw = (uint16_t)(48U + (uint32_t)(p - 1U) * (2047U - 48U) / (POWER_UART_MAX - 1U));
    }

    (void)ESC_SetThrottleRaw(ESC_CH1, raw, false);
    (void)ESC_SetThrottleRaw(ESC_CH2, raw, false);
    ESC_TestPwm_SetPower(p);  /* PD15 调试 PWM 与 power 同步：0->1ms，1000->2ms */
    ESC_RecoverIfStuck(2);
    if (!ESC_IsBusy()) {
        (void)ESC_Update();
    }
}

static void ESC_DebugFlag_Task(void)
{
    s_esc_dbg_report_flag = 1U;
}

/** 自检：验证各模块初始化结果，并输出到 Debug */
static void User_SelfCheck(void)
{
    IMU_Data_t imu_data;
    int imu1_ok = (IMU_Read(&s_imu1_handle, &imu_data) == IMU_OK);
    int imu2_ok = (IMU_Read(&s_imu2_handle, &imu_data) == IMU_OK);
    // int sd_ok   = SD_IsReady();
    uint32_t cy = Timing_GetCycles();
    int timing_ok = (cy > 0);

    Debug_Printf("=== SelfCheck ===\r\n");
    Debug_Printf("  IMU1:  %s\r\n", imu1_ok ? "OK" : "FAIL");
    Debug_Printf("  IMU2:  %s\r\n", imu2_ok ? "OK" : "FAIL");
    // Debug_Printf("  SD:    %s\r\n", sd_ok ? "OK" : "FAIL");
    Debug_Printf("  DWT:   %s\r\n", timing_ok ? "OK" : "FAIL");
    Debug_Printf("  Tick:  %lu ms\r\n", (unsigned long)HAL_GetTick());
    Debug_Printf("================\r\n");

    /* 短暂轮询以尽量送出自检结果（非阻塞） */
    // for (int i = 0; i < 50; i++) {
    //     HAL_Delay(1);
    //     Debug_Process();
    // }
}

/** 解析 VOFA+ 控件格式 "power:N"，仅做数据更新，返回 1 表示已消费该行。
 *  有界解析：最多读 4 位数字，避免超长数字导致 strtol 或 Cmd 处理卡死。 */
static int ParsePowerFromUart(const char *buf)
{
    if (buf == NULL) return 0;
    if (strncmp(buf, "power:", 6) != 0) return 0;
    const char *p = buf + 6;
    while (*p == ' ' || *p == '\t') p++;
    if (*p < '0' || *p > '9') return 0;
    uint32_t val = 0U;
    const int max_digits = 4;  /* 0..9999，之后 clamp 到 0..1000 */
    for (int i = 0; i < max_digits && *p >= '0' && *p <= '9'; i++, p++)
        val = val * 10U + (uint32_t)(*p - '0');
    if (val > POWER_UART_MAX) val = POWER_UART_MAX;
    s_power_from_uart = (uint16_t)val;
    return 1;
}

/** Debug 层：Driver 拆包后的指令包回调，交由 Cmd 解析执行；先尝试解析 power:N 作为动力参数 */
static void Debug_OnLine(const char *buf)
{
    if (ParsePowerFromUart(buf)) {
        /* 已作为动力参数更新，不再交给 Cmd，避免长行 "power:99999..." 进入 Cmd 缓冲区导致卡死 */
        return;
    }
    Cmd_Status_t ret = Cmd_Process(buf);
    if (ret == CMD_ERR_UNKNOWN)
        Debug_Printf("ERR unknown cmd:[%s]\r\n", buf);
    else if (ret == CMD_ERR_PARAM || ret == CMD_ERR_TOO_LONG)
        Debug_Printf("ERR format\r\n");
}

/** 200Hz 定时任务：读取双 IMU，解算姿态，输出欧拉角与周期耗时 [roll pitch yaw] deg, [us] */
static void IMU_500Hz_Task(void)
{
    uint32_t t0 = Timing_GetCycles();

    IMU_Data_t data1, data2;
    int ok1 = (IMU_Read(&s_imu1_handle, &data1) == IMU_OK);
    int ok2 = (IMU_Read(&s_imu2_handle, &data2) == IMU_OK);
    if (ok1 && ok2)
        Attitude_Est_Update(&data1, &data2, 0.005f);
    else if (ok1)
        Attitude_Est_Update(&data1, NULL, 0.005f);
    else if (ok2)
        Attitude_Est_Update(NULL, &data2, 0.005f);

    uint32_t t1 = Timing_GetCycles();
    uint32_t us = Timing_CyclesToUs(t1 - t0);

    if (Attitude_Est_IsReady()) {
        float roll, pitch, yaw;
        Attitude_Est_GetEuler(&roll, &pitch, &yaw);
        // Debug_Printf("EUL %.2f %.2f %.2f %lu us\r\n",
        //     (double)(roll * 57.29577951308232f),
        //     (double)(pitch * 57.29577951308232f),
        //     (double)(yaw * 57.29577951308232f),
        //     (unsigned long)us);
    }
}

/**
 * @brief 用户初始化
 */
void User_Main_Init(void)
{
    Debug_Init(Debug_Transport_Send);
    Cmd_Init();
    Cmd_RegisterBuiltins();
    Debug_UART_SetLineCallback(Debug_OnLine);
    Debug_UART_Init();

    Timing_Init();
    // Attitude_Est_Init();

    /* 陀螺仪1：SPI1，CS=PE2 */
    s_imu1_cfg.hspi   = &hspi1;
    s_imu1_cfg.cs_port = GPIOE;
    s_imu1_cfg.cs_pin  = GPIO_PIN_2;
    // IMU_Interface_Init(&s_imu1_handle, IMU_TYPE_ICM42688, &s_imu1_cfg);

    // (void)IMU_Init(&s_imu1_handle);

    /* 陀螺仪2：SPI4，CS=PE11 */
    s_imu2_cfg.hspi    = &hspi4;
    s_imu2_cfg.cs_port = GPIOE;
    s_imu2_cfg.cs_pin  = GPIO_PIN_11;
    // IMU_Interface_Init(&s_imu2_handle, IMU_TYPE_ICM42688, &s_imu2_cfg);

    // (void)IMU_Init(&s_imu2_handle);

    /* SD 卡初始化（可选：无卡时不影响启动，日志模块可调用 SD_IsReady 检查） */
    // (void)SD_Init();

    /* ESC 初始化：双向 DShot300（TIM2_CH3/CH4），上电先发送停转帧 */
    if (ESC_Init() == ESC_OK) {
        Debug_BlockingPrintf("ESC_Init OK\r\n");
        (void)ESC_SetThrottleBidirectional(ESC_CH1, 0);
        (void)ESC_SetThrottleBidirectional(ESC_CH2, 0);
        (void)ESC_Update();
        s_esc_arm_start_ms = HAL_GetTick();
        s_esc_test_state = 0U;
        s_esc_test_raw = 0U;
        Debug_BlockingPrintf("ESC: throttle from UART power:0..%u (e.g. power:500), 1kHz DShot\r\n",
            (unsigned)POWER_UART_MAX);
    }
    else {
        Debug_BlockingPrintf("ESC_Init FAILED\r\n");
    }
    

    /* 使用 PD15=TIM4_CH4 输出 50Hz PWM，用于测试模拟 PWM 电调/舵机信号输入 */
    ESC_TestPwm_StartOnPD15();

    /* 200Hz IMU 任务：PIT_CH0，周期 5000us，NVIC 优先级 1（高优先级） */
    PIT_Config_t pit_cfg = {
        .period_us = 5000,   /* 200Hz = 5ms */
        .priority  = 1,
        .callback  = IMU_500Hz_Task,
    };
    if (PIT_Init(PIT_CH0, &pit_cfg) == PIT_OK)
        // PIT_Start(PIT_CH0);

    /* 200Hz 姿态解算任务：PIT_CH1（含 imu_cali 占位实现） */
    pit_cfg.period_us = 5000;
    pit_cfg.priority  = 2;
    pit_cfg.callback  = Attitude_Task;
    if (PIT_Init(PIT_CH1, &pit_cfg) == PIT_OK)
        // PIT_Start(PIT_CH1);

    /* 4kHz ESC 发送任务：提高油门帧连续性 */
    pit_cfg.period_us = 2000;
    pit_cfg.priority  = 3;
    pit_cfg.callback  = ESC_1kHz_Task;
    if (PIT_Init(PIT_CH2, &pit_cfg) == PIT_OK)
        PIT_Start(PIT_CH2);

    /* 5Hz ESC 调试标志任务：在主循环里打印状态，避免在中断里打印 */
    pit_cfg.period_us = 1000000;
    pit_cfg.priority  = 10;
    pit_cfg.callback  = ESC_DebugFlag_Task;
    if (PIT_Init(PIT_CH3, &pit_cfg) == PIT_OK)
        PIT_Start(PIT_CH3);

    /* 自检并输出结果 */
    // User_SelfCheck();

    // (void)ESC_SetThrottleBidirectional(ESC_CH1, 100);
    // (void)ESC_Update();
    // Debug_Printf("ESC_CH1 100\r\n");
    // HAL_Delay(1000);
    // (void)ESC_SetThrottleBidirectional(ESC_CH1, 0);
    // (void)ESC_Update();
    // Debug_Printf("ESC_CH1 0\r\n");
}

/**
 * @brief 用户主循环体
 */
void User_Main_Loop(void)
{
    if (s_esc_dbg_report_flag) {
        ESC_DebugInfo_t info;
        s_esc_dbg_report_flag = 0U;
        ESC_GetDebugInfo(&info);
        Debug_Printf("ESC DBG init=%u busy=%u msp=%lu upd_req=%lu upd_ok=%lu busy_rej=%lu fail3=%lu fail4=%lu done3=%lu done4=%lu irq3=%lu irq4=%lu raw1=%u raw2=%u\r\n",
            (unsigned)info.initialized,
            (unsigned)info.dma_busy,
            (unsigned long)info.msp_init_count,
            (unsigned long)info.update_req_count,
            (unsigned long)info.update_ok_count,
            (unsigned long)info.update_busy_reject_count,
            (unsigned long)info.start_dma_ch3_fail_count,
            (unsigned long)info.start_dma_ch4_fail_count,
            (unsigned long)info.pulse_done_ch3_count,
            (unsigned long)info.pulse_done_ch4_count,
            (unsigned long)info.dma_irq_ch3_count,
            (unsigned long)info.dma_irq_ch4_count,
            (unsigned)info.last_raw_ch1,
            (unsigned)info.last_raw_ch2);
        Debug_Printf("ESC DBG2 half3=%lu half4=%lu terr=%lu recover=%lu\r\n",
            (unsigned long)info.pulse_half_ch3_count,
            (unsigned long)info.pulse_half_ch4_count,
            (unsigned long)info.tim_error_count,
            (unsigned long)info.stuck_recover_count);
        Debug_Printf("ESC power(uart)=%u elapsed=%lu ms\r\n",
            (unsigned)s_power_from_uart,
            (unsigned long)(HAL_GetTick() - s_esc_arm_start_ms));
    }

    Debug_UART_Process();
    Debug_Process();
}
