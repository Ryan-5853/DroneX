/**
 * @file User_main.c
 * @brief 用户主程序：初始化与主循环逻辑，由 main.c 调用。
 */

#include "User_main.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "Debug/Cmd/Cmd.h"
#include "Application/Attitude/Attitude.h"
#include "Algorithm/Attitude_Est/Attitude_Est.h"
#include "Driver/IMU/IMU_interface.h"
#include "Driver/IMU/ICM42688.h"
#include "Driver/PIT/PIT.h"
#include "Driver/Timing/Timing.h"
#include "main.h"

/* 陀螺仪1 句柄与配置 */
static IMU_Handle_t     s_imu1_handle;
static ICM42688_SPI_Config_t s_imu1_cfg;
/* 陀螺仪2 句柄与配置 */
static IMU_Handle_t     s_imu2_handle;
static ICM42688_SPI_Config_t s_imu2_cfg;

/** Debug 层：Driver 拆包后的指令包回调，交由 Cmd 解析执行 */
static void Debug_OnLine(const char *buf)
{
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
    Attitude_Est_Init();

    /* 陀螺仪1：SPI1，CS=PE2 */
    s_imu1_cfg.hspi   = &hspi1;
    s_imu1_cfg.cs_port = GPIOE;
    s_imu1_cfg.cs_pin  = GPIO_PIN_2;
    IMU_Interface_Init(&s_imu1_handle, IMU_TYPE_ICM42688, &s_imu1_cfg);

    (void)IMU_Init(&s_imu1_handle);

    /* 陀螺仪2：SPI4，CS=PE11 */
    s_imu2_cfg.hspi    = &hspi4;
    s_imu2_cfg.cs_port = GPIOE;
    s_imu2_cfg.cs_pin  = GPIO_PIN_11;
    IMU_Interface_Init(&s_imu2_handle, IMU_TYPE_ICM42688, &s_imu2_cfg);

    (void)IMU_Init(&s_imu2_handle);

    /* 200Hz IMU 任务：PIT_CH0，周期 5000us，NVIC 优先级 1（高优先级） */
    PIT_Config_t pit_cfg = {
        .period_us = 5000,   /* 200Hz = 5ms */
        .priority  = 1,
        .callback  = IMU_500Hz_Task,
    };
    if (PIT_Init(PIT_CH0, &pit_cfg) == PIT_OK)
        PIT_Start(PIT_CH0);

    /* 200Hz 姿态解算任务：PIT_CH1（含 imu_cali 占位实现） */
    pit_cfg.period_us = 5000;
    pit_cfg.priority  = 2;
    pit_cfg.callback  = Attitude_Task;
    if (PIT_Init(PIT_CH1, &pit_cfg) == PIT_OK)
        PIT_Start(PIT_CH1);
}

/**
 * @brief 用户主循环体
 */
void User_Main_Loop(void)
{
    Debug_UART_Process();
    Debug_Process();
}
