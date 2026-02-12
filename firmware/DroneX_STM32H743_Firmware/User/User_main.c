/**
 * @file User_main.c
 * @brief 用户主程序：初始化与主循环逻辑，由 main.c 调用。
 */

#include "User_main.h"
#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "Debug/Cmd/Cmd.h"
#include "Application/Attitude/Attitude.h"
#include "Driver/IMU/IMU_interface.h"
#include "Driver/IMU/ICM42688.h"
#include "Driver/PIT/PIT.h"
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

/** 200Hz 定时任务：读取 IMU 并输出，高优先级 */
static void IMU_200Hz_Task(void)
{
    IMU_Data_t data1, data2;
    if (IMU_Read(&s_imu1_handle, &data1) == IMU_OK) {
        float ax_g = data1.accel[0] / 9.80665f;
        float ay_g = data1.accel[1] / 9.80665f;
        float az_g = data1.accel[2] / 9.80665f;
        Debug_Printf("%.3f,%.3f,%.3f\n",
            (double)data1.gyro[0], (double)data1.gyro[1], (double)data1.gyro[2]);
    }
    // if (IMU_Read(&s_imu2_handle, &data2) == IMU_OK) {
    //     float ax_g = data2.accel[0] / 9.80665f;
    //     float ay_g = data2.accel[1] / 9.80665f;
    //     float az_g = data2.accel[2] / 9.80665f;
    //     Debug_Printf("IMU2 GYRO %.3f %.3f %.3f rad/s | ACC %.2f %.2f %.2f g | T %.1f C\r\n",
    //         (double)data2.gyro[0], (double)data2.gyro[1], (double)data2.gyro[2],
    //         (double)ax_g, (double)ay_g, (double)az_g, (double)data2.temperature);
    // }
}

/**
 * @brief 用户初始化
 */
void User_Main_Init(void)
{
    Debug_Init(Debug_Transport_Send);
    Debug_Printf("DroneX User Main ready\r\n");

    Cmd_Init();
    Cmd_RegisterBuiltins();
    Debug_UART_SetLineCallback(Debug_OnLine);
    Debug_UART_Init();

    /* 陀螺仪1：SPI1，CS=PE2 */
    s_imu1_cfg.hspi   = &hspi1;
    s_imu1_cfg.cs_port = GPIOE;
    s_imu1_cfg.cs_pin  = GPIO_PIN_2;
    IMU_Interface_Init(&s_imu1_handle, IMU_TYPE_ICM42688, &s_imu1_cfg);

    if (IMU_Init(&s_imu1_handle) != IMU_OK) {
        Debug_Printf("IMU1 init failed\r\n");
    } else {
        Debug_Printf("IMU1 ICM42688 OK\r\n");
    }

    /* 陀螺仪2：SPI4，CS=PE11 */
    s_imu2_cfg.hspi    = &hspi4;
    s_imu2_cfg.cs_port = GPIOE;
    s_imu2_cfg.cs_pin  = GPIO_PIN_11;
    IMU_Interface_Init(&s_imu2_handle, IMU_TYPE_ICM42688, &s_imu2_cfg);

    if (IMU_Init(&s_imu2_handle) != IMU_OK) {
        Debug_Printf("IMU2 init failed\r\n");
    } else {
        Debug_Printf("IMU2 ICM42688 OK\r\n");
    }

    /* 200Hz IMU 任务：PIT_CH0，周期 5000us，NVIC 优先级 1（高优先级） */
    PIT_Config_t pit_cfg = {
        .period_us = 5000,   /* 200Hz = 5ms */
        .priority  = 1,
        .callback  = IMU_200Hz_Task,
    };
    // if (PIT_Init(PIT_CH0, &pit_cfg) == PIT_OK) {
    //     PIT_Start(PIT_CH0);
    //     Debug_Printf("PIT IMU 200Hz task started\r\n");
    // } else {
    //     Debug_Printf("PIT IMU task init failed\r\n");
    // }

    /* 100Hz 姿态解算任务：PIT_CH1（含 imu_cali 占位实现） */
    pit_cfg.period_us = 10000;
    pit_cfg.priority  = 2;
    pit_cfg.callback  = Attitude_Task;
    if (PIT_Init(PIT_CH1, &pit_cfg) == PIT_OK) {
        PIT_Start(PIT_CH1);
        Debug_Printf("PIT Attitude 100Hz task started\r\n");
    } else {
        Debug_Printf("PIT Attitude task init failed\r\n");
    }
}

/**
 * @brief 用户主循环体
 */
void User_Main_Loop(void)
{
    Debug_UART_Process();
    Debug_Process();
}
