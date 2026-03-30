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
#include "Driver/RC/RC.h"
#include "Driver/Vector_Gimbal/Vector_Gimbal.h"
#include "System/System_Manager.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* 陀螺仪1 句柄与配置 */
static IMU_Handle_t     s_imu1_handle;
static ICM42688_SPI_Config_t s_imu1_cfg;
/* 陀螺仪2 句柄与配置 */
static IMU_Handle_t     s_imu2_handle;
static ICM42688_SPI_Config_t s_imu2_cfg;
static int16_t s_gimbol_cmd0 = 0;
static int16_t s_gimbol_cmd1 = 0;
/* 共轴反桨：两路电机共用同一 RC 油门通道（当前为 ch[2]，即遥控显示 CH3） */
static volatile int16_t s_esc_coax_throttle = 0;
static uint32_t s_esc_print_ms = 0;
#if ESC_LOG_DSHOT
static uint32_t s_dshot_log_last_ok;
static uint32_t s_dshot_log_last_rej;
static uint8_t  s_dshot_log_inited;
#if ESC_BIDIR_DSHOT
static uint32_t s_bidir_last_rx_ok[ESC_CH_COUNT];
static uint32_t s_bidir_last_rx_err[ESC_CH_COUNT];
#endif
#endif

/* RC 通道归一化值 -1000..1000 映射到 ESC 油门 0..300（30%） */
static int16_t MapRcToEsc30Percent(int16_t rc_norm)
{
    int32_t t = ((int32_t)rc_norm ) * 200 / 1000;
    if (t < 0) {
        t = 0;
    } else if (t > 200) {
        t = 200;
    }
    return (int16_t)t;
}

/* ESC：同一油门值写入 CH1/CH2，映射到 30% 上限（PIT 周期见下方，与 DShot 标称帧率一致） */
static void ESC_DShot_Task(void)
{
    const RC_Signal_t *rc = RC_GetSignal();
    s_esc_coax_throttle = MapRcToEsc30Percent(rc->ch[2]);
#if ESC_BIDIR_DSHOT
    const bool esc_req_telemetry = true;
#else
    const bool esc_req_telemetry = false;
#endif
    (void)ESC_Service_WriteCommand(ESC_CH1, s_esc_coax_throttle, esc_req_telemetry);
    (void)ESC_Service_WriteCommand(ESC_CH2, s_esc_coax_throttle, esc_req_telemetry);
    ESC_Service_Tick();
}

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
    RC_Init();

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
        ESC_Service_Init();
        // (void)ESC_SetThrottleBidirectional(ESC_CH1, 0);
        // (void)ESC_SetThrottleBidirectional(ESC_CH2, 0);
        // (void)ESC_Update();
        HAL_Delay(1000);
    }
    else {
        Debug_BlockingPrintf("ESC_Init FAILED\r\n");
    }
    

    /* 云台舵机：TIM4_CH3/CH4 -> PD14/PD15，约333Hz，接口 -10000..10000 */
    Vector_Gimbal_Init();

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

    /* DShot300 发送节拍：250us ≈ 4kHz（与常见飞控一致，高于原 500Hz） */
    pit_cfg.period_us = 250;
    pit_cfg.priority  = 3;
    pit_cfg.callback  = ESC_DShot_Task;
    if (PIT_Init(PIT_CH2, &pit_cfg) == PIT_OK)
        PIT_Start(PIT_CH2);

    System_Manager_Init();
}

/**
 * @brief 用户主循环体
 */
void User_Main_Loop(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_esc_print_ms) >= 100U) {
        s_esc_print_ms = now;
        Debug_Printf("ESC coax both motors (ch3 map): %d\r\n",
                     (int)s_esc_coax_throttle);
#if ESC_LOG_DSHOT
        {
            ESC_DebugInfo_t di;
            ESC_GetDebugInfo(&di);
            if (!s_dshot_log_inited) {
                s_dshot_log_inited   = 1U;
                s_dshot_log_last_ok  = di.update_ok_count;
                s_dshot_log_last_rej = di.update_busy_reject_count;
#if ESC_BIDIR_DSHOT
                for (uint8_t chi = 0U; chi < ESC_CH_COUNT; chi++) {
                    ESC_ServiceState_t sst;
                    if (ESC_Service_ReadState((ESC_Channel_t)chi, &sst)) {
                        s_bidir_last_rx_ok[chi]  = sst.rx_ok_count;
                        s_bidir_last_rx_err[chi] = sst.rx_err_count;
                    }
                }
#endif
            } else {
                uint32_t d_ok  = di.update_ok_count - s_dshot_log_last_ok;
                uint32_t d_rej = di.update_busy_reject_count - s_dshot_log_last_rej;
                s_dshot_log_last_ok  = di.update_ok_count;
                s_dshot_log_last_rej = di.update_busy_reject_count;
                Debug_Printf(
                    "DSHOT300 tx: +100ms ok=%lu busy_rej=%lu | cum pulse ch3/4=%lu/%lu stuck=%lu | last_raw=%u/%u dma=0x%02x\r\n",
                    (unsigned long)d_ok,
                    (unsigned long)d_rej,
                    (unsigned long)di.pulse_done_ch3_count,
                    (unsigned long)di.pulse_done_ch4_count,
                    (unsigned long)di.stuck_recover_count,
                    (unsigned)di.last_raw_ch1,
                    (unsigned)di.last_raw_ch2,
                    (unsigned)di.dma_busy);
#if ESC_BIDIR_DSHOT
                {
                    ESC_ServiceState_t st1, st2;
                    (void)ESC_Service_ReadState(ESC_CH1, &st1);
                    (void)ESC_Service_ReadState(ESC_CH2, &st2);
                    uint32_t d_rx_ok1  = st1.rx_ok_count - s_bidir_last_rx_ok[0];
                    uint32_t d_rx_ok2  = st2.rx_ok_count - s_bidir_last_rx_ok[1];
                    uint32_t d_rx_er1  = st1.rx_err_count - s_bidir_last_rx_err[0];
                    uint32_t d_rx_er2  = st2.rx_err_count - s_bidir_last_rx_err[1];
                    s_bidir_last_rx_ok[0]  = st1.rx_ok_count;
                    s_bidir_last_rx_ok[1]  = st2.rx_ok_count;
                    s_bidir_last_rx_err[0] = st1.rx_err_count;
                    s_bidir_last_rx_err[1] = st2.rx_err_count;
                    uint32_t rpm1 = ESC_Telemetry_ErpmToMechanicalRpm(&st1.last_telemetry);
                    uint32_t rpm2 = ESC_Telemetry_ErpmToMechanicalRpm(&st2.last_telemetry);
                    Debug_Printf(
                        "DSHOT bidir: rpm=%lu/%lu erpm12=%u/%u | +100ms rx_ok=%lu/%lu rx_fail=%lu/%lu | cum ok=%lu/%lu fail=%lu/%lu\r\n",
                        (unsigned long)rpm1,
                        (unsigned long)rpm2,
                        (unsigned)st1.last_telemetry.data_12bit,
                        (unsigned)st2.last_telemetry.data_12bit,
                        (unsigned long)d_rx_ok1,
                        (unsigned long)d_rx_ok2,
                        (unsigned long)d_rx_er1,
                        (unsigned long)d_rx_er2,
                        (unsigned long)st1.rx_ok_count,
                        (unsigned long)st2.rx_ok_count,
                        (unsigned long)st1.rx_err_count,
                        (unsigned long)st2.rx_err_count);
                }
#endif
            }
        }
#endif
    }

    const RC_Signal_t *rc = RC_GetSignal();
    s_gimbol_cmd0 = (int16_t)((int32_t)rc->ch[0] * 3);
    s_gimbol_cmd1 = (int16_t)((int32_t)rc->ch[1] * 3);
    (void)Vector_Gimbal_Set(VECTOR_GIMBAL_AXIS_0, s_gimbol_cmd0);
    (void)Vector_Gimbal_Set(VECTOR_GIMBAL_AXIS_1, s_gimbol_cmd1);

    Debug_UART_Process();
    RC_Process();
    System_Manager_Tick1ms();
    Debug_Process();
}
