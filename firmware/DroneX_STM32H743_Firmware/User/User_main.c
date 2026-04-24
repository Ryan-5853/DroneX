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
#include "Algorithm/Att_Ctrl_PID/Att_Ctrl_PID.h"
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
#include <string.h>

/* 陀螺仪1 句柄与配置 */
static IMU_Handle_t     s_imu1_handle;
static ICM42688_SPI_Config_t s_imu1_cfg;
/* 陀螺仪2 句柄与配置 */
static IMU_Handle_t     s_imu2_handle;
static ICM42688_SPI_Config_t s_imu2_cfg;
static float s_gimbol_cmd0 = 0;
static float s_gimbol_cmd1 = 0;
static uint32_t s_esc_print_ms = 0;
static uint32_t s_imu_last_cycles = 0U;
static uint8_t s_att_ctrl_was_run = 0U;
static uint8_t s_att_ctrl_snapshot_valid = 0U;
static float s_att_ctrl_roll_sp = 0.0f;
static float s_att_ctrl_pitch_sp = 0.0f;
static float s_att_ctrl_roll = 0.0f;
static float s_att_ctrl_pitch = 0.0f;
static float s_att_ctrl_out_x = 0.0f;
static float s_att_ctrl_out_y = 0.0f;


#define PI 3.14159265358979323846f
/* 用户主循环调试输出开关 */
#define USER_DBG_PRINT_RC_NORM       0U
#define USER_DBG_PRINT_SYS_STATE     0U
#define USER_DBG_PRINT_IMU_ATT       0U
#define USER_DBG_PRINT_IMU_BODY      0U
#define USER_DBG_PRINT_CTRL_VOFA     1U
#define USER_DBG_PRINT_PERIOD_MS    100U
#define USER_DBG_CTRL_VOFA_PERIOD_MS 20U
#define USER_ATT_CTRL_MAX_ROLL_RAD   (20.0f*PI/180.0f) /* 20 deg */
#define USER_ATT_CTRL_MAX_PITCH_RAD  (20.0f*PI/180.0f) /* 20 deg */

static const Att_Ctrl_PID_Config_t s_att_ctrl_cfg = {
    .gains_roll = { 1.0f, 0.0f, 10.0f },
    .gains_pitch = { 1.0f, 0.0f, 10.0f },

    .roll_out_min = -1.0f,
    .roll_out_max = 1.0f,
    .pitch_out_min = -1.0f,
    .pitch_out_max = 1.0f,

    .enable_roll_i_limit = 0U,
    .roll_i_min = 0.0f,
    .roll_i_max = 0.0f,
    .enable_pitch_i_limit = 0U,
    .pitch_i_min = 0.0f,
    .pitch_i_max = 0.0f,

    .servo_min_norm = -1.0f,
    .servo_max_norm = 1.0f,

    .roll_servo_gain = 1.0f,
    .pitch_servo_gain = 1.0f,
    .servo_trim_norm = { 0.0f, 0.0f },
};

/* 动力转发任务：由 System 层根据状态决定是否/如何下发动力命令。 */
static void System_Power_Task(void)
{
    System_Manager_PowerControlTask();
}

static float Rc_ToNorm(int16_t rc_norm)
{
    float v = (float)rc_norm * 0.001f;

    if (v < -1.0f) {
        return -1.0f;
    }
    if (v > 1.0f) {
        return 1.0f;
    }
    return v;
}

static uint32_t User_GetMonotonicUs(void)
{
    static uint8_t inited = 0U;
    static uint32_t last_cycles = 0U;
    static uint32_t time_us = 0U;
    uint32_t now_cycles = Timing_GetCycles();

    if (inited == 0U) {
        last_cycles = now_cycles;
        inited = 1U;
        return time_us;
    }

    time_us += Timing_CyclesToUs(now_cycles - last_cycles);
    last_cycles = now_cycles;
    return time_us;
}

static void Vector_Gimbal_ManualTask(const RC_Signal_t *rc)
{
    if (rc == NULL) {
        return;
    }

    s_gimbol_cmd0 = Rc_ToNorm(rc->ch[0]);
    s_gimbol_cmd1 = Rc_ToNorm(rc->ch[1]);
    (void)Vector_Gimbal_SetNormalized(VECTOR_GIMBAL_AXIS_X, s_gimbol_cmd0);
    (void)Vector_Gimbal_SetNormalized(VECTOR_GIMBAL_AXIS_Y, s_gimbol_cmd1);
}

static void Vector_Gimbal_AutoTask(const RC_Signal_t *rc)
{
    float roll;
    float pitch;
    float out_x;
    float out_y;
    Att_Ctrl_PID_In_t in;
    Att_Ctrl_PID_Out_t out;
    int rc_pid;

    if ((rc == NULL) || (rc->is_online == 0U) || !Attitude_Est_IsReady()) {
        (void)Vector_Gimbal_SetNormalized(VECTOR_GIMBAL_AXIS_X, 0.0f);
        (void)Vector_Gimbal_SetNormalized(VECTOR_GIMBAL_AXIS_Y, 0.0f);
        s_att_ctrl_snapshot_valid = 0U;
        Att_Ctrl_PID_Reset();
        return;
    }

    Attitude_Est_GetEuler(&roll, &pitch, NULL);

    memset(&in, 0, sizeof(in));
    in.roll_rad = roll;
    in.pitch_rad = pitch;
    in.roll_sp_rad = Rc_ToNorm(rc->ch[0]) * USER_ATT_CTRL_MAX_ROLL_RAD;
    in.pitch_sp_rad = -Rc_ToNorm(rc->ch[1]) * USER_ATT_CTRL_MAX_PITCH_RAD;
    in.time_us = User_GetMonotonicUs();

    rc_pid = Att_Ctrl_PID_Update(&in, &out);
    if (rc_pid == ATT_CTRL_PID_ERR_TIME) {
        Att_Ctrl_PID_Reset();
        rc_pid = Att_Ctrl_PID_Update(&in, &out);
    }
    if (rc_pid == ATT_CTRL_PID_OK) {
        out_x = out.servo_norm[0];
        out_y = -out.servo_norm[1];
        (void)Vector_Gimbal_SetNormalized(VECTOR_GIMBAL_AXIS_X, out_x);
        (void)Vector_Gimbal_SetNormalized(VECTOR_GIMBAL_AXIS_Y, out_y);

        s_att_ctrl_roll_sp = in.roll_sp_rad;
        s_att_ctrl_pitch_sp = in.pitch_sp_rad;
        s_att_ctrl_roll = roll;
        s_att_ctrl_pitch = pitch;
        s_att_ctrl_out_x = out_x;
        s_att_ctrl_out_y = out_y;
        s_att_ctrl_snapshot_valid = 1U;
    } else {
        s_att_ctrl_snapshot_valid = 0U;
    }
}

/** Debug 层：Driver 拆包后的指令包回调，交由 Cmd 解析执行 */
static void Debug_CtrlVofaTask(uint32_t now_ms)
{
    static uint32_t s_last_vofa_ms = 0U;

    if (USER_DBG_PRINT_CTRL_VOFA == 0U) {
        return;
    }
    if ((now_ms - s_last_vofa_ms) < USER_DBG_CTRL_VOFA_PERIOD_MS) {
        return;
    }
    if ((s_att_ctrl_snapshot_valid == 0U) || (Debug_IsQueueEmpty() == 0)) {
        return;
    }

    s_last_vofa_ms = now_ms;
    (void)Debug_Printf("%.6f,%.6f,%.6f\n",
                       (double)s_att_ctrl_pitch_sp,
                       (double)s_att_ctrl_pitch,
                       (double)s_att_ctrl_out_y);
}

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
    uint32_t now_cycles = Timing_GetCycles();
    float dt_s = 0.005f;

    if (s_imu_last_cycles != 0U) {
        uint32_t dt_us = Timing_CyclesToUs(now_cycles - s_imu_last_cycles);
        if ((dt_us >= 1000U) && (dt_us <= 20000U)) {
            dt_s = (float)dt_us * 1.0e-6f;
        }
    }
    s_imu_last_cycles = now_cycles;

    IMU_Data_t data1, data2;
    int ok1 = (IMU_Read(&s_imu1_handle, &data1) == IMU_OK);
    int ok2 = (IMU_Read(&s_imu2_handle, &data2) == IMU_OK);

    Attitude_OnImuSample(ok1 ? &data1 : NULL, ok1, ok2 ? &data2 : NULL, ok2);

    if (ok1 && ok2)
        Attitude_Est_Update(&data1, &data2, dt_s);
    else if (ok1)
        Attitude_Est_Update(&data1, NULL, dt_s);
    else if (ok2)
        Attitude_Est_Update(NULL, &data2, dt_s);

    if (Attitude_Est_IsReady()) {
        float roll, pitch, yaw;
        Attitude_Est_GetEuler(&roll, &pitch, &yaw);
        // Debug_Printf("EUL %.2f %.2f %.2f dt=%.4f\r\n",
        //     (double)(roll * 57.29577951308232f),
        //     (double)(pitch * 57.29577951308232f),
        //     (double)(yaw * 57.29577951308232f),
        //     (double)dt_s);
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
    Attitude_Est_Init();
    Attitude_Init();
    (void)Att_Ctrl_PID_Init(&s_att_ctrl_cfg);

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
        PIT_Start(PIT_CH0);

    /* 200Hz 姿态解算任务：PIT_CH1（含 imu_cali 占位实现） */
    pit_cfg.period_us = 5000;
    pit_cfg.priority  = 2;
    pit_cfg.callback  = Attitude_Task;
    if (PIT_Init(PIT_CH1, &pit_cfg) == PIT_OK)
        // PIT_Start(PIT_CH1);

    /* DShot300 发送节拍：250us ≈ 4kHz（与常见飞控一致，高于原 500Hz） */
    pit_cfg.period_us = 250;
    pit_cfg.priority  = 3;
    pit_cfg.callback  = System_Power_Task;
    if (PIT_Init(PIT_CH2, &pit_cfg) == PIT_OK)
        PIT_Start(PIT_CH2);

    System_Manager_Init();
}

/**
 * @brief 用户主循环体
 */
void User_Main_Loop(void)
{
    const RC_Signal_t *rc = RC_GetSignal();
    uint32_t now = HAL_GetTick();
    if ((now - s_esc_print_ms) >= USER_DBG_PRINT_PERIOD_MS) {
        s_esc_print_ms = now;
        if (USER_DBG_PRINT_RC_NORM != 0U) {
            /* 输出遥控器归一化通道（-1000..1000）和关键状态位 */
            Debug_Printf(
                "RC norm ch1=%d ch2=%d ch3=%d ch4=%d ch5=%d ch6=%d ch7=%d ch8=%d ch9=%d ch10=%d ch11=%d ch12=%d ch13=%d ch14=%d ch15=%d ch16=%d sw17=%u sw18=%u online=%u lost=%u fs=%u\r\n",
                (int)rc->ch[0],
                (int)rc->ch[1],
                (int)rc->ch[2],
                (int)rc->ch[3],
                (int)rc->ch[4],
                (int)rc->ch[5],
                (int)rc->ch[6],
                (int)rc->ch[7],
                (int)rc->ch[8],
                (int)rc->ch[9],
                (int)rc->ch[10],
                (int)rc->ch[11],
                (int)rc->ch[12],
                (int)rc->ch[13],
                (int)rc->ch[14],
                (int)rc->ch[15],
                (unsigned)rc->ch17,
                (unsigned)rc->ch18,
                (unsigned)rc->is_online,
                (unsigned)rc->frame_lost,
                (unsigned)rc->failsafe);
        }
        if (USER_DBG_PRINT_SYS_STATE != 0U) {
            const System_State_t *sys = System_Manager_GetState();
            Debug_Printf(
                "SYS mode=%d fault=0x%08lx armed=%u pwr_act=%u pwr_out=%u reboot=%u safety=%u\r\n",
                (int)sys->mode,
                (unsigned long)sys->fault_flags,
                (unsigned)sys->armed,
                (unsigned)sys->power_unit_activated,
                (unsigned)sys->power_output_enabled,
                (unsigned)sys->reboot_required,
                (unsigned)sys->safety_triggered);
        }
        if (USER_DBG_PRINT_IMU_ATT != 0U) {
            if (Attitude_Est_IsReady()) {
                float roll, pitch, yaw;
                Attitude_Est_GetEuler(&roll, &pitch, &yaw);

                Debug_Printf(
                    "ATT deg roll=%.2f pitch=%.2f yaw=%.2f\r\n",
                    (double)(roll * 57.29577951308232f),
                    (double)(pitch * 57.29577951308232f),
                    (double)(yaw * 57.29577951308232f));
            } else {
                Debug_Printf("ATT not-ready\r\n");
            }
        }
        if (USER_DBG_PRINT_IMU_BODY != 0U) {
            IMU_Data_t imu1_body;
            IMU_Data_t imu2_body;
            int imu1_ready;
            int imu2_ready;

            __disable_irq();
            imu1_ready = (Attitude_Est_GetImuBodyData(ATTITUDE_EST_IMU_1, &imu1_body) == ATTITUDE_EST_OK);
            imu2_ready = (Attitude_Est_GetImuBodyData(ATTITUDE_EST_IMU_2, &imu2_body) == ATTITUDE_EST_OK);
            __enable_irq();

            if (imu1_ready != 0) {
                Debug_Printf("IMU1 body gyro=[%.5f %.5f %.5f] acc=[%.5f %.5f %.5f]\r\n",
                            (double)imu1_body.gyro[0],
                            (double)imu1_body.gyro[1],
                            (double)imu1_body.gyro[2],
                            (double)imu1_body.accel[0],
                            (double)imu1_body.accel[1],
                            (double)imu1_body.accel[2]);
            } else {
                Debug_Printf("IMU1 body not-ready\r\n");
            }

            if (imu2_ready != 0) {
                Debug_Printf("IMU2 body gyro=[%.5f %.5f %.5f] acc=[%.5f %.5f %.5f]\r\n",
                            (double)imu2_body.gyro[0],
                            (double)imu2_body.gyro[1],
                            (double)imu2_body.gyro[2],
                            (double)imu2_body.accel[0],
                            (double)imu2_body.accel[1],
                            (double)imu2_body.accel[2]);
            } else {
                Debug_Printf("IMU2 body not-ready\r\n");
            }
        }
    }

    if (System_Manager_GetState()->mode == SYS_MODE_RUN) {
        if (s_att_ctrl_was_run == 0U) {
            Att_Ctrl_PID_Reset();
            s_att_ctrl_was_run = 1U;
        }
        Vector_Gimbal_AutoTask(rc);
    } else {
        if (s_att_ctrl_was_run != 0U) {
            Att_Ctrl_PID_Reset();
            s_att_ctrl_was_run = 0U;
        }
        s_att_ctrl_snapshot_valid = 0U;
        Vector_Gimbal_ManualTask(rc);
    }

    Debug_CtrlVofaTask(HAL_GetTick());

    Debug_UART_Process();
    RC_Process();
    System_Manager_Tick1ms();
    Debug_Process();
}
