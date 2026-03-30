#ifndef __ESC_H__
#define __ESC_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 1=主循环周期性打印 DShot 发送统计（约 100ms 一条，勿在中断内打开大量日志） */
#ifndef ESC_LOG_DSHOT
#define ESC_LOG_DSHOT  1
#endif

/* 1=帧内请求 DShot 遥测位，接收 eRPM 等（双向 DShot） */
#ifndef ESC_BIDIR_DSHOT
#define ESC_BIDIR_DSHOT  0
#endif

/*
 * 双向 DShot 时建议 1：每帧只让一路电机请求遥测并进入 RX。
 * 两路同时 RX + 高节拍易导致 TIM2 长时间 BUSY、stuck 增加、电机无响应。
 * 设为 0 则两路同时请求（仅建议调试用）。
 */
#ifndef ESC_BIDIR_TELEM_ALTERNATE
#define ESC_BIDIR_TELEM_ALTERNATE  1
#endif

/* 每 N 帧才允许发一次遥测请求（参考 BF 降低 RX 占用）；1=每帧可请求（易堵死） */
#ifndef ESC_BIDIR_TELEM_EVERY_N
#define ESC_BIDIR_TELEM_EVERY_N  8U
#endif

/* 机械转速换算：与 Betaflight 等一致 rpm ≈ eRPM_raw * 200 / 电机磁极数（整圈磁极数，如 14） */
#ifndef ESC_MOTOR_POLE_COUNT
#define ESC_MOTOR_POLE_COUNT  14U
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESC_OK = 0,
    ESC_ERR_PARAM,
    ESC_ERR_HAL,
    ESC_ERR_BUSY,
    ESC_ERR_NOT_INIT,
} ESC_Status_t;

typedef enum {
    ESC_CH1 = 0, /* TIM2_CH3 -> PA2 */
    ESC_CH2 = 1, /* TIM2_CH4 -> PA3 */
    ESC_CH_COUNT
} ESC_Channel_t;

typedef struct {
    uint32_t msp_init_count;
    uint32_t update_req_count;
    uint32_t update_ok_count;
    uint32_t update_busy_reject_count;
    uint32_t start_dma_ch3_fail_count;
    uint32_t start_dma_ch4_fail_count;
    uint32_t pulse_done_ch3_count;
    uint32_t pulse_done_ch4_count;
    uint32_t dma_irq_ch3_count;
    uint32_t dma_irq_ch4_count;
    uint32_t pulse_half_ch3_count;
    uint32_t pulse_half_ch4_count;
    uint32_t tim_error_count;
    uint32_t stuck_recover_count;
    uint16_t last_raw_ch1;
    uint16_t last_raw_ch2;
    uint8_t  dma_busy;
    uint8_t  initialized;
} ESC_DebugInfo_t;

typedef struct {
    uint16_t raw_word;          /* 解析得到的 16-bit 回传字 */
    uint16_t data_12bit;        /* raw_word[15:4] */
    uint8_t  crc;               /* raw_word[3:0] */
    uint8_t  crc_ok;            /* CRC 是否通过 */
    uint8_t  valid;             /* 本次回传是否有效 */
    uint8_t  updated;           /* 自上次读取后是否有新数据 */
    uint8_t  edge_count;        /* 本帧捕获到的边沿数量 */
    uint32_t timestamp_ms;      /* HAL_GetTick 时间戳 */
} ESC_Telemetry_t;

/* 由遥测 12 位 eRPM 粗算机械转速 (RPM)；无效或非 eRPM 类型时返回 0 */
static inline uint32_t ESC_Telemetry_ErpmToMechanicalRpm(const ESC_Telemetry_t *t)
{
    if (t == NULL || !t->valid) {
        return 0U;
    }
    uint32_t poles = (uint32_t)ESC_MOTOR_POLE_COUNT;
    if (poles == 0U) {
        poles = 14U;
    }
    return (uint32_t)t->data_12bit * 200U / poles;
}

typedef struct {
    int16_t throttle;           /* 期望油门，建议范围 0..1000 */
    uint8_t request_telemetry;  /* 1=请求回传 */
} ESC_Command_t;

typedef struct {
    uint8_t initialized;
    uint8_t last_update_ok;
    uint32_t tick_count;
    uint32_t update_ok_count;
    uint32_t update_err_count;
    uint32_t rx_ok_count;
    uint32_t rx_err_count;
    ESC_Telemetry_t last_telemetry;
} ESC_ServiceState_t;

/*
 * 当前按非反转电调配置输出标准 DShot300：
 *  0       : stop
 *  1..1000 : DShot 48..2047（正向全范围）
 *  负数输入会被钳位为 0。
 */
ESC_Status_t ESC_Init(void);
ESC_Status_t ESC_SetThrottleBidirectional(ESC_Channel_t ch, int16_t throttle);
ESC_Status_t ESC_SetThrottleBidirectionalEx(ESC_Channel_t ch, int16_t throttle, bool request_telemetry);
ESC_Status_t ESC_SetInputMapped(ESC_Channel_t ch, uint16_t input_value);
ESC_Status_t ESC_SetThrottleRaw(ESC_Channel_t ch, uint16_t dshot_value, bool request_telemetry);
ESC_Status_t ESC_Update(void);
bool ESC_IsBusy(void);
void ESC_GetDebugInfo(ESC_DebugInfo_t *info);
bool ESC_GetTelemetry(ESC_Channel_t ch, ESC_Telemetry_t *out);
void ESC_RecoverIfStuck(uint32_t timeout_ms);
void ESC_TestPwm_StartOnPD15(void);
void ESC_TestPwm_SetPower(uint16_t power_0_1000);

/* 服务层封装：上层只写命令，周期调用 Tick 即可自动运行 */
void ESC_Service_Init(void);
ESC_Status_t ESC_Service_WriteCommand(ESC_Channel_t ch, int16_t throttle, bool request_telemetry);
void ESC_Service_Tick(void);
bool ESC_Service_ReadState(ESC_Channel_t ch, ESC_ServiceState_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __ESC_H__ */

