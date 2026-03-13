#ifndef __ESC_H__
#define __ESC_H__

#include <stdbool.h>
#include <stdint.h>

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

/*
 * 当前按非反转电调配置输出标准 DShot300：
 *  0       : stop
 *  1..1000 : DShot 48..2047（正向全范围）
 *  负数输入会被钳位为 0。
 */
ESC_Status_t ESC_Init(void);
ESC_Status_t ESC_SetThrottleBidirectional(ESC_Channel_t ch, int16_t throttle);
ESC_Status_t ESC_SetInputMapped(ESC_Channel_t ch, uint16_t input_value);
ESC_Status_t ESC_SetThrottleRaw(ESC_Channel_t ch, uint16_t dshot_value, bool request_telemetry);
ESC_Status_t ESC_Update(void);
bool ESC_IsBusy(void);
void ESC_GetDebugInfo(ESC_DebugInfo_t *info);
void ESC_RecoverIfStuck(uint32_t timeout_ms);
void ESC_TestPwm_StartOnPD15(void);
void ESC_TestPwm_SetPower(uint16_t power_0_1000);

#ifdef __cplusplus
}
#endif

#endif /* __ESC_H__ */

