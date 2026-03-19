#include "Driver/RC/RC.h"
#include "main.h"
#include <string.h>

#define SBUS_FRAME_LEN         25U
#define SBUS_FRAME_HEADER      0x0FU
#define SBUS_FRAME_FOOTER      0x00U
#define SBUS_TIMEOUT_MS        30U
#define RC_RAW_MIN             192U
#define RC_RAW_MID             992U
#define RC_RAW_MAX             1792U

static volatile RC_Signal_t s_rc_signal;
static uint8_t s_rx_buf[SBUS_FRAME_LEN];
static uint8_t s_rx_idx;

static int16_t RC_MapRawToNorm(int32_t raw)
{
    int32_t v;

    if (raw <= (int32_t)RC_RAW_MID) {
        v = (raw - (int32_t)RC_RAW_MID) * 1000 / ((int32_t)RC_RAW_MID - (int32_t)RC_RAW_MIN);
    } else {
        v = (raw - (int32_t)RC_RAW_MID) * 1000 / ((int32_t)RC_RAW_MAX - (int32_t)RC_RAW_MID);
    }

    if (v < -1000) v = -1000;
    if (v > 1000)  v = 1000;
    return (int16_t)v;
}

static void RC_DecodeFrame(const uint8_t *buf)
{
    uint16_t raw[16];

    if ((buf[0] != SBUS_FRAME_HEADER) || (buf[24] != SBUS_FRAME_FOOTER)) {
        return;
    }

    raw[0]  = (uint16_t)((buf[1]       | (buf[2]  << 8U))                         & 0x07FFU);
    raw[1]  = (uint16_t)(((buf[2]>>3U) | (buf[3]  << 5U))                         & 0x07FFU);
    raw[2]  = (uint16_t)(((buf[3]>>6U) | (buf[4]  << 2U) | (buf[5]  << 10U))      & 0x07FFU);
    raw[3]  = (uint16_t)(((buf[5]>>1U) | (buf[6]  << 7U))                         & 0x07FFU);
    raw[4]  = (uint16_t)(((buf[6]>>4U) | (buf[7]  << 4U))                         & 0x07FFU);
    raw[5]  = (uint16_t)(((buf[7]>>7U) | (buf[8]  << 1U) | (buf[9]  << 9U))       & 0x07FFU);
    raw[6]  = (uint16_t)(((buf[9]>>2U) | (buf[10] << 6U))                         & 0x07FFU);
    raw[7]  = (uint16_t)(((buf[10]>>5U)| (buf[11] << 3U))                         & 0x07FFU);
    raw[8]  = (uint16_t)((buf[12]      | (buf[13] << 8U))                         & 0x07FFU);
    raw[9]  = (uint16_t)(((buf[13]>>3U)| (buf[14] << 5U))                         & 0x07FFU);
    raw[10] = (uint16_t)(((buf[14]>>6U)| (buf[15] << 2U) | (buf[16] << 10U))      & 0x07FFU);
    raw[11] = (uint16_t)(((buf[16]>>1U)| (buf[17] << 7U))                         & 0x07FFU);
    raw[12] = (uint16_t)(((buf[17]>>4U)| (buf[18] << 4U))                         & 0x07FFU);
    raw[13] = (uint16_t)(((buf[18]>>7U)| (buf[19] << 1U) | (buf[20] << 9U))       & 0x07FFU);
    raw[14] = (uint16_t)(((buf[20]>>2U)| (buf[21] << 6U))                         & 0x07FFU);
    raw[15] = (uint16_t)(((buf[21]>>5U)| (buf[22] << 3U))                         & 0x07FFU);

    for (uint32_t i = 0U; i < 16U; i++) {
        s_rc_signal.ch[i] = RC_MapRawToNorm((int32_t)raw[i]);
    }

    s_rc_signal.ch17       = ((buf[23] & 0x80U) != 0U) ? 1U : 0U;
    s_rc_signal.ch18       = ((buf[23] & 0x40U) != 0U) ? 1U : 0U;
    s_rc_signal.frame_lost = ((buf[23] & 0x20U) != 0U) ? 1U : 0U;
    s_rc_signal.failsafe   = ((buf[23] & 0x10U) != 0U) ? 1U : 0U;
    s_rc_signal.is_online  = 1U;
    s_rc_signal.last_update_ms = HAL_GetTick();
    s_rc_signal.frame_count++;
}

void RC_Init(void)
{
    memset((void *)&s_rc_signal, 0, sizeof(s_rc_signal));
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_idx = 0U;

    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_PE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
}

void RC_Process(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_rc_signal.last_update_ms) > SBUS_TIMEOUT_MS) {
        s_rc_signal.is_online = 0U;
    }
}

const RC_Signal_t *RC_GetSignal(void)
{
    return (const RC_Signal_t *)&s_rc_signal;
}

void RC_UART_IRQHandler(void)
{
    uint32_t isr = huart6.Instance->ISR;

    if ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        huart6.Instance->ICR = (USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF);
        (void)huart6.Instance->RDR;
        s_rx_idx = 0U;
        return;
    }

    if ((isr & USART_ISR_RXNE_RXFNE) != 0U) {
        uint8_t b = (uint8_t)(huart6.Instance->RDR & 0xFFU);

        if ((s_rx_idx == 0U) && (b != SBUS_FRAME_HEADER)) {
            return;
        }

        s_rx_buf[s_rx_idx++] = b;
        if (s_rx_idx >= SBUS_FRAME_LEN) {
            RC_DecodeFrame(s_rx_buf);
            s_rx_idx = 0U;
        }
    }
}
