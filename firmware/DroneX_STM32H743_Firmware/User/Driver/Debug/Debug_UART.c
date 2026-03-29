/**
 * @file Debug_UART.c
 * @brief Driver 层：USART1 TX/RX，IT 非阻塞接收。
 *
 * TX：DMA 非阻塞发送。s_tx_buf 位于 AHB SRAM 0x30000000（D2 域 DMA 可访问），
 *     MPU Region1 已将该区域置为非 cache，消除 D-Cache 一致性问题。
 * RX：按 \r\n 拆包，通过回调将指令包传递给上层。使用静态缓冲避免栈覆盖。
 */

#include "Driver/Debug/Debug.h"
#include "Driver/Debug/Debug_UART.h"
#include "main.h"
#include <string.h>

#define DEBUG_UART_TX_BUF_SIZE  DEBUG_MSG_MAX_LEN
#define DEBUG_UART_RX_BUF_SIZE  256
#define DEBUG_UART_BLOCKING_TIMEOUT_MS  50U

/* DMA TX 缓冲区：AHB SRAM 0x30000000（D2 域 DMA 可访问）。MPU Region1 已置为非 cache。 */
__attribute__((at(0x30000000))) __attribute__((aligned(32)))
static uint8_t s_tx_buf[DEBUG_UART_TX_BUF_SIZE];

/* volatile 标志位：1=空闲可发送，0=DMA 忙。DMA 完成回调复位为 1 */
static volatile uint8_t s_dma_tx_idle = 1;

/* RX: 中断接收单字节，写入线性缓冲；遇 \r\n 置位 line_ready，主循环处理 */
static uint8_t s_rx_byte;
static char s_rx_buf[DEBUG_UART_RX_BUF_SIZE];
static char s_line_buf[DEBUG_UART_RX_BUF_SIZE];   /* 中断写入，主循环读出 */
static char s_line_copy[DEBUG_UART_RX_BUF_SIZE];  /* 主循环拷贝后交给回调，避免回调执行时被覆盖 */
static volatile uint16_t s_rx_len;
static volatile uint8_t s_line_ready;
static Debug_UART_LineCallback_t s_line_cb;

static void rx_restart(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/**
 * @brief 重写：传输层是否空闲。Debug_Process 先检查队列再检查此标志，都满足才出队、拷贝、启动 DMA。
 */
int Debug_Transport_IsReady(void)
{
    return s_dma_tx_idle ? 1 : 0;
    // return 1;
}

/* ---------------------------------------------------------------------------
 * 重写 Debug_Transport_BlockingSend：HAL 轮询发送，阻塞至全部发出。
 * 用于 DMA / 队列尚未初始化时的早期启动调试。
 * --------------------------------------------------------------------------- */
uint32_t Debug_Transport_BlockingSend(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) return 0;
    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, (uint16_t)len, DEBUG_UART_BLOCKING_TIMEOUT_MS) != HAL_OK)
        return 0;
    return len;
}

/* ---------------------------------------------------------------------------
 * 重写 Debug_Transport_Send：仅在 s_dma_tx_idle 时由 Debug_Process 调用。拷贝、启动 DMA 后置忙。
 * --------------------------------------------------------------------------- */
uint32_t Debug_Transport_Send(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0 || len > DEBUG_UART_TX_BUF_SIZE)
        return 0;

    if (!s_dma_tx_idle)
        return 0;

    memcpy(s_tx_buf, data, len);
    if (HAL_UART_Transmit_DMA(&huart1, s_tx_buf, (uint16_t)len) != HAL_OK)
        return 0;

    s_dma_tx_idle = 0;  /* 启动 DMA，置忙 */
    return len;
}

/* ---------------------------------------------------------------------------
 * UART TX DMA 完成：复位标志位，触发下一条发送
 * --------------------------------------------------------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    s_dma_tx_idle = 1;  /* DMA 结束，复位为空闲 */
    Debug_Process();    /* 立即尝试发送下一条 */
}

/* ---------------------------------------------------------------------------
 * UART RX 中断回调：收单字节，遇 \r\n 拆包
 * --------------------------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t b = s_rx_byte;
    if (b == '\r' || b == '\n') {
        if (s_rx_len > 0) {
            s_rx_buf[s_rx_len] = '\0';
            memcpy(s_line_buf, s_rx_buf, (size_t)s_rx_len + 1);
            s_line_ready = 1;
        }
        s_rx_len = 0;
    } else {
        if (s_rx_len < DEBUG_UART_RX_BUF_SIZE - 1) {
            s_rx_buf[s_rx_len++] = (char)b;
        } else {
            s_rx_len = 0; /* 溢出丢弃 */
        }
    }
    rx_restart();
}

/* ---------------------------------------------------------------------------
 * UART 错误回调：溢出/帧错误等发生后 HAL 不再触发 RxCplt，需在此重启接收，否则 power 等串口数据会永久停止更新
 * --------------------------------------------------------------------------- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    (void)HAL_UART_AbortReceive(huart);
    s_rx_len = 0;
    rx_restart();
}

void Debug_UART_SetLineCallback(Debug_UART_LineCallback_t cb)
{
    s_line_cb = cb;
}

void Debug_UART_Init(void)
{
    s_rx_len = 0;
    s_line_ready = 0;
    s_dma_tx_idle = 1;
    rx_restart();
}

void Debug_UART_Flush(void)
{
    uint32_t timeout = 500;
    while ((!Debug_IsQueueEmpty() || !s_dma_tx_idle) && timeout--) {
        Debug_Process();
        if (!Debug_IsQueueEmpty() || !s_dma_tx_idle)
            HAL_Delay(1);
    }
}

void Debug_UART_Process(void)
{
    if (!s_line_ready) return;

    s_line_ready = 0;
    /* 拷贝到静态缓冲再回调，避免栈上缓冲被中断覆盖；回调可能调用 Debug_Printf */
    size_t n = strlen(s_line_buf) + 1;
    if (n > DEBUG_UART_RX_BUF_SIZE) n = DEBUG_UART_RX_BUF_SIZE;
    memcpy(s_line_copy, s_line_buf, n);
    s_line_copy[DEBUG_UART_RX_BUF_SIZE - 1] = '\0';
    if (s_line_cb)
        s_line_cb(s_line_copy);
}
