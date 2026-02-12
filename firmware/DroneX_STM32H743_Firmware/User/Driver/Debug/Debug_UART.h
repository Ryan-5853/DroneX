/**
 * @file Debug_UART.h
 * @brief Driver 层：USART1 TX/RX，非阻塞发送与接收。
 *        仅负责协议以下：拆包后通过回调将指令包传递给上层（Debug 层 Cmd 模块）。
 */

#ifndef __DEBUG_UART_H__
#define __DEBUG_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/** 指令包回调：接收到完整行（以 \r\n 结束）时调用，buf 以 \0 结尾 */
typedef void (*Debug_UART_LineCallback_t)(const char *buf);

/**
 * @brief 设置指令包回调。应在 Init 前调用，由 Debug 层注册 Cmd 处理。
 */
void Debug_UART_SetLineCallback(Debug_UART_LineCallback_t cb);

/**
 * @brief 初始化 Debug UART：配置 DMA TX，启动 IT RX。
 *        应在 Debug_Init 之后、主循环前调用。
 */
void Debug_UART_Init(void);

/**
 * @brief 非阻塞处理：从接收缓冲中取出完整行，调用回调传递。
 *        应在主循环中周期调用。
 */
void Debug_UART_Process(void);

/**
 * @brief 刷新输出：等待队列排空且 UART DMA 发送完成。
 *        用于命令响应后确保输出完整再返回，避免偶发撕裂。
 */
void Debug_UART_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_UART_H__ */
