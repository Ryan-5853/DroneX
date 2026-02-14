/**
 * @file Debug.h
 * @brief 非阻塞调试输出框架：整条消息格式化入队，由后端按条发送，保证不撕裂。
 *
 * 通信链路：
 *   Debug_Printf (主循环/中断) -> 环形队列 -> Debug_Process (主循环/DMA 回调) -> Debug_Transport_Send -> UART DMA
 *
 * 设计要点：
 *   - 所有缓冲使用静态内存，避免中断覆盖栈
 *   - 入队/出队在临界区内完成，防止竞态
 *   - 队列满时丢弃新消息，不覆盖正在发送的 slot
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * 配置（可移至 Config.h 或通过编译选项覆盖）
 * -------------------------------------------------------------------------- */
#ifndef DEBUG_MSG_MAX_LEN
#define DEBUG_MSG_MAX_LEN  1024   /* 单条消息最大字节数（含结尾 \r\n）；需与 MPU DMA 区域大小一致 */
#endif
#ifndef DEBUG_MSG_SLOTS
#define DEBUG_MSG_SLOTS   8       /* 环形队列消息条数；满则丢弃新消息 */
#endif

/* ---------------------------------------------------------------------------
 * 返回值
 * --------------------------------------------------------------------------- */
typedef enum {
    DEBUG_OK = 0,
    DEBUG_ERR_PARAM,
    DEBUG_ERR_FULL,     /* 队列满，本条被丢弃 */
    DEBUG_ERR_TRUNC,    /* 格式化截断，但已入队 */
} Debug_Status_t;

/* ---------------------------------------------------------------------------
 * 传输回调：将一段数据发送出去（如 UART DMA）。
 * 非阻塞：若当前无法发送则返回 0，框架会保留当前条下次再试。
 * 若返回 len，表示已接受整段数据，框架会从队列移除该条并继续下一条。
 * 若使用弱符号 Debug_Transport_Send，则无需注册。
 * --------------------------------------------------------------------------- */
typedef uint32_t (*Debug_Transport_SendFn)(const uint8_t *data, uint32_t len);

/**
 * @brief 初始化调试模块（清空队列，可选注册传输回调）。
 * @param transport 若非 NULL，将作为发送回调；否则使用弱符号 Debug_Transport_Send。
 */
Debug_Status_t Debug_Init(Debug_Transport_SendFn transport);

/**
 * @brief 非阻塞格式化输出：先整条 vsnprintf，再入队。不在本函数内发送。
 * @return DEBUG_OK 入队成功；DEBUG_ERR_FULL 队列满丢弃；DEBUG_ERR_TRUNC 被截断但已入队。
 */
Debug_Status_t Debug_Printf(const char *fmt, ...);

/**
 * @brief 从队列取出一条消息并交给传输层发送。应在主循环或 DMA 空闲时周期调用。
 *        若传输层返回 0（忙），本条会保留，下次再试，保证按条完整发送。
 */
void Debug_Process(void);

/**
 * @brief 队列是否为空。
 */
int Debug_IsQueueEmpty(void);

/**
 * @brief 弱符号：底层发送接口。默认空实现；在应用层或 UART 驱动中重写即可接入硬件。
 * @return 已接受的字节数（若 == len 表示本条已全部交付）；0 表示当前无法发送（非阻塞忙）。
 */
uint32_t Debug_Transport_Send(const uint8_t *data, uint32_t len);

/**
 * @brief 弱符号：传输层是否空闲可发送。DMA 驱动重写，返回 1 表示空闲。
 */
int Debug_Transport_IsReady(void);

/* ---------------------------------------------------------------------------
 * 使用说明：
 * 1. Debug_Init(transport)：初始化队列，注册传输回调（如 Debug_Transport_Send）。
 * 2. Debug_UART_Init()：初始化 UART、DMA TX、IT RX（在 Debug_Init 之后）。
 * 3. 主循环：Debug_UART_Process()（处理 RX 拆包）、Debug_Process()（驱动 TX 发送）。
 * 4. DMA 完成回调中调用 Debug_Process()，实现链式发送下一条。
 * --------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
