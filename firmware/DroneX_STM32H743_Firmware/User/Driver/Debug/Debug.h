/**
 * @file Debug.h
 * @brief 非阻塞调试输出框架：整条消息格式化入队，由后端按条发送，保证不撕裂。
 *        硬件传输层通过 Debug_Transport_Send 弱符号或注册回调接入，具体 UART/DMA 稍后绑定。
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
#define DEBUG_MSG_MAX_LEN  256   /* 单条消息最大字节数（含结尾 \r\n） */
#endif
#ifndef DEBUG_MSG_SLOTS
#define DEBUG_MSG_SLOTS   8     /* 环形队列消息条数，满则丢最旧 */
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
 * @brief 弱符号：底层发送接口。默认空实现；在应用层或 UART 驱动中重写即可接入硬件。
 * @return 已接受的字节数（若 == len 表示本条已全部交付）；0 表示当前无法发送（非阻塞忙）。
 */
uint32_t Debug_Transport_Send(const uint8_t *data, uint32_t len);

/* ---------------------------------------------------------------------------
 * 使用说明：
 * 1. 在系统初始化时调用 Debug_Init(NULL) 或 Debug_Init(your_transport_callback)。
 * 2. 主循环中周期调用 Debug_Process()，将队列中的消息交给传输层发送。
 * 3. 具体硬件：重写 Debug_Transport_Send()（如 UART+DMA），或通过 Debug_Init(fn) 注册回调。
 * --------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H__ */
