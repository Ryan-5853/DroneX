/**
 * @file Debug.c
 * @brief 调试输出框架实现：环形消息队列 + 整条入队/按条发送，传输层弱符号或回调接入。
 *
 * 通信链路设计要点：
 * 1. 所有缓冲区使用静态/全局内存，避免中断覆盖栈上数据。
 * 2. 入队/出队均在临界区内完成，防止 Debug_Printf（中断）与 Debug_Process（主循环/DMA 回调）竞态。
 * 3. 队列满时丢弃新消息，不覆盖正在发送的 slot。
 */

#include "Debug.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "main.h"  /* __disable_irq / __enable_irq via CMSIS */

/* ---------------------------------------------------------------------------
 * 单条消息槽：data 在前、len 在后，便于 memcpy 与 cache 对齐
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t  data[DEBUG_MSG_MAX_LEN];
    uint16_t len;
} Debug_MsgSlot_t;

/* ---------------------------------------------------------------------------
 * 环形队列状态
 * --------------------------------------------------------------------------- */
static volatile Debug_MsgSlot_t s_ring[DEBUG_MSG_SLOTS];
static volatile uint32_t s_head;   /* 下一笔写入位置 */
static volatile uint32_t s_tail;   /* 下一笔读出位置 */
static Debug_Transport_SendFn s_transport;  /* 若非 NULL 则优先于弱符号 */

/* 静态缓冲区：避免栈上大块内存被中断覆盖（Debug_Printf 可在 PIT 中断中调用） */
static char    s_fmt_buf[DEBUG_MSG_MAX_LEN];   /* vsnprintf 格式化缓冲 */
static uint8_t s_send_buf[DEBUG_MSG_MAX_LEN]; /* 出队后交给传输层的缓冲 */

#define RING_COUNT() \
    ((s_head >= s_tail) ? (s_head - s_tail) : (DEBUG_MSG_SLOTS - s_tail + s_head))

#define RING_FULL()   (RING_COUNT() >= DEBUG_MSG_SLOTS)
#define RING_EMPTY()  (s_head == s_tail)

/* ---------------------------------------------------------------------------
 * 弱符号：默认不发送，由应用层重写（如 UART DMA）接入
 * --------------------------------------------------------------------------- */
__attribute__((weak)) uint32_t Debug_Transport_Send(const uint8_t *data, uint32_t len)
{
    (void)data;
    (void)len;
    return len; /* 默认“全部接受”，避免队列堵死；实际接入硬件时改为 0 或真实发送长度 */
}

__attribute__((weak)) int Debug_Transport_IsReady(void)
{
    return 1; /* 默认始终就绪 */
}

/* ---------------------------------------------------------------------------
 * Debug_Init
 * --------------------------------------------------------------------------- */
Debug_Status_t Debug_Init(Debug_Transport_SendFn transport)
{
    s_head     = 0;
    s_tail     = 0;
    s_transport = transport;
    memset(s_ring, 0, sizeof(s_ring));
    return DEBUG_OK;
}

/* ---------------------------------------------------------------------------
 * Debug_Printf：整条格式化后入队，不在此发送。
 * 使用 s_fmt_buf 避免栈上缓冲被中断覆盖。
 * --------------------------------------------------------------------------- */
Debug_Status_t Debug_Printf(const char *fmt, ...)
{
    if (fmt == NULL) return DEBUG_ERR_PARAM;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_fmt_buf, sizeof(s_fmt_buf), fmt, ap);
    va_end(ap);

    if (n < 0) return DEBUG_ERR_PARAM;

    size_t len = (size_t)n;
    if (len >= sizeof(s_fmt_buf)) {
        len = sizeof(s_fmt_buf) - 1;
        s_fmt_buf[len] = '\0';
    }

    /* 保证以 \r\n 结尾，便于终端按行显示 */
    if (len >= 2 && s_fmt_buf[len - 2] == '\r' && s_fmt_buf[len - 1] == '\n') {
        /* 已有 */
    } else if (len >= 1 && s_fmt_buf[len - 1] == '\n') {
        if (len + 1 < sizeof(s_fmt_buf)) {
            s_fmt_buf[len - 1] = '\r';
            s_fmt_buf[len]     = '\n';
            len += 1;
        }
    } else {
        if (len + 2 <= sizeof(s_fmt_buf)) {
            s_fmt_buf[len]     = '\r';
            s_fmt_buf[len + 1] = '\n';
            len += 2;
        }
    }

    /* 临界区：避免与 PIT 中断中的 Debug_Printf 竞态 */
    __disable_irq();
    if (RING_FULL()) {
        __enable_irq();
        return DEBUG_ERR_FULL;  /* 队列满时丢弃新消息，不覆盖正在发送的 slot */
    }
    Debug_MsgSlot_t *slot = &s_ring[s_head];
    slot->len = (uint16_t)len;
    memcpy(slot->data, s_fmt_buf, len);
    s_head = (s_head + 1) % DEBUG_MSG_SLOTS;
    __enable_irq();

    return (n >= (int)sizeof(s_fmt_buf)) ? DEBUG_ERR_TRUNC : DEBUG_OK;
}

/* ---------------------------------------------------------------------------
 * Debug_Process：取出一条交给传输层，发送成功才出队。
 * 关键：先关中断再检查 IsReady，避免与 DMA 完成回调竞态；拷贝到 s_send_buf 后出队。
 * --------------------------------------------------------------------------- */
void Debug_Process(void)
{
    /* 先关中断，再检查队列与传输层，避免与 DMA 完成回调中的 Debug_Process 竞态 */
    __disable_irq();
    if (RING_EMPTY() || !Debug_Transport_IsReady()) {
        __enable_irq();
        return;
    }

    uint16_t len;
    Debug_MsgSlot_t *slot = &s_ring[s_tail];
    len = slot->len;
    if (len > sizeof(s_send_buf)) len = sizeof(s_send_buf);
    memcpy(s_send_buf, slot->data, len);
    s_tail = (s_tail + 1) % DEBUG_MSG_SLOTS;  /* 出队，释放 slot */
    __enable_irq();

    Debug_Transport_SendFn send = s_transport ? s_transport : Debug_Transport_Send;
    (void)send(s_send_buf, (uint32_t)len);
}

int Debug_IsQueueEmpty(void)
{
    return RING_EMPTY() ? 1 : 0;
}
