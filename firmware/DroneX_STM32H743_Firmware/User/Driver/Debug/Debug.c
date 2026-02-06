/**
 * @file Debug.c
 * @brief 调试输出框架实现：环形消息队列 + 整条入队/按条发送，传输层弱符号或回调接入。
 */

#include "Debug.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * 单条消息槽
 * --------------------------------------------------------------------------- */
typedef struct {
    uint16_t len;
    uint8_t  data[DEBUG_MSG_MAX_LEN];
} Debug_MsgSlot_t;

/* ---------------------------------------------------------------------------
 * 环形队列状态
 * --------------------------------------------------------------------------- */
static Debug_MsgSlot_t s_ring[DEBUG_MSG_SLOTS];
static uint32_t        s_head;   /* 下一笔写入位置 */
static uint32_t        s_tail;   /* 下一笔读出位置 */
static Debug_Transport_SendFn s_transport;  /* 若非 NULL 则优先于弱符号 */

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
 * Debug_Printf：整条格式化后入队，不在此发送
 * --------------------------------------------------------------------------- */
Debug_Status_t Debug_Printf(const char *fmt, ...)
{
    if (fmt == NULL) return DEBUG_ERR_PARAM;

    char buf[DEBUG_MSG_MAX_LEN];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return DEBUG_ERR_PARAM;

    size_t len = (size_t)n;
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
        buf[len] = '\0';
    }

    /* 保证以 \r\n 结尾，便于终端按行显示 */
    if (len >= 2 && buf[len - 2] == '\r' && buf[len - 1] == '\n') {
        /* 已有 */
    } else if (len >= 1 && buf[len - 1] == '\n') {
        if (len + 1 < sizeof(buf)) {
            buf[len - 1] = '\r';
            buf[len]     = '\n';
            len += 1;
        }
    } else {
        if (len + 2 <= sizeof(buf)) {
            buf[len]     = '\r';
            buf[len + 1] = '\n';
            len += 2;
        }
    }

    if (RING_FULL()) {
        /* 丢最旧一条，腾出一格 */
        s_tail = (s_tail + 1) % DEBUG_MSG_SLOTS;
    }

    Debug_MsgSlot_t *slot = &s_ring[s_head];
    slot->len = (uint16_t)len;
    memcpy(slot->data, buf, len);
    s_head = (s_head + 1) % DEBUG_MSG_SLOTS;

    return (n >= (int)sizeof(buf)) ? DEBUG_ERR_TRUNC : DEBUG_OK;
}

/* ---------------------------------------------------------------------------
 * Debug_Process：取出一条交给传输层，发送成功才出队
 * --------------------------------------------------------------------------- */
void Debug_Process(void)
{
    if (RING_EMPTY()) return;

    Debug_MsgSlot_t *slot = &s_ring[s_tail];
    Debug_Transport_SendFn send = s_transport ? s_transport : Debug_Transport_Send;
    uint32_t sent = send(slot->data, (uint32_t)slot->len);

    if (sent == slot->len) {
        s_tail = (s_tail + 1) % DEBUG_MSG_SLOTS;
    }
    /* 若 sent == 0 或 sent < len，保留本条，下次再试，保证不撕裂 */
}
