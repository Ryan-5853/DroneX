#ifndef __RC_H__
#define __RC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t  ch[16];
    uint8_t  ch17;
    uint8_t  ch18;
    uint8_t  frame_lost;
    uint8_t  failsafe;
    uint8_t  is_online;
    uint32_t last_update_ms;
    uint32_t frame_count;
} RC_Signal_t;

void RC_Init(void);
void RC_Process(void);
const RC_Signal_t *RC_GetSignal(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC_H__ */
