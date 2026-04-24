#ifndef __RC_H__
#define __RC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t  ch[16];
    //通道定义如下：
    /*
        CH1:右摇杆向右为正；-1000~1000
        CH2:右摇杆向前为正；-1000~1000

        CH3:左摇杆向前为正；-1000~1000
        CH4:左摇杆向右为正；-1000~1000

        CH5:最左侧钮子开关，下1000中0上-1000
        CH6:次左侧钮子开关，下1000上-1000
        CH7:次右侧钮子开关，下1000上-1000
        CH8:最右侧钮子开关，下1000中0上-1000

        CH9:左旋钮，向右为正，-1000~1000
        CH10:右旋钮，向右为正，-1000~1000

        其他通道预留，暂不使用


        应用层定义：
    */
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
