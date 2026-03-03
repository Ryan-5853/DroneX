/**
 * @file ESC_configurer.h
 * @brief ESC 穿透调试模式：进入后阻塞 CPU，轮询 IO 实现信号穿透。
 *        num=1/2 进入电调1/2穿透，num=0 退出。
 *        Debug 层模块，与 CMD 平级。
 *
 * 穿透模式 IO 映射：
 *   PD6(输入) ──→ PA2(上拉开漏输出)：外部信号穿透至电调
 *   PA2(读回) ──→ PD5(推挽输出)  ：电调实际电平回读监控
 */

#ifndef __ESC_CONFIGURER_H__
#define __ESC_CONFIGURER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 请求 ESC 穿透调试（由 ESC_cfg 指令调用）。
 *
 * num=1/2 时：停止所有 PIT 中断，初始化穿透 IO，进入阻塞轮询。
 *             函数在收到 num=0 退出指令前不会返回。
 * num=0 时  ：置位退出标志，解除阻塞循环，恢复 PIT 中断。
 *
 * @param num  0=退出调试模式，1=电调1穿透，2=电调2穿透
 * @return 0 成功，-1 参数无效
 */
int ESC_Config(int num);

#ifdef __cplusplus
}
#endif

#endif /* __ESC_CONFIGURER_H__ */
