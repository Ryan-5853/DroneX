/**
 * @file User_main.h
 * @brief 用户主程序接口：由 main.c 调用的初始化与主循环。
 */

#ifndef __USER_MAIN_H__
#define __USER_MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 用户初始化：在 HAL 与外设初始化完成后调用，可在此初始化 Debug、驱动等。
 */
void User_Main_Init(void);

/**
 * @brief 用户主循环体：在 while(1) 中每轮调用，可在此执行 Debug_Process、控制逻辑等。
 */
void User_Main_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_MAIN_H__ */
