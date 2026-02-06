/**
 * @file Pinout.h
 * @brief 板级外设与引脚映射。外设初始化由 CubeMX 在 main() 中调用 MX_GPIO_Init() / MX_SPIx_Init() 完成，此处仅做「用哪个外设、哪根引脚」的映射。
 *
 * 使用方式：在 CubeMX 中配置好 SPI 与 GPIO 后，在 main.h 中会有 extern SPI_HandleTypeDef hspi1 等；
 * 本文件将逻辑名（如 IMU1_SPI）映射到这些句柄与引脚，供 User_main 或各模块填充具体驱动的 Config 使用。
 */

#ifndef __PINOUT_H__
#define __PINOUT_H__

#include "main.h"

/* ----------------------------------------------------------------------------
 * IMU1（ICM42688，SPI）
 * 请在 CubeMX 中配置：SPI1（或其它 SPIx）、SCK/MISO/MOSI 引脚；CS 使用 GPIO 软件控制则在此指定
 * ---------------------------------------------------------------------------- */
/* main.h 中需有 extern SPI_HandleTypeDef hspi1（在 CubeMX 中使能 SPI1 并重新生成后会自动出现） */
#define IMU1_SPI_HANDLE      (&hspi1)
#define IMU1_CS_GPIO_PORT    GPIOA
#define IMU1_CS_GPIO_PIN     GPIO_PIN_4

#endif /* __PINOUT_H__ */
