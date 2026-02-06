/**
 * @file ICM42688.h
 * @brief ICM42688 SPI 驱动：配置类型与接口实现声明（供 IMU_interface 绑定）。
 */

#ifndef __ICM42688_H__
#define __ICM42688_H__

#include "stm32h7xx_hal.h"
#include "IMU_interface.h"

/* ----------------------------------------------------------------------------
 * 实例配置：由调用方填充并传入 IMU_Interface_Init(handle, IMU_TYPE_ICM42688, &cfg)
 * 外设与引脚来源建议从 Pinout.h 的 IMU1_SPI_HANDLE / IMU1_CS_GPIO_* 等映射
 * ---------------------------------------------------------------------------- */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef     *cs_port;
    uint16_t          cs_pin;
} ICM42688_SPI_Config_t;

/* 实现 IMU 接口的 init/read，供 IMU_Interface_Init 挂到 handle 上 */
IMU_Status_t ICM42688_Init(IMU_Handle_t *handle);
IMU_Status_t ICM42688_Read(IMU_Handle_t *handle, IMU_Data_t *data);

#endif /* __ICM42688_H__ */
