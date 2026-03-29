/**
 * @file Flash_Param.h
 * @brief 内部 Flash 参数区驱动：0x081E0000 - 0x081FFFFF（Bank2 末扇区），整扇区擦除后写入。
 */

#ifndef __FLASH_PARAM_H__
#define __FLASH_PARAM_H__

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "Driver/Flash/ParamFlash_Data.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 参数区起始地址（与链接脚本配合，勿将程序段放入此区间） */
#define FLASH_PARAM_BASE_ADDR   (0x081E0000UL)
/** 参数区结束地址（含） */
#define FLASH_PARAM_END_ADDR    (0x081FFFFFUL)

/** Flash 中存储的布局版本；修改 ParamFlash_Data_t 后请 +1 */
#define FLASH_PARAM_LAYOUT_VER  (1U)

/**
 * @brief 从 Flash 读取校验后的参数块，覆盖 *out。
 * @retval HAL_OK 成功；HAL_ERROR 魔数/CRC/版本或长度不匹配，或指针为空（out 不被改写）。
 */
HAL_StatusTypeDef Flash_Param_Load(ParamFlash_Data_t *out);

/**
 * @brief 将 *in 写入参数区：擦除本扇区后按 32 字节字编程（含文件头与 CRC）。
 * @retval HAL_OK 成功；HAL_ERROR 擦除/编程失败或指针为空。
 */
HAL_StatusTypeDef Flash_Param_Save(const ParamFlash_Data_t *in);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_PARAM_H__ */
