/**
 * @file ParamFlash_Data.h
 * @brief 需掉电保存的参数体：在此定义字段；变更布局时请同步提高 Flash_Param 中的布局版本号。
 */

#ifndef __PARAM_FLASH_DATA_H__
#define __PARAM_FLASH_DATA_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 应用参数（RAM 中与 Flash 中各一份，由 Flash_Param_Save / Flash_Param_Load 同步）。
 */
typedef struct
{
    uint32_t flags;
    uint32_t reserved_u32[7];
    /* 在此添加 float/int/数组等；保持自然对齐；改结构后递增 Flash_Param 布局版本 */
} ParamFlash_Data_t;

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_FLASH_DATA_H__ */
