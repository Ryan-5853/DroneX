/**
 * @file Cmd_cfg_table.h
 * @brief cfg 指令：参数名字符串 → 内存地址与类型（整型 / 浮点）。
 */

#ifndef __CMD_CFG_TABLE_H__
#define __CMD_CFG_TABLE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 可写参数类型（与表中地址实际类型一致） */
typedef enum {
    CMD_CFG_TYPE_INT32 = 0,
    CMD_CFG_TYPE_FLOAT,
} Cmd_Cfg_Type_t;

typedef struct {
    const char *name; /**< 参数名，与指令中 name= 一致；表以 name==NULL 结尾 */
    void *addr;       /**< 指向 int32_t 或 float，由 type 决定 */
    Cmd_Cfg_Type_t type;
} Cmd_Cfg_Entry_t;

/**
 * @brief 在表中查找参数名（大小写敏感）。
 * @return 命中项指针；未找到返回 NULL
 */
const Cmd_Cfg_Entry_t *Cmd_Cfg_Lookup(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_CFG_TABLE_H__ */
