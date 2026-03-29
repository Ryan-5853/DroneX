/**
 * @file Cmd_cfg_table.c
 * @brief cfg 映射表：在此添加 { "名称", &变量, CMD_CFG_TYPE_* }，最后一项由哨兵结束。
 */

#include "Debug/Cmd/Cmd_cfg_table.h"
#include <string.h>

/*
 * 示例（取消注释并保证变量链接可见）：
 * static int32_t s_dbg_mode;
 * static float s_dbg_gain;
 * static const Cmd_Cfg_Entry_t s_cfg_table[] = {
 *     { "dbg_mode", &s_dbg_mode, CMD_CFG_TYPE_INT32 },
 *     { "dbg_gain", &s_dbg_gain, CMD_CFG_TYPE_FLOAT },
 *     { NULL, NULL, CMD_CFG_TYPE_INT32 },
 * };
 */

static const Cmd_Cfg_Entry_t s_cfg_table[] = {
    { NULL, NULL, CMD_CFG_TYPE_INT32 },
};

const Cmd_Cfg_Entry_t *Cmd_Cfg_Lookup(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (const Cmd_Cfg_Entry_t *e = s_cfg_table; e->name != NULL; e++) {
        if (strcmp(e->name, name) == 0) {
            return e;
        }
    }
    return NULL;
}
