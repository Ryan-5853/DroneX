/**
 * @file Cmd.h
 * @brief 上位机指令解析与执行：格式 <cmd>:para1=xx;para2=xx;\r\n
 *        解析后按命令名查表，调用注册的回调并传入参数。
 *        Debug 层模块，与 Driver 层平级。
 */

#ifndef __CMD_H__
#define __CMD_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * 配置
 * -------------------------------------------------------------------------- */
#ifndef CMD_MAX_LEN
#define CMD_MAX_LEN       256   /* 单条指令最大字节数（含 \r\n） */
#endif
#ifndef CMD_MAX_PARAMS
#define CMD_MAX_PARAMS    16    /* 单条指令最大参数个数 */
#endif
#ifndef CMD_MAX_NAME_LEN
#define CMD_MAX_NAME_LEN  32   /* 命令名最大长度 */
#endif
#ifndef CMD_MAX_VAL_LEN
#define CMD_MAX_VAL_LEN   64   /* 单个参数值最大长度 */
#endif

/* ----------------------------------------------------------------------------
 * 参数结构：key=value
 * -------------------------------------------------------------------------- */
typedef struct {
    char key[CMD_MAX_VAL_LEN];   /* 参数名，不含 '=' */
    char value[CMD_MAX_VAL_LEN]; /* 参数值 */
} Cmd_Param_t;

/* ----------------------------------------------------------------------------
 * 回调类型：cmd 为命令名，params 为参数数组，n 为参数个数
 * -------------------------------------------------------------------------- */
typedef void (*Cmd_Callback_t)(const char *cmd, const Cmd_Param_t *params, int n);

/* ----------------------------------------------------------------------------
 * 返回值
 * -------------------------------------------------------------------------- */
typedef enum {
    CMD_OK = 0,
    CMD_ERR_PARAM,
    CMD_ERR_EMPTY,
    CMD_ERR_UNKNOWN,
    CMD_ERR_TOO_LONG,
} Cmd_Status_t;

/**
 * @brief 初始化 Cmd 模块（清空命令表）。
 */
void Cmd_Init(void);

/**
 * @brief 注册内置命令：help、version、status、reset、imu_cali。
 */
void Cmd_RegisterBuiltins(void);

/**
 * @brief 注册命令：命令名 -> 回调。同名覆盖。
 * @param name 命令名（如 "help"），匹配时大小写敏感
 * @param cb   回调函数
 */
void Cmd_Register(const char *name, Cmd_Callback_t cb);

/**
 * @brief 解析并执行一条指令。格式：<cmd>:para1=xx;para2=xx;\r\n
 *        由 Driver 层拆包后传入。
 * @param buf  以 \0 结尾的指令字符串（可含 \r\n，会忽略）
 * @return CMD_OK 成功执行；CMD_ERR_* 解析或执行失败
 */
Cmd_Status_t Cmd_Process(const char *buf);

/**
 * @brief 按参数名查找参数值
 * @return 指向 value 的指针；未找到返回 NULL
 */
const char *Cmd_GetParam(const Cmd_Param_t *params, int n, const char *key);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_H__ */
