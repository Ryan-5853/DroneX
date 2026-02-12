/**
 * @file Cmd.c
 * @brief 指令解析与回调执行。格式：<cmd>:para1=xx;para2=xx;\r\n
 *        Debug 层模块。
 */

#include "Debug/Cmd/Cmd.h"
#include <string.h>
#include <ctype.h>

#define CMD_TABLE_SIZE  24

typedef struct {
    char name[CMD_MAX_NAME_LEN];
    Cmd_Callback_t cb;
} Cmd_Entry_t;

static Cmd_Entry_t s_table[CMD_TABLE_SIZE];
static int s_count;

/* 去除首尾空白 */
static void trim(char *s)
{
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    p = s + strlen(s);
    while (p > s && isspace((unsigned char)*(p - 1))) *--p = '\0';
}

void Cmd_Init(void)
{
    s_count = 0;
    memset(s_table, 0, sizeof(s_table));
}

void Cmd_Register(const char *name, Cmd_Callback_t cb)
{
    if (name == NULL || cb == NULL) return;

    /* 同名覆盖 */
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_table[i].name, name) == 0) {
            s_table[i].cb = cb;
            return;
        }
    }

    if (s_count >= CMD_TABLE_SIZE) return;

    size_t len = strlen(name);
    if (len >= CMD_MAX_NAME_LEN) len = CMD_MAX_NAME_LEN - 1;
    memcpy(s_table[s_count].name, name, len);
    s_table[s_count].name[len] = '\0';
    s_table[s_count].cb = cb;
    s_count++;
}

static int str_equal(const char *a, const char *b)
{
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b))
            return 0;
        a++; b++;
    }
    return (*a == *b);
}

static Cmd_Callback_t find_cb(const char *name)
{
    for (int i = 0; i < s_count; i++) {
        if (str_equal(s_table[i].name, name))
            return s_table[i].cb;
    }
    return NULL;
}

Cmd_Status_t Cmd_Process(const char *buf)
{
    if (buf == NULL) return CMD_ERR_PARAM;

    char work[CMD_MAX_LEN];
    size_t len = strlen(buf);
    if (len >= sizeof(work)) return CMD_ERR_TOO_LONG;
    memcpy(work, buf, len + 1);

    /* 去掉 \r\n */
    for (char *p = work; *p; p++) {
        if (*p == '\r' || *p == '\n') { *p = '\0'; break; }
    }
    trim(work);
    if (work[0] == '\0') return CMD_ERR_EMPTY;

    /* 找第一个 ':' 或全角 '：'（U+FF1A，UTF-8: EF BC 9A） */
    char *colon = strchr(work, ':');
    if (colon == NULL) {
        for (char *p = work; p[0] && p[1] && p[2]; p++) {
            if ((unsigned char)p[0] == 0xEF && (unsigned char)p[1] == 0xBC && (unsigned char)p[2] == 0x9A) {
                p[0] = ':';
                memmove(p + 1, p + 3, strlen(p + 3) + 1);
                colon = p;
                break;
            }
        }
    }

    char cmd[CMD_MAX_NAME_LEN];
    char *rest;
    if (colon != NULL) {
        size_t cmd_len = (size_t)(colon - work);
        if (cmd_len >= CMD_MAX_NAME_LEN) cmd_len = CMD_MAX_NAME_LEN - 1;
        memcpy(cmd, work, cmd_len);
        cmd[cmd_len] = '\0';
        rest = colon + 1;
    } else {
        /* 无冒号：整行视为命令名，无参数（如 help、version） */
        size_t cmd_len = strlen(work);
        if (cmd_len >= CMD_MAX_NAME_LEN) cmd_len = CMD_MAX_NAME_LEN - 1;
        memcpy(cmd, work, cmd_len);
        cmd[cmd_len] = '\0';
        rest = work + cmd_len;  /* 指向结尾，*rest='\0' */
    }
    trim(cmd);
    if (cmd[0] == '\0') return CMD_ERR_PARAM;

    Cmd_Callback_t cb = find_cb(cmd);
    if (cb == NULL) return CMD_ERR_UNKNOWN;

    /* 解析参数：para1=xx;para2=xx; */
    Cmd_Param_t params[CMD_MAX_PARAMS];
    int n = 0;

    while (*rest && n < CMD_MAX_PARAMS) {
        trim(rest);
        if (*rest == '\0') break;

        char *eq = strchr(rest, '=');
        if (eq == NULL) break;

        size_t klen = (size_t)(eq - rest);
        if (klen >= CMD_MAX_VAL_LEN) klen = CMD_MAX_VAL_LEN - 1;
        memcpy(params[n].key, rest, klen);
        params[n].key[klen] = '\0';
        trim(params[n].key);

        char *semi = strchr(eq + 1, ';');
        if (semi != NULL) {
            size_t vlen = (size_t)(semi - (eq + 1));
            if (vlen >= CMD_MAX_VAL_LEN) vlen = CMD_MAX_VAL_LEN - 1;
            memcpy(params[n].value, eq + 1, vlen);
            params[n].value[vlen] = '\0';
            rest = semi + 1;
        } else {
            /* 最后一个参数，到行尾 */
            size_t vlen = strlen(eq + 1);
            if (vlen >= CMD_MAX_VAL_LEN) vlen = CMD_MAX_VAL_LEN - 1;
            memcpy(params[n].value, eq + 1, vlen);
            params[n].value[vlen] = '\0';
            rest = eq + 1 + vlen;
        }
        trim(params[n].value);
        n++;
    }

    cb(cmd, params, n);
    return CMD_OK;
}

const char *Cmd_GetParam(const Cmd_Param_t *params, int n, const char *key)
{
    if (params == NULL || key == NULL) return NULL;
    for (int i = 0; i < n; i++) {
        if (strcmp(params[i].key, key) == 0)
            return params[i].value;
    }
    return NULL;
}
