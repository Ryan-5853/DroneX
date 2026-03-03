/**
 * @file SD.h
 * @brief SD 卡驱动：基于 HAL_SD + FatFs，提供文件系统操作接口
 *
 * 硬件：SDMMC1，4-bit 模式
 * 引脚：PC8-D0, PC9-D1, PC10-D2, PC11-D3, PC12-CLK, PD2-CMD
 *
 * 上层（如日志模块）通过本接口创建文件、写入数据、进行基本文件系统操作。
 */

#ifndef __SD_H__
#define __SD_H__

#include "ff.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * 状态码
 * ---------------------------------------------------------------------------- */
typedef enum {
    SD_OK = 0,
    SD_ERR_NOT_READY,   /* 未初始化或卡未就绪 */
    SD_ERR_OPEN,        /* 打开文件失败 */
    SD_ERR_WRITE,       /* 写入失败 */
    SD_ERR_READ,        /* 读取失败 */
    SD_ERR_CLOSE,       /* 关闭失败 */
    SD_ERR_MOUNT,       /* 挂载失败 */
    SD_ERR_PARAM,       /* 参数错误 */
} SD_Status_t;

/* ----------------------------------------------------------------------------
 * 初始化与反初始化
 * ---------------------------------------------------------------------------- */

/**
 * @brief 初始化 SD 卡与文件系统
 * @return SD_OK 成功，否则失败
 */
SD_Status_t SD_Init(void);

/**
 * @brief 反初始化 SD 卡
 */
void SD_DeInit(void);

/**
 * @brief 检查 SD 卡是否就绪
 */
bool SD_IsReady(void);

/* ----------------------------------------------------------------------------
 * 文件操作
 * ---------------------------------------------------------------------------- */

/**
 * @brief 创建并打开文件（若存在则覆盖）
 * @param path 路径，如 "0:/log.txt" 或 "0:/dir/log.txt"
 * @param fp   输出：文件对象指针，由调用方分配
 * @return SD_OK 成功
 */
SD_Status_t SD_CreateFile(const char *path, FIL *fp);

/**
 * @brief 打开已存在的文件
 * @param path 路径
 * @param fp   输出：文件对象指针
 * @param mode FA_READ / FA_WRITE / FA_OPEN_APPEND 等（见 ff.h）
 * @return SD_OK 成功
 */
SD_Status_t SD_Open(const char *path, FIL *fp, uint8_t mode);

/**
 * @brief 关闭文件
 */
SD_Status_t SD_Close(FIL *fp);

/**
 * @brief 写入数据
 * @param fp   已打开的文件对象
 * @param buf  数据缓冲区
 * @param len  字节数
 * @param bw   输出：实际写入字节数（可为 NULL）
 */
SD_Status_t SD_Write(FIL *fp, const void *buf, uint32_t len, uint32_t *bw);

/**
 * @brief 读取数据
 */
SD_Status_t SD_Read(FIL *fp, void *buf, uint32_t len, uint32_t *br);

/**
 * @brief 格式化写入（类似 fprintf）
 * @param fp   已打开的文件对象
 * @param fmt  格式字符串
 * @return 写入字符数，失败返回 < 0
 */
int SD_Printf(FIL *fp, const char *fmt, ...);

/**
 * @brief 刷新文件到磁盘
 */
SD_Status_t SD_Sync(FIL *fp);

/**
 * @brief 移动文件指针
 */
SD_Status_t SD_Seek(FIL *fp, uint32_t offset);

/* ----------------------------------------------------------------------------
 * 目录与文件系统操作
 * ---------------------------------------------------------------------------- */

/**
 * @brief 创建目录
 */
SD_Status_t SD_Mkdir(const char *path);

/**
 * @brief 删除文件或目录
 */
SD_Status_t SD_Unlink(const char *path);

/**
 * @brief 重命名/移动
 */
SD_Status_t SD_Rename(const char *old_path, const char *new_path);

/**
 * @brief 获取文件状态
 */
SD_Status_t SD_Stat(const char *path, FILINFO *fno);

/**
 * @brief 获取空闲簇数量
 * @param nclst 输出：空闲簇数
 */
SD_Status_t SD_GetFree(uint32_t *nclst);

/**
 * @brief 打开目录
 */
SD_Status_t SD_Opendir(const char *path, FF_DIR *dp);

/**
 * @brief 读取目录项
 */
SD_Status_t SD_Readdir(FF_DIR *dp, FILINFO *fno);

/**
 * @brief 关闭目录
 */
SD_Status_t SD_Closedir(FF_DIR *dp);

#ifdef __cplusplus
}
#endif

#endif /* __SD_H__ */
