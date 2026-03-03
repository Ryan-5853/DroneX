/**
 * @file ffconf.h
 * @brief FatFs 配置文件 - 适用于 STM32 嵌入式系统，无 RTOS 依赖
 *
 * FatFs R0.15 - Generic FAT Filesystem Module
 * Copyright (C) 2022, ChaN, all right reserved.
 */

#ifndef _FFCONF_DEFINED
#define _FFCONF_DEFINED

/*---------------------------------------------------------------------------/
/  Configurations of FatFs Module
/---------------------------------------------------------------------------*/

#define FFCONF_DEF  80286   /* Revision ID (须与 ff.h 中 FF_DEFINED 一致) */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_READONLY  0
#define FF_FS_MINIMIZE  0
#define FF_USE_FIND     0
#define FF_USE_MKFS     0
#define FF_USE_FASTSEEK 0
#define FF_USE_EXPAND   0
#define FF_USE_CHMOD    0
#define FF_USE_LABEL    0
#define FF_USE_FORWARD  0

/* 启用 f_printf, f_puts, f_putc, f_gets - 便于写日志 */
#define FF_USE_STRFUNC  1
#define FF_PRINT_LLI    1
#define FF_PRINT_FLOAT  1
#define FF_STRF_ENCODE  3   /* UTF-8 */

/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define FF_CODE_PAGE  936  /* Simplified Chinese (DBCS) */

#define FF_USE_LFN    0    /* 禁用长文件名以节省 RAM */
#define FF_MAX_LFN    255
#define FF_LFN_UNICODE 0
#define FF_LFN_BUF    255
#define FF_SFN_BUF    12

#define FF_FS_RPATH   0

/*---------------------------------------------------------------------------/
/ Drive/Volume Configurations
/---------------------------------------------------------------------------*/

#define FF_VOLUMES       1
#define FF_STR_VOLUME_ID 0
#define FF_VOLUME_STRS   "RAM","NAND","CF","SD","SD2","USB","USB2","USB3"
#define FF_MULTI_PARTITION 0

#define FF_MIN_SS  512
#define FF_MAX_SS  512
#define FF_LBA64   0
#define FF_MIN_GPT 0x10000000
#define FF_USE_TRIM 0

/*---------------------------------------------------------------------------/
/ System Configurations
/---------------------------------------------------------------------------*/

#define FF_FS_TINY    0
#define FF_FS_EXFAT   0

/* 无 RTC：使用固定时间戳 */
#define FF_FS_NORTC   1
#define FF_NORTC_MON  1
#define FF_NORTC_MDAY 1
#define FF_NORTC_YEAR 2025

#define FF_FS_NOFSINFO 0
#define FF_FS_LOCK     0
#define FF_FS_REENTRANT 0
#define FF_FS_TIMEOUT  1000

#endif /* _FFCONF_DEFINED */
