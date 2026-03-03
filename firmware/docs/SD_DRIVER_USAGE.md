# SD 卡驱动使用说明

## 概述

SD 卡驱动位于 `User/Driver/SD/`，基于 HAL_SD + FatFs，提供文件系统操作接口。  
FatFs 库位于 `3rdPartyLib/FatFs/`，由 SD 驱动引用，不直接嵌入。

## 硬件

- **接口**：SDMMC1，4-bit 模式
- **引脚**：PC8-D0, PC9-D1, PC10-D2, PC11-D3, PC12-CLK, PD2-CMD

## 初始化

`SD_Init()` 已在 `User_Main_Init()` 中调用。无卡或初始化失败时不影响系统启动，可通过 `SD_IsReady()` 检查。

## 日志写入示例

```c
#include "Driver/SD/SD.h"

void write_log_example(void)
{
    if (!SD_IsReady())
        return;

    FIL fp;
    if (SD_CreateFile("0:/log.txt", &fp) != SD_OK)
        return;

    SD_Printf(&fp, "time=%lu, roll=%.2f\r\n", (unsigned long)HAL_GetTick(), 1.23f);
    SD_Close(&fp);
}
```

## 追加写入

```c
FIL fp;
if (SD_Open("0:/log.txt", &fp, FA_OPEN_APPEND | FA_WRITE) == SD_OK) {
    SD_Write(&fp, "new line\n", 9, NULL);
    SD_Close(&fp);
}
```

## 目录操作

```c
SD_Mkdir("0:/logs");           /* 创建目录 */
SD_Unlink("0:/old.txt");       /* 删除文件 */
SD_Rename("0:/a.txt", "0:/b.txt");  /* 重命名 */
```

## 接口一览

| 函数 | 说明 |
|------|------|
| `SD_Init` / `SD_DeInit` | 初始化 / 反初始化 |
| `SD_IsReady` | 检查是否就绪 |
| `SD_CreateFile` | 创建并打开（覆盖） |
| `SD_Open` / `SD_Close` | 打开 / 关闭 |
| `SD_Write` / `SD_Read` | 写入 / 读取 |
| `SD_Printf` | 格式化写入 |
| `SD_Sync` | 刷新到磁盘 |
| `SD_Mkdir` / `SD_Unlink` / `SD_Rename` | 目录与文件管理 |
| `SD_Stat` / `SD_GetFree` | 状态与空间 |
| `SD_Opendir` / `SD_Readdir` / `SD_Closedir` | 目录遍历（参数类型 `FF_DIR`） |

路径格式：`"0:/filename.txt"`，`0:` 表示逻辑驱动器 0（SD 卡）。
