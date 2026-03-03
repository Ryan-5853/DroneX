# FatFs - Generic FAT Filesystem Module

FatFs R0.15 (ChaN) - 嵌入式 FAT 文件系统库。

本目录包含 FatFs 核心源码，由 SD 卡驱动层引用。磁盘 I/O 接口（diskio）在 `User/Driver/SD/diskio_sd.c` 中实现。

## 文件说明

- `ff.c`, `ff.h` - FatFs 核心
- `ffconf.h` - 项目配置（无 RTOS、无 LFN、支持 f_printf）
- `diskio.h` - 磁盘 I/O 接口声明
