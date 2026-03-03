/**
 * @file diskio_sd.c
 * @brief FatFs 磁盘 I/O 层：SD 卡 HAL_SD 桥接
 *
 * 实现 disk_initialize, disk_status, disk_read, disk_write, disk_ioctl，
 * 供 FatFs 调用。底层使用 HAL_SD。
 */

#include "diskio.h"
#include "stm32h7xx_hal.h"

/* SD 句柄：由 SD.c 在 SD_Init 时填充 */
extern SD_HandleTypeDef hsd_sdmmc;

#define SD_PDRV  0   /* 物理驱动器号 */

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != SD_PDRV)
        return STA_NOINIT;

    if (HAL_SD_Init(&hsd_sdmmc) != HAL_OK)
        return STA_NOINIT;

    return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != SD_PDRV)
        return STA_NOINIT;

    if (HAL_SD_GetCardState(&hsd_sdmmc) != HAL_SD_CARD_TRANSFER)
        return STA_NOINIT;

    return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != SD_PDRV)
        return RES_PARERR;

    if (HAL_SD_ReadBlocks(&hsd_sdmmc, buff, sector, count, 5000) != HAL_OK)
        return RES_ERROR;

    /* 等待传输完成，带超时防止无卡时死循环 */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd_sdmmc) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) >= 5000U)
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != SD_PDRV)
        return RES_PARERR;

    if (HAL_SD_WriteBlocks(&hsd_sdmmc, (uint8_t *)buff, sector, count, 5000) != HAL_OK)
        return RES_ERROR;

    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(&hsd_sdmmc) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) >= 5000U)
            return RES_ERROR;
    }
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != SD_PDRV)
        return RES_PARERR;

    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;

    case GET_SECTOR_COUNT: {
        HAL_SD_CardInfoTypeDef info;
        if (HAL_SD_GetCardInfo(&hsd_sdmmc, &info) != HAL_OK)
            return RES_ERROR;
        *(LBA_t *)buff = info.LogBlockNbr;
        return RES_OK;
    }

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        return RES_OK;

    default:
        return RES_PARERR;
    }
}
