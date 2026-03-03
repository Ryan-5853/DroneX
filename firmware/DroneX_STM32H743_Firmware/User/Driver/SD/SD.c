/**
 * @file SD.c
 * @brief SD 卡驱动：HAL 初始化、FatFs 挂载、高层文件操作接口
 */

#include "SD.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>   /* vsnprintf for SD_Printf */

/* SD 句柄：供 diskio_sd.c 使用 */
SD_HandleTypeDef hsd_sdmmc;

static FATFS s_fatfs;
static bool s_mounted = false;

static SD_Status_t FR_To_SD(FRESULT fr)
{
    switch (fr) {
    case FR_OK:               return SD_OK;
    case FR_DISK_ERR:         return SD_ERR_NOT_READY;
    case FR_NOT_READY:        return SD_ERR_NOT_READY;
    case FR_NO_FILE:          return SD_ERR_OPEN;
    case FR_NO_PATH:          return SD_ERR_OPEN;
    case FR_INVALID_NAME:     return SD_ERR_PARAM;
    case FR_DENIED:           return SD_ERR_WRITE;
    case FR_EXIST:            return SD_ERR_OPEN;
    case FR_INVALID_OBJECT:   return SD_ERR_PARAM;
    case FR_WRITE_PROTECTED: return SD_ERR_WRITE;
    case FR_INVALID_DRIVE:    return SD_ERR_PARAM;
    case FR_NOT_ENABLED:      return SD_ERR_MOUNT;
    case FR_NO_FILESYSTEM:    return SD_ERR_MOUNT;
    default:                  return SD_ERR_PARAM;
    }
}

SD_Status_t SD_Init(void)
{
    FRESULT fr;

    hsd_sdmmc.Instance = SDMMC1;
    hsd_sdmmc.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd_sdmmc.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd_sdmmc.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd_sdmmc.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd_sdmmc.Init.ClockDiv            = 4;   /* SDMMC_CLK = 480MHz / (2*4) = 60MHz */

    if (HAL_SD_Init(&hsd_sdmmc) != HAL_OK)
        return SD_ERR_NOT_READY;

    fr = f_mount(&s_fatfs, "0:", 1);
    if (fr != FR_OK)
        return FR_To_SD(fr);

    s_mounted = true;
    return SD_OK;
}

void SD_DeInit(void)
{
    if (s_mounted) {
        f_mount(NULL, "0:", 0);
        s_mounted = false;
    }
    HAL_SD_DeInit(&hsd_sdmmc);
}

bool SD_IsReady(void)
{
    return s_mounted && (HAL_SD_GetCardState(&hsd_sdmmc) == HAL_SD_CARD_TRANSFER);
}

SD_Status_t SD_CreateFile(const char *path, FIL *fp)
{
    FRESULT fr = f_open(fp, path, FA_CREATE_ALWAYS | FA_WRITE);
    return FR_To_SD(fr);
}

SD_Status_t SD_Open(const char *path, FIL *fp, uint8_t mode)
{
    FRESULT fr = f_open(fp, path, mode);
    return FR_To_SD(fr);
}

SD_Status_t SD_Close(FIL *fp)
{
    FRESULT fr = f_close(fp);
    return FR_To_SD(fr);
}

SD_Status_t SD_Write(FIL *fp, const void *buf, uint32_t len, uint32_t *bw)
{
    UINT n;
    FRESULT fr = f_write(fp, buf, len, &n);
    if (bw) *bw = n;
    return FR_To_SD(fr);
}

SD_Status_t SD_Read(FIL *fp, void *buf, uint32_t len, uint32_t *br)
{
    UINT n;
    FRESULT fr = f_read(fp, buf, len, &n);
    if (br) *br = n;
    return FR_To_SD(fr);
}

int SD_Printf(FIL *fp, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0 || (unsigned)n >= sizeof(buf))
        return -1;
    UINT bw;
    if (f_write(fp, buf, (unsigned)n, &bw) != FR_OK || bw != (unsigned)n)
        return -1;
    return (int)bw;
}

SD_Status_t SD_Sync(FIL *fp)
{
    return FR_To_SD(f_sync(fp));
}

SD_Status_t SD_Seek(FIL *fp, uint32_t offset)
{
    return FR_To_SD(f_lseek(fp, offset));
}

SD_Status_t SD_Mkdir(const char *path)
{
    return FR_To_SD(f_mkdir(path));
}

SD_Status_t SD_Unlink(const char *path)
{
    return FR_To_SD(f_unlink(path));
}

SD_Status_t SD_Rename(const char *old_path, const char *new_path)
{
    return FR_To_SD(f_rename(old_path, new_path));
}

SD_Status_t SD_Stat(const char *path, FILINFO *fno)
{
    return FR_To_SD(f_stat(path, fno));
}

SD_Status_t SD_GetFree(uint32_t *nclst)
{
    FATFS *fs;
    DWORD n;
    FRESULT fr = f_getfree("0:", &n, &fs);
    if (fr != FR_OK)
        return FR_To_SD(fr);
    if (nclst) *nclst = n;
    return SD_OK;
}

SD_Status_t SD_Opendir(const char *path, FF_DIR *dp)
{
    return FR_To_SD(f_opendir(dp, path));
}

SD_Status_t SD_Readdir(FF_DIR *dp, FILINFO *fno)
{
    return FR_To_SD(f_readdir(dp, fno));
}

SD_Status_t SD_Closedir(FF_DIR *dp)
{
    return FR_To_SD(f_closedir(dp));
}
