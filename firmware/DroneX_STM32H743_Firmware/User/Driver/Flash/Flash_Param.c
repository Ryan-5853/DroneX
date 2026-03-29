/**
 * @file Flash_Param.c
 */

#include "Driver/Flash/Flash_Param.h"
#include "stm32h7xx_hal_flash_ex.h"
#include <string.h>

#define FLASH_PARAM_MAGIC       (0x44525058UL) /* 'DRPX' */

#define FLASH_WORD_BYTES        (32U)
#define FLASH_PARAM_HEADER_BYTES (FLASH_WORD_BYTES)

#define FLASH_PARAM_BANK        (FLASH_BANK_2)
#define FLASH_PARAM_SECTOR      (FLASH_SECTOR_7)

static uint32_t round_up_u32(uint32_t v, uint32_t align)
{
    return (v + align - 1U) & ~(align - 1U);
}

static uint32_t blob_total_bytes(void)
{
    return round_up_u32(FLASH_PARAM_HEADER_BYTES + (uint32_t)sizeof(ParamFlash_Data_t), FLASH_WORD_BYTES);
}

typedef struct
{
    uint32_t magic;
    uint32_t layout_ver;
    uint32_t data_size;
    uint32_t crc32;
    uint32_t reserved[4];
} Flash_Param_FileHeader_t;


static uint32_t crc32_ieee(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i];
        for (int b = 0; b < 8; b++)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static HAL_StatusTypeDef flash_param_erase_sector(void)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Banks        = FLASH_PARAM_BANK,
        .Sector       = FLASH_PARAM_SECTOR,
        .NbSectors    = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t sector_err = 0xFFFFFFFFUL;
    return HAL_FLASHEx_Erase(&erase, &sector_err);
}

HAL_StatusTypeDef Flash_Param_Load(ParamFlash_Data_t *out)
{
    if (out == NULL)
    {
        return HAL_ERROR;
    }

    const Flash_Param_FileHeader_t *hdr =
        (const Flash_Param_FileHeader_t *)(uintptr_t)FLASH_PARAM_BASE_ADDR;

    if (hdr->magic != FLASH_PARAM_MAGIC)
    {
        return HAL_ERROR;
    }
    if (hdr->layout_ver != FLASH_PARAM_LAYOUT_VER)
    {
        return HAL_ERROR;
    }
    if (hdr->data_size != (uint32_t)sizeof(ParamFlash_Data_t))
    {
        return HAL_ERROR;
    }

    const uint8_t *payload = (const uint8_t *)(uintptr_t)(FLASH_PARAM_BASE_ADDR + FLASH_PARAM_HEADER_BYTES);
    uint32_t crc_expect = crc32_ieee(payload, hdr->data_size);
    if (crc_expect != hdr->crc32)
    {
        return HAL_ERROR;
    }

    memcpy(out, payload, sizeof(ParamFlash_Data_t));
    return HAL_OK;
}

HAL_StatusTypeDef Flash_Param_Save(const ParamFlash_Data_t *in)
{
    if (in == NULL)
    {
        return HAL_ERROR;
    }

    Flash_Param_FileHeader_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic      = FLASH_PARAM_MAGIC;
    hdr.layout_ver = FLASH_PARAM_LAYOUT_VER;
    hdr.data_size  = (uint32_t)sizeof(ParamFlash_Data_t);
    hdr.crc32      = crc32_ieee((const uint8_t *)in, hdr.data_size);

    uint32_t total = blob_total_bytes();

    static __attribute__((aligned(32))) uint32_t s_flash_word[FLASH_WORD_BYTES / sizeof(uint32_t)];

    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK)
    {
        return st;
    }

    st = flash_param_erase_sector();
    if (st == HAL_OK)
    {
        for (uint32_t off = 0; off < total && st == HAL_OK; off += FLASH_WORD_BYTES)
        {
            memset(s_flash_word, 0xFF, FLASH_WORD_BYTES);
            if (off < FLASH_PARAM_HEADER_BYTES)
            {
                memcpy(s_flash_word, (const uint8_t *)&hdr + off, FLASH_PARAM_HEADER_BYTES - off);
            }
            else
            {
                uint32_t payload_off = off - FLASH_PARAM_HEADER_BYTES;
                uint32_t n = FLASH_WORD_BYTES;
                if (payload_off < sizeof(ParamFlash_Data_t))
                {
                    uint32_t copy = sizeof(ParamFlash_Data_t) - payload_off;
                    if (copy > n)
                    {
                        copy = n;
                    }
                    memcpy(s_flash_word, (const uint8_t *)in + payload_off, copy);
                }
            }

            st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   FLASH_PARAM_BASE_ADDR + off,
                                   (uint32_t)(uintptr_t)s_flash_word);
        }
    }

    (void)HAL_FLASH_Lock();
    return st;
}
