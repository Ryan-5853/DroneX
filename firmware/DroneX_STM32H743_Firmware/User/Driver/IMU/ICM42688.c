/**
 * @file ICM42688.c
 * @brief ICM42688-P SPI 驱动：仅做硬件读写与物理量换算，不做偏置校正。
 */

#include "ICM42688.h"

/* ----------------------------------------------------------------------------
 * 寄存器地址（Bank 0，详见 DS-000347 ICM-42688-P）
 * ---------------------------------------------------------------------------- */
#define ICM42688_REG_BANK_SEL    0x76u
#define ICM42688_REG_WHO_AM_I    0x75u
#define ICM42688_REG_PWR_MGMT0   0x4Eu
#define ICM42688_REG_GYRO_CONFIG0 0x4Fu
#define ICM42688_REG_ACCEL_CONFIG0 0x50u
#define ICM42688_REG_TEMP_DATA1  0x1Du
#define ICM42688_WHO_AM_I_VAL    0x47u

/* SPI 读：首字节 0x80 | reg_addr */
#define ICM42688_SPI_READ        (1u << 7)

/* 量程与换算（当前配置：2000 dps，16g；若修改 GYRO/ACCEL_CONFIG0 需同步改此处） */
#define ICM42688_GYRO_FS_DPS     2000.f
#define ICM42688_ACCEL_FS_G      16.f
#define ICM42688_GYRO_SCALE      (ICM42688_GYRO_FS_DPS / 32768.f * (3.14159265358979f / 180.f))
#define ICM42688_ACCEL_SCALE     (ICM42688_ACCEL_FS_G * 9.80665f / 32768.f)
#define ICM42688_TEMP_SCALE      (1.f / 132.48f)
#define ICM42688_TEMP_OFFSET     25.f

static IMU_Status_t ICM42688_WriteReg(ICM42688_SPI_Config_t *cfg, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7Fu, val };
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_RESET);
    IMU_Status_t ok = (HAL_SPI_Transmit(cfg->hspi, tx, 2, 10) == HAL_OK) ? IMU_OK : IMU_ERROR;
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
    return ok;
}

static IMU_Status_t ICM42688_ReadBlock(ICM42688_SPI_Config_t *cfg, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (len == 0) return IMU_OK;
    uint8_t addr = reg | ICM42688_SPI_READ;
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_RESET);
    IMU_Status_t ok = IMU_ERROR;
    if (HAL_SPI_Transmit(cfg->hspi, &addr, 1, 10) == HAL_OK &&
        HAL_SPI_Receive(cfg->hspi, buf, len, 10) == HAL_OK)
        ok = IMU_OK;
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
    return ok;
}

IMU_Status_t ICM42688_Init(IMU_Handle_t *handle)
{
    if (handle == NULL || handle->context == NULL) return IMU_ERROR;
    ICM42688_SPI_Config_t *cfg = (ICM42688_SPI_Config_t *)handle->context;

    /* CS 空闲为高 */
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);

    /* 选 Bank 0：写 REG_BANK_SEL(0x76) = 0 */
    uint8_t tx_bank[2] = { ICM42688_REG_BANK_SEL & 0x7Fu, 0x00u };
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(cfg->hspi, tx_bank, 2, 10);
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);

    /* 读 WHO_AM_I */
    uint8_t who = 0;
    if (ICM42688_ReadBlock(cfg, ICM42688_REG_WHO_AM_I, &who, 1) != IMU_OK)
        return IMU_ERROR;
    if (who != ICM42688_WHO_AM_I_VAL)
        return IMU_ERROR;

    /* 上电：PWR_MGMT0 = 0x0F（陀螺+加速度计 LN 模式） */
    if (ICM42688_WriteReg(cfg, ICM42688_REG_PWR_MGMT0, 0x0Fu) != IMU_OK)
        return IMU_ERROR;
    /* 量程：GYRO_CONFIG0 / ACCEL_CONFIG0（示例：2000dps、16g，按需改） */
    if (ICM42688_WriteReg(cfg, ICM42688_REG_GYRO_CONFIG0, 0x03u) != IMU_OK)
        return IMU_ERROR;
    if (ICM42688_WriteReg(cfg, ICM42688_REG_ACCEL_CONFIG0, 0x03u) != IMU_OK)
        return IMU_ERROR;

    return IMU_OK;
}

IMU_Status_t ICM42688_Read(IMU_Handle_t *handle, IMU_Data_t *data)
{
    if (handle == NULL || handle->context == NULL || data == NULL) return IMU_ERROR;
    ICM42688_SPI_Config_t *cfg = (ICM42688_SPI_Config_t *)handle->context;

    /* 从 0x1D 起连续读 14 字节：TEMP(2) + ACCEL(6) + GYRO(6)，对应 ICM42688-P 寄存器顺序 */
    uint8_t raw[14];
    if (ICM42688_ReadBlock(cfg, ICM42688_REG_TEMP_DATA1, raw, sizeof(raw)) != IMU_OK)
        return IMU_ERROR;

    /* 温度：TEMP_DATA1(高) TEMP_DATA0(低) */
    int16_t temp_raw = (int16_t)((raw[0] << 8) | raw[1]);
    data->temperature = (float)temp_raw * ICM42688_TEMP_SCALE + ICM42688_TEMP_OFFSET;

    /* 加速度：0x1F~0x24，相对 0x1D 偏移 2（DATASHEET: TEMP->ACCEL->GYRO 顺序） */
    int16_t ax = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t ay = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t az = (int16_t)((raw[6] << 8) | raw[7]);
    data->accel[0] = (float)ax * ICM42688_ACCEL_SCALE;
    data->accel[1] = (float)ay * ICM42688_ACCEL_SCALE;
    data->accel[2] = (float)az * ICM42688_ACCEL_SCALE;

    /* 陀螺：0x25~0x2A，相对 0x1D 偏移 8 */
    int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
    data->gyro[0] = (float)gx * ICM42688_GYRO_SCALE;
    data->gyro[1] = (float)gy * ICM42688_GYRO_SCALE;
    data->gyro[2] = (float)gz * ICM42688_GYRO_SCALE;

    return IMU_OK;
}

IMU_Status_t ICM42688_ReadWithRaw(IMU_Handle_t *handle, IMU_Data_t *data, ICM42688_RawData_t *raw)
{
    if (handle == NULL || handle->context == NULL || data == NULL) return IMU_ERROR;
    ICM42688_SPI_Config_t *cfg = (ICM42688_SPI_Config_t *)handle->context;

    uint8_t buf[14];
    if (ICM42688_ReadBlock(cfg, ICM42688_REG_TEMP_DATA1, buf, sizeof(buf)) != IMU_OK)
        return IMU_ERROR;

    int16_t temp_raw = (int16_t)((buf[0] << 8) | buf[1]);
    data->temperature = (float)temp_raw * ICM42688_TEMP_SCALE + ICM42688_TEMP_OFFSET;

    int16_t ax = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t ay = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t az = (int16_t)((buf[6] << 8) | buf[7]);
    data->accel[0] = (float)ax * ICM42688_ACCEL_SCALE;
    data->accel[1] = (float)ay * ICM42688_ACCEL_SCALE;
    data->accel[2] = (float)az * ICM42688_ACCEL_SCALE;

    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
    data->gyro[0] = (float)gx * ICM42688_GYRO_SCALE;
    data->gyro[1] = (float)gy * ICM42688_GYRO_SCALE;
    data->gyro[2] = (float)gz * ICM42688_GYRO_SCALE;

    if (raw != NULL) {
        raw->gyro[0]  = gx;
        raw->gyro[1]  = gy;
        raw->gyro[2]  = gz;
        raw->accel[0] = ax;
        raw->accel[1] = ay;
        raw->accel[2] = az;
        raw->temp     = temp_raw;
    }

    return IMU_OK;
}
