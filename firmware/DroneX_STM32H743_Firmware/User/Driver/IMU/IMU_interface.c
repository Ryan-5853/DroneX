/**
 * @file IMU_interface.c
 * @brief IMU 接口层：按类型绑定 context 与函数指针，对外统一 Init/Read。
 */

#include "IMU_interface.h"
#include "ICM42688.h"

IMU_Status_t IMU_Interface_Init(IMU_Handle_t *handle, IMU_Type_t type, void *config)
{
    if (handle == NULL) return IMU_ERROR;

    handle->type    = type;
    handle->context = config;

    switch (type) {
        case IMU_TYPE_ICM42688:
            handle->init = ICM42688_Init;
            handle->read = ICM42688_Read;
            break;
        case IMU_TYPE_MPU6050:
        case IMU_TYPE_BMI088:
        default:
            return IMU_ERROR;
    }
    return IMU_OK;
}

IMU_Status_t IMU_Init(IMU_Handle_t *handle)
{
    if (handle == NULL || handle->init == NULL) return IMU_ERROR;
    return handle->init(handle);
}

IMU_Status_t IMU_Read(IMU_Handle_t *handle, IMU_Data_t *data)
{
    if (handle == NULL || handle->read == NULL || data == NULL) return IMU_ERROR;
    return handle->read(handle, data);
}
