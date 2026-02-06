#ifndef __IMU_INTERFACE_H__
#define __IMU_INTERFACE_H__

typedef enum {
    IMU_OK = 0,
    IMU_ERROR = -1,
} IMU_Status_t;

typedef enum {
    IMU_TYPE_MPU6050 = 0,
    IMU_TYPE_BMI088  = 1,
    IMU_TYPE_ICM42688 = 2,
} IMU_Type_t;

/** IMU 单次采样数据，单位：gyro [rad/s]，accel [m/s^2]，temperature [°C]，机体系。驱动层不做偏置校正，由算法/应用层处理。 */
typedef struct {
    float gyro[3];
    float accel[3];
    float temperature;
} IMU_Data_t;

/** 偏置类型，供算法/应用层做校正时使用（驱动层不提供 get/set_bias） */
typedef struct {
    float gyro_bias[3];
    float accel_bias[3];
} IMU_Bias_t;

/** 具体驱动通过 handle->context 获取本实例的配置，实现多 IMU 隔离 */
typedef struct IMU_Handle IMU_Handle_t;
struct IMU_Handle {
    IMU_Type_t type;
    void *context;   /* 指向类型相关的实例配置，由用户在 Interface_Init 时传入并保持有效 */
    IMU_Status_t (*init)(IMU_Handle_t *handle);
    IMU_Status_t (*read)(IMU_Handle_t *handle, IMU_Data_t *data);
};

/** 绑定类型与实例配置，填充句柄；config 为类型相关（如 ICM42688_Config_t），须在句柄生命周期内有效 */
IMU_Status_t IMU_Interface_Init(IMU_Handle_t *handle, IMU_Type_t type, void *config);

IMU_Status_t IMU_Init(IMU_Handle_t *handle);
IMU_Status_t IMU_Read(IMU_Handle_t *handle, IMU_Data_t *data);

#endif