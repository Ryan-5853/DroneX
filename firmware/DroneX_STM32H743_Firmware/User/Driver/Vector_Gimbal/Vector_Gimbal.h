#ifndef __VECTOR_GIMBAL_H__
#define __VECTOR_GIMBAL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VECTOR_GIMBAL_AXIS_0 = 0, /* PD14 -> TIM4_CH3 */
    VECTOR_GIMBAL_AXIS_1 = 1, /* PD15 -> TIM4_CH4 */
    VECTOR_GIMBAL_AXIS_COUNT
} VectorGimbal_Axis_t;

typedef enum {
    VECTOR_GIMBAL_OK = 0,
    VECTOR_GIMBAL_ERR_PARAM,
    VECTOR_GIMBAL_ERR_HAL
} VectorGimbal_Status_t;

void Vector_Gimbal_Init(void);
VectorGimbal_Status_t Vector_Gimbal_Set(VectorGimbal_Axis_t axis, int16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __VECTOR_GIMBAL_H__ */
