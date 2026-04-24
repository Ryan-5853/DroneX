#ifndef __VECTOR_GIMBAL_H__
#define __VECTOR_GIMBAL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VECTOR_GIMBAL_AXIS_X = 0, /* X servo */
    VECTOR_GIMBAL_AXIS_Y = 1, /* Y servo */
    VECTOR_GIMBAL_AXIS_0 = VECTOR_GIMBAL_AXIS_X, /* backward-compatible alias */
    VECTOR_GIMBAL_AXIS_1 = VECTOR_GIMBAL_AXIS_Y, /* backward-compatible alias */
    VECTOR_GIMBAL_AXIS_COUNT
} VectorGimbal_Axis_t;

typedef enum {
    VECTOR_GIMBAL_OK = 0,
    VECTOR_GIMBAL_ERR_PARAM,
    VECTOR_GIMBAL_ERR_HAL
} VectorGimbal_Status_t;

void Vector_Gimbal_Init(void);

/* First-order output low-pass time constant. Set 0 to disable filtering. */
void Vector_Gimbal_SetFilterTimeMs(uint16_t time_ms);

/* Raw command interface: value is limited to [-10000, 10000]. */
VectorGimbal_Status_t Vector_Gimbal_SetRaw(VectorGimbal_Axis_t axis, int16_t value);

/* Normalized command interface: value is limited to [-1.0f, 1.0f]. */
VectorGimbal_Status_t Vector_Gimbal_SetNormalized(VectorGimbal_Axis_t axis, float value);

/* Backward-compatible alias for the raw command interface. */
VectorGimbal_Status_t Vector_Gimbal_Set(VectorGimbal_Axis_t axis, int16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __VECTOR_GIMBAL_H__ */
