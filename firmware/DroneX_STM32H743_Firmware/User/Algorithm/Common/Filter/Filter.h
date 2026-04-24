#ifndef __ALGORITHM_COMMON_FILTER_H__
#define __ALGORITHM_COMMON_FILTER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FILTER_PI_F
#define FILTER_PI_F 3.14159265358979323846f
#endif

#define FILTER_OK             0
#define FILTER_ERR_NULL      -1
#define FILTER_ERR_PARAM     -2
#define FILTER_ERR_NOT_INIT  -3

typedef enum {
    FILTER_KIND_NONE = 0,
    FILTER_KIND_FIR,
    FILTER_KIND_IIR1,
    FILTER_KIND_BIQUAD_CASCADE
} Filter_Kind_t;

typedef enum {
    FILTER_SHAPE_GENERIC = 0,
    FILTER_SHAPE_LOW_PASS,
    FILTER_SHAPE_HIGH_PASS,
    FILTER_SHAPE_BAND_PASS,
    FILTER_SHAPE_BAND_STOP
} Filter_Shape_t;

typedef struct {
    const float *coeffs;
    float *state;
    uint16_t tap_count;
    uint16_t index;
} Filter_FIR_t;

typedef struct {
    Filter_Shape_t shape;
    float tau_s;
    float x_prev;
    float y_prev;
    uint8_t initialized;
} Filter_IIR1_t;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} Filter_BiquadCoeff_t;

typedef struct {
    float d1;
    float d2;
} Filter_BiquadState_t;

typedef struct {
    const Filter_BiquadCoeff_t *coeffs;
    Filter_BiquadState_t *state;
    uint8_t section_count;
} Filter_BiquadCascade_t;

typedef struct {
    Filter_Kind_t kind;
    union {
        Filter_FIR_t fir;
        Filter_IIR1_t iir1;
        Filter_BiquadCascade_t biquad;
    } obj;
} Filter_t;

int Filter_InitFIR(Filter_t *filter, const float *coeffs, uint16_t tap_count, float *state_buffer);
int Filter_InitIIR1(Filter_t *filter, Filter_Shape_t shape, float tau_s);
int Filter_SetIIR1TimeConstant(Filter_t *filter, float tau_s);
int Filter_InitBiquadCascade(Filter_t *filter,
    const Filter_BiquadCoeff_t *coeffs,
    Filter_BiquadState_t *state_buffer,
    uint8_t section_count);

int Filter_DesignBiquadLowPass(Filter_BiquadCoeff_t *coeff, float sample_hz, float cutoff_hz, float q);
int Filter_DesignBiquadHighPass(Filter_BiquadCoeff_t *coeff, float sample_hz, float cutoff_hz, float q);
int Filter_DesignBiquadBandPass(Filter_BiquadCoeff_t *coeff, float sample_hz, float center_hz, float q);
int Filter_DesignBiquadBandStop(Filter_BiquadCoeff_t *coeff, float sample_hz, float center_hz, float q);

void Filter_Reset(Filter_t *filter);
int Filter_Process(Filter_t *filter, float input, float dt_s, float *output);

#ifdef __cplusplus
}
#endif

#endif /* __ALGORITHM_COMMON_FILTER_H__ */
