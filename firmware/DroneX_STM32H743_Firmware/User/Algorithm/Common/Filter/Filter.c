#include "Algorithm/Common/Filter/Filter.h"
#include <math.h>
#include <string.h>

static int filter_is_valid_biquad_param(float sample_hz, float freq_hz, float q)
{
    if (sample_hz <= 0.0f || freq_hz <= 0.0f || q <= 0.0f) {
        return 0;
    }
    if (freq_hz >= (0.5f * sample_hz)) {
        return 0;
    }
    return 1;
}

static void filter_normalize_biquad(Filter_BiquadCoeff_t *coeff,
    float b0, float b1, float b2, float a0, float a1, float a2)
{
    coeff->b0 = b0 / a0;
    coeff->b1 = b1 / a0;
    coeff->b2 = b2 / a0;
    coeff->a1 = a1 / a0;
    coeff->a2 = a2 / a0;
}

int Filter_InitFIR(Filter_t *filter, const float *coeffs, uint16_t tap_count, float *state_buffer)
{
    if (filter == NULL || coeffs == NULL || state_buffer == NULL) {
        return FILTER_ERR_NULL;
    }
    if (tap_count == 0U) {
        return FILTER_ERR_PARAM;
    }

    memset(filter, 0, sizeof(*filter));
    filter->kind = FILTER_KIND_FIR;
    filter->obj.fir.coeffs = coeffs;
    filter->obj.fir.state = state_buffer;
    filter->obj.fir.tap_count = tap_count;
    memset(state_buffer, 0, (size_t)tap_count * sizeof(float));
    return FILTER_OK;
}

int Filter_InitIIR1(Filter_t *filter, Filter_Shape_t shape, float tau_s)
{
    if (filter == NULL) {
        return FILTER_ERR_NULL;
    }
    if ((shape != FILTER_SHAPE_LOW_PASS) && (shape != FILTER_SHAPE_HIGH_PASS)) {
        return FILTER_ERR_PARAM;
    }
    if (tau_s < 0.0f) {
        return FILTER_ERR_PARAM;
    }

    memset(filter, 0, sizeof(*filter));
    filter->kind = FILTER_KIND_IIR1;
    filter->obj.iir1.shape = shape;
    filter->obj.iir1.tau_s = tau_s;
    return FILTER_OK;
}

int Filter_SetIIR1TimeConstant(Filter_t *filter, float tau_s)
{
    if (filter == NULL) {
        return FILTER_ERR_NULL;
    }
    if (filter->kind != FILTER_KIND_IIR1) {
        return FILTER_ERR_NOT_INIT;
    }
    if (tau_s < 0.0f) {
        return FILTER_ERR_PARAM;
    }

    filter->obj.iir1.tau_s = tau_s;
    return FILTER_OK;
}

int Filter_InitBiquadCascade(Filter_t *filter,
    const Filter_BiquadCoeff_t *coeffs,
    Filter_BiquadState_t *state_buffer,
    uint8_t section_count)
{
    if (filter == NULL || coeffs == NULL || state_buffer == NULL) {
        return FILTER_ERR_NULL;
    }
    if (section_count == 0U) {
        return FILTER_ERR_PARAM;
    }

    memset(filter, 0, sizeof(*filter));
    filter->kind = FILTER_KIND_BIQUAD_CASCADE;
    filter->obj.biquad.coeffs = coeffs;
    filter->obj.biquad.state = state_buffer;
    filter->obj.biquad.section_count = section_count;
    memset(state_buffer, 0, (size_t)section_count * sizeof(Filter_BiquadState_t));
    return FILTER_OK;
}

int Filter_DesignBiquadLowPass(Filter_BiquadCoeff_t *coeff, float sample_hz, float cutoff_hz, float q)
{
    float w0;
    float cos_w0;
    float sin_w0;
    float alpha;
    float a0;

    if (coeff == NULL) {
        return FILTER_ERR_NULL;
    }
    if (!filter_is_valid_biquad_param(sample_hz, cutoff_hz, q)) {
        return FILTER_ERR_PARAM;
    }

    w0 = 2.0f * FILTER_PI_F * cutoff_hz / sample_hz;
    cos_w0 = cosf(w0);
    sin_w0 = sinf(w0);
    alpha = sin_w0 / (2.0f * q);
    a0 = 1.0f + alpha;

    filter_normalize_biquad(coeff,
        (1.0f - cos_w0) * 0.5f,
        1.0f - cos_w0,
        (1.0f - cos_w0) * 0.5f,
        a0,
        -2.0f * cos_w0,
        1.0f - alpha);
    return FILTER_OK;
}

int Filter_DesignBiquadHighPass(Filter_BiquadCoeff_t *coeff, float sample_hz, float cutoff_hz, float q)
{
    float w0;
    float cos_w0;
    float sin_w0;
    float alpha;
    float a0;

    if (coeff == NULL) {
        return FILTER_ERR_NULL;
    }
    if (!filter_is_valid_biquad_param(sample_hz, cutoff_hz, q)) {
        return FILTER_ERR_PARAM;
    }

    w0 = 2.0f * FILTER_PI_F * cutoff_hz / sample_hz;
    cos_w0 = cosf(w0);
    sin_w0 = sinf(w0);
    alpha = sin_w0 / (2.0f * q);
    a0 = 1.0f + alpha;

    filter_normalize_biquad(coeff,
        (1.0f + cos_w0) * 0.5f,
        -(1.0f + cos_w0),
        (1.0f + cos_w0) * 0.5f,
        a0,
        -2.0f * cos_w0,
        1.0f - alpha);
    return FILTER_OK;
}

int Filter_DesignBiquadBandPass(Filter_BiquadCoeff_t *coeff, float sample_hz, float center_hz, float q)
{
    float w0;
    float cos_w0;
    float sin_w0;
    float alpha;
    float a0;

    if (coeff == NULL) {
        return FILTER_ERR_NULL;
    }
    if (!filter_is_valid_biquad_param(sample_hz, center_hz, q)) {
        return FILTER_ERR_PARAM;
    }

    w0 = 2.0f * FILTER_PI_F * center_hz / sample_hz;
    cos_w0 = cosf(w0);
    sin_w0 = sinf(w0);
    alpha = sin_w0 / (2.0f * q);
    a0 = 1.0f + alpha;

    filter_normalize_biquad(coeff,
        alpha,
        0.0f,
        -alpha,
        a0,
        -2.0f * cos_w0,
        1.0f - alpha);
    return FILTER_OK;
}

int Filter_DesignBiquadBandStop(Filter_BiquadCoeff_t *coeff, float sample_hz, float center_hz, float q)
{
    float w0;
    float cos_w0;
    float sin_w0;
    float alpha;
    float a0;

    if (coeff == NULL) {
        return FILTER_ERR_NULL;
    }
    if (!filter_is_valid_biquad_param(sample_hz, center_hz, q)) {
        return FILTER_ERR_PARAM;
    }

    w0 = 2.0f * FILTER_PI_F * center_hz / sample_hz;
    cos_w0 = cosf(w0);
    sin_w0 = sinf(w0);
    alpha = sin_w0 / (2.0f * q);
    a0 = 1.0f + alpha;

    filter_normalize_biquad(coeff,
        1.0f,
        -2.0f * cos_w0,
        1.0f,
        a0,
        -2.0f * cos_w0,
        1.0f - alpha);
    return FILTER_OK;
}

void Filter_Reset(Filter_t *filter)
{
    uint8_t i;

    if (filter == NULL) {
        return;
    }

    if (filter->kind == FILTER_KIND_FIR) {
        memset(filter->obj.fir.state, 0, (size_t)filter->obj.fir.tap_count * sizeof(float));
        filter->obj.fir.index = 0U;
        return;
    }
    if (filter->kind == FILTER_KIND_IIR1) {
        filter->obj.iir1.x_prev = 0.0f;
        filter->obj.iir1.y_prev = 0.0f;
        filter->obj.iir1.initialized = 0U;
        return;
    }
    if (filter->kind == FILTER_KIND_BIQUAD_CASCADE) {
        for (i = 0U; i < filter->obj.biquad.section_count; i++) {
            filter->obj.biquad.state[i].d1 = 0.0f;
            filter->obj.biquad.state[i].d2 = 0.0f;
        }
    }
}

int Filter_Process(Filter_t *filter, float input, float dt_s, float *output)
{
    if (filter == NULL || output == NULL) {
        return FILTER_ERR_NULL;
    }

    if (filter->kind == FILTER_KIND_FIR) {
        Filter_FIR_t *fir = &filter->obj.fir;
        float y = 0.0f;
        uint16_t tap;
        uint16_t idx;

        fir->state[fir->index] = input;
        idx = fir->index;
        for (tap = 0U; tap < fir->tap_count; tap++) {
            y += fir->coeffs[tap] * fir->state[idx];
            if (idx == 0U) {
                idx = (uint16_t)(fir->tap_count - 1U);
            } else {
                idx--;
            }
        }

        fir->index++;
        if (fir->index >= fir->tap_count) {
            fir->index = 0U;
        }
        *output = y;
        return FILTER_OK;
    }

    if (filter->kind == FILTER_KIND_IIR1) {
        Filter_IIR1_t *iir1 = &filter->obj.iir1;
        float y;

        if ((iir1->initialized == 0U) || (iir1->tau_s <= 0.0f)) {
            if (iir1->shape == FILTER_SHAPE_HIGH_PASS) {
                y = (iir1->tau_s <= 0.0f) ? input : 0.0f;
            } else {
                y = input;
            }
            iir1->x_prev = input;
            iir1->y_prev = y;
            iir1->initialized = 1U;
            *output = y;
            return FILTER_OK;
        }

        if (dt_s <= 0.0f) {
            *output = iir1->y_prev;
            return FILTER_OK;
        }

        if (iir1->shape == FILTER_SHAPE_LOW_PASS) {
            float alpha = dt_s / (iir1->tau_s + dt_s);
            y = iir1->y_prev + alpha * (input - iir1->y_prev);
        } else {
            float alpha = iir1->tau_s / (iir1->tau_s + dt_s);
            y = alpha * (iir1->y_prev + input - iir1->x_prev);
        }

        iir1->x_prev = input;
        iir1->y_prev = y;
        *output = y;
        return FILTER_OK;
    }

    if (filter->kind == FILTER_KIND_BIQUAD_CASCADE) {
        Filter_BiquadCascade_t *biquad = &filter->obj.biquad;
        float stage_in = input;
        uint8_t i;

        for (i = 0U; i < biquad->section_count; i++) {
            const Filter_BiquadCoeff_t *c = &biquad->coeffs[i];
            Filter_BiquadState_t *s = &biquad->state[i];
            float y = c->b0 * stage_in + s->d1;

            s->d1 = c->b1 * stage_in - c->a1 * y + s->d2;
            s->d2 = c->b2 * stage_in - c->a2 * y;
            stage_in = y;
        }

        *output = stage_in;
        return FILTER_OK;
    }

    return FILTER_ERR_NOT_INIT;
}
