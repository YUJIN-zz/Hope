#pragma once
#define _USE_MATH_DEFINES
#include "esp_err.h"
#include "dsps_fft2r.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GCC_PHAT_N_BUFFER  1024
#define GCC_PHAT_N_FFT     2048
#define GCC_PHAT_FS        16000

typedef enum {
    RIGHT = 0,
    FRONT = 1,
    LEFT  = 2,
    BACK  = 3
} Direction;

esp_err_t gcc_phat_init(void);

float gcc_phat_compute(const float *mic1sig, const float *mic2sig);

float compute_TDOA(float tau_x, float tau_y);

Direction get_direction(float theta);
