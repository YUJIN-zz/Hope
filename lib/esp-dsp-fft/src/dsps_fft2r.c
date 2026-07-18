/*
 * SPDX-License-Identifier: Apache-2.0
 * ESP-DSP algorithm/API compatibility: Copyright Espressif Systems.
 * Project-specific minimal implementation and packaging modifications.
 *
 * FFT-only, portable subset compatible with the ESP-DSP dsps_fft2r API.
 * The radix-2 butterfly and coefficient convention follow ESP-DSP's ANSI
 * implementation. No examples, tests, fixed-point FFT, or assembly kernels
 * are included in this PlatformIO library.
 */

#include "dsps_fft2r.h"

#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float *s_twiddle = NULL;
static int s_twiddle_size = 0;
static int s_owns_twiddle = 0;

static int is_power_of_two(int value)
{
    return value > 0 && (value & (value - 1)) == 0;
}

esp_err_t dsps_fft2r_init_fc32(float *fft_table_buff, int table_size)
{
    if (!is_power_of_two(table_size) || table_size < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_twiddle != NULL) {
        return s_twiddle_size >= table_size ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    if (fft_table_buff != NULL) {
        s_twiddle = fft_table_buff;
        s_owns_twiddle = 0;
    }
    else {
        s_twiddle = (float *)malloc((size_t)table_size * sizeof(float));
        if (s_twiddle == NULL) {
            return ESP_ERR_NO_MEM;
        }
        s_owns_twiddle = 1;
    }

    s_twiddle_size = table_size;

    const float angle_step = (float)(2.0 * M_PI) / (float)table_size;
    for (int i = 0; i < table_size / 2; ++i) {
        s_twiddle[2 * i] = cosf((float)i * angle_step);
        s_twiddle[2 * i + 1] = sinf((float)i * angle_step);
    }

    return ESP_OK;
}

void dsps_fft2r_deinit_fc32(void)
{
    if (s_owns_twiddle) {
        free(s_twiddle);
    }

    s_twiddle = NULL;
    s_twiddle_size = 0;
    s_owns_twiddle = 0;
}

esp_err_t dsps_fft2r_fc32_ansi(float *data, int n)
{
    if (data == NULL || !is_power_of_two(n)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_twiddle == NULL || s_twiddle_size < n) {
        return ESP_ERR_INVALID_STATE;
    }

    int coefficient_step = s_twiddle_size / n;
    int groups = 1;

    for (int half_size = n / 2; half_size > 0; half_size >>= 1) {
        int first = 0;

        for (int group = 0; group < groups; ++group) {
            int coefficient_index = group * coefficient_step;
            float cosine = s_twiddle[2 * coefficient_index];
            float sine = s_twiddle[2 * coefficient_index + 1];

            for (int i = 0; i < half_size; ++i) {
                int second = first + half_size;

                float second_real = data[2 * second];
                float second_imag = data[2 * second + 1];
                float rotated_real = cosine * second_real + sine * second_imag;
                float rotated_imag = cosine * second_imag - sine * second_real;

                float first_real = data[2 * first];
                float first_imag = data[2 * first + 1];

                data[2 * second] = first_real - rotated_real;
                data[2 * second + 1] = first_imag - rotated_imag;
                data[2 * first] = first_real + rotated_real;
                data[2 * first + 1] = first_imag + rotated_imag;
                ++first;
            }

            first += half_size;
        }

        groups <<= 1;
    }

    return ESP_OK;
}

esp_err_t dsps_bit_rev_fc32_ansi(float *data, int n)
{
    if (data == NULL || !is_power_of_two(n)) {
        return ESP_ERR_INVALID_ARG;
    }

    int reversed = 0;

    for (int i = 1; i < n - 1; ++i) {
        int bit = n >> 1;

        while (reversed & bit) {
            reversed ^= bit;
            bit >>= 1;
        }
        reversed ^= bit;

        if (i < reversed) {
            float real = data[2 * i];
            float imag = data[2 * i + 1];

            data[2 * i] = data[2 * reversed];
            data[2 * i + 1] = data[2 * reversed + 1];
            data[2 * reversed] = real;
            data[2 * reversed + 1] = imag;
        }
    }

    return ESP_OK;
}
