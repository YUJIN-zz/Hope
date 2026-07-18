// SPDX-License-Identifier: Apache-2.0
// ESP-DSP API compatibility: Copyright Espressif Systems.
// Project-specific minimal implementation and packaging modifications.
//
// Minimal API-compatible subset of Espressif ESP-DSP's dsps_fft2r module.
// Only the floating-point radix-2 functions used by direction_estimate.c
// are exposed here.
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifndef CONFIG_DSP_MAX_FFT_SIZE
#define CONFIG_DSP_MAX_FFT_SIZE 4096
#endif

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t dsps_fft2r_init_fc32(float *fft_table_buff, int table_size);
void dsps_fft2r_deinit_fc32(void);
esp_err_t dsps_fft2r_fc32_ansi(float *data, int n);
esp_err_t dsps_bit_rev_fc32_ansi(float *data, int n);

// Keep the public names used by ESP-DSP callers while selecting the portable
// ANSI implementation. This avoids pulling target-specific assembly sources.
#define dsps_fft2r_fc32(data, n) dsps_fft2r_fc32_ansi((data), (n))
#define dsps_bit_rev_fc32(data, n) dsps_bit_rev_fc32_ansi((data), (n))

#ifdef __cplusplus
}
#endif
