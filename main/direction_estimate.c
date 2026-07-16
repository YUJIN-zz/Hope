#include "direction_estimate.h"

esp_err_t gcc_phat_init(void) {
    return dsps_fft2r_init_fc32(NULL, GCC_PHAT_N_FFT);
}

// 마이크 사이의 시간차를 계산하는 함수
// mic1: leftmic, southmic
// mic2: rightmic, northmic
float gcc_phat_compute(const float *mic1sig, const float *mic2sigS) {

    // [1] Zero-pad into complex interleaved buffers (size = N_FFT * 2 floats)
    static float buf_mic1[GCC_PHAT_N_FFT * 2];
    static float buf_mic2[GCC_PHAT_N_FFT * 2];
    static float buf_result[GCC_PHAT_N_FFT * 2];

    for (int i = 0; i < GCC_PHAT_N_FFT; i++) {
        buf_mic1[i*2]   = (i < GCC_PHAT_N_BUFFER) ? mic1sig[i] : 0.0f;
        buf_mic1[i*2+1] = 0.0f;
        buf_mic2[i*2]   = (i < GCC_PHAT_N_BUFFER) ? mic2sig[i] : 0.0f;
        buf_mic2[i*2+1] = 0.0f;
    }

    // [2] FFT
    dsps_fft2r_fc32(buf_mic1, GCC_PHAT_N_FFT);
    dsps_bit_rev_fc32(buf_mic1, GCC_PHAT_N_FFT);
    dsps_fft2r_fc32(buf_mic2, GCC_PHAT_N_FFT);
    dsps_bit_rev_fc32(buf_mic2, GCC_PHAT_N_FFT);

    // [3] Cross-spectrum + PHAT normalization
    for (int i = 0; i < GCC_PHAT_N_FFT; i++) {
        float re1 = buf_mic1[i*2],   im1 = buf_mic1[i*2+1];
        float re2 = buf_mic2[i*2],   im2 = buf_mic2[i*2+1];

        // R[k] = X1[k] * conj(X2[k])
        float re_r = re1*re2 + im1*im2;
        float im_r = im1*re2 - re1*im2;

        float mag = sqrtf(re_r*re_r + im_r*im_r);
        if (mag > 0.0f) {
            buf_result[i*2]   = re_r / mag;
            buf_result[i*2+1] = im_r / mag;
        } else {
            buf_result[i*2]   = 0.0f;
            buf_result[i*2+1] = 0.0f;
        }
    }

    // [4] IFFT: conjugate → FFT → conjugate → scale
    for (int i = 0; i < GCC_PHAT_N_FFT; i++) {
        buf_result[i*2+1] = -buf_result[i*2+1];
    }

    dsps_fft2r_fc32(buf_result, GCC_PHAT_N_FFT);
    dsps_bit_rev_fc32(buf_result, GCC_PHAT_N_FFT);
    float inv_n = 1.0f / (float)GCC_PHAT_N_FFT;

    for (int i = 0; i < GCC_PHAT_N_FFT; i++) {
        buf_result[i*2]   =  buf_result[i*2]   * inv_n;
        buf_result[i*2+1] = -buf_result[i*2+1] * inv_n;
    }

    // [5] Find max correlation index
    int max_index = 0;
    float max_value = buf_result[0]*buf_result[0] + buf_result[1]*buf_result[1];

    for (int i = 1; i < GCC_PHAT_N_FFT; i++) {
        float value = buf_result[i*2]*buf_result[i*2] + buf_result[i*2+1]*buf_result[i*2+1];
        if (value > max_value) {
            max_value = value;
            max_index = i;
        }
    }

    // [6] Convert index to TDOA in seconds
    float tau;
    if (max_index <= GCC_PHAT_N_FFT / 2)
        tau = (float)max_index / (float)GCC_PHAT_FS;
    else
        tau = (float)(max_index - GCC_PHAT_N_FFT) / (float)GCC_PHAT_FS;

    return tau;
}

float compute_TDOA(float tau_x, float tau_y) {
    return atan2f(tau_y, tau_x);
}

Direction get_direction(float theta) {
    if (theta >= -M_PI/4 && theta < M_PI/4) {
        return RIGHT;
    } else if (theta >= M_PI/4 && theta < 3*M_PI/4) {
        return FRONT;
    } else if (theta >= -3*M_PI/4 && theta < -M_PI/4) {
        return BACK;
    } else {
        return LEFT;
    }
}
