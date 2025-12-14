#ifndef _INCLUDE_FFT_WRAPPER_H_
#define _INCLUDE_FFT_WRAPPER_H_

// Use relative path so Arduino finds it without extra include path
#include "../fft/kiss_fftr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int nfft;
    void* work;
    size_t work_size;
    kiss_fftr_cfg cfg;
} fft_plan_t;

int fft_plan_init_with_buffer(fft_plan_t* plan, int nfft, void* buffer, size_t buffer_size);
void fft_plan_free(fft_plan_t* plan);
void fft_execute(const fft_plan_t* plan, const kiss_fft_scalar* timedata, kiss_fft_cpx* freqdata);

#ifdef __cplusplus
}
#endif

#endif // _INCLUDE_FFT_WRAPPER_H_
