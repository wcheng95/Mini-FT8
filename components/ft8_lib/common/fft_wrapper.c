#include "fft_wrapper.h"
#include <stdlib.h>

int fft_plan_init_with_buffer(fft_plan_t* plan, int nfft, void* buffer, size_t buffer_size)
{
    if (plan == NULL || buffer == NULL)
    {
        return -1;
    }
    plan->nfft = nfft;
    plan->work = buffer;
    plan->work_size = buffer_size;
    size_t needed = 0;
    kiss_fftr_alloc(plan->nfft, 0, NULL, &needed);
    if (needed > buffer_size)
    {
        plan->cfg = NULL;
        return -1;
    }
    plan->cfg = kiss_fftr_alloc(plan->nfft, 0, plan->work, &needed);
    if (plan->cfg == NULL)
    {
        return -1;
    }
    return 0;
}

void fft_plan_free(fft_plan_t* plan)
{
    if (plan == NULL)
    {
        return;
    }
    plan->work = NULL;
    plan->work_size = 0;
    plan->cfg = NULL;
    plan->nfft = 0;
}

void fft_execute(const fft_plan_t* plan, const kiss_fft_scalar* timedata, kiss_fft_cpx* freqdata)
{
    if (plan == NULL || plan->cfg == NULL)
    {
        return;
    }
    kiss_fftr(plan->cfg, timedata, freqdata);
}
