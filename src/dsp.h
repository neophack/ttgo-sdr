/* dsp.h — spectrum math shared by the TFT engine and the esp-sdr protocol.
 *
 * Powers use esp-sdr's normalized FFT domain: X = FFT(IQ10 * 64 * hann) / n,
 * p = |X|^2. A full-scale tone (amplitude 512) gives 10*log10(p) = 84.3 dB,
 * so dBFS = 10*log10(p) - 84.3 for every FFT size, exactly the mapping
 * esp-web-sdr applies to on-chip spectra (app.js specToDbfs).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSP_MAX_FFT 2048u
#define DSP_DBFS_OFFSET 84.3f

enum {
    DSP_NEGATE_Q = 1,    /* conj(I+jQ): positive frequencies above the LO */
    DSP_REMOVE_MEAN = 2, /* subtract the capture's mean I/Q (DC / LO leakage) */
    DSP_MAX = 4,         /* max-hold across segments instead of summing */
};

/* esp-sdr spectrum_dc.h tracker: mode 1 = slow (1/64 per FFT), 0 = per-FFT. */
typedef struct {
    float re, im;
    bool ready;
    unsigned mode;
} dsp_dc_t;

void dsp_init(void);
bool dsp_fast(void); /* the dual-core 256-point path passed its boot self-test */

/* DSP_MAX_FFT floats of power workspace, shared by the engine task's local
 * scans and protocol commands (they never run concurrently). */
float *dsp_scratch(void);

/* Split `segments * n` capture words into consecutive n-point Hann FFTs and
 * add each segment's |X|^2 into pow[] (natural FFT order: bin k = +k*fs/n
 * when DSP_NEGATE_Q is set, -k*fs/n otherwise). dc may be NULL. */
void dsp_welch(const uint32_t *words, unsigned n, unsigned segments, unsigned flags, float *pow,
               dsp_dc_t *dc);

/* dsp_welch in two halves: begin hands the segments to the other core and
 * returns, end joins in and adds the result into pow. In between the words
 * must stay readable and no other dsp_* call may run. */
void dsp_welch_begin(const uint32_t *words, unsigned n, unsigned segments, unsigned flags, dsp_dc_t *dc);
void dsp_welch_end(float *pow);

/* esp-sdr wire code: 20*log10(p) rounded, i.e. 0.5 dB steps (0..255). */
uint8_t dsp_power_code(float p);
float dsp_dbfs(float p);

static inline int dsp_i(uint32_t w) { return (int32_t)(w << 22) >> 22; }
static inline int dsp_q(uint32_t w) { return (int32_t)(w << 12) >> 22; }

#ifdef __cplusplus
}
#endif
