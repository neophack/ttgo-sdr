/* dsp.c — Welch spectra on esp-dsp's optimized float FFT (host builds use a
 * portable radix-2 so tools/test_dsp.c can check scaling and orientation).
 * Power coding follows ESPARGOS/esp-sdr main/common/spectrum_math.h. */
#include "dsp.h"
#include <math.h>
#include <string.h>

/* The project builds at -Os; the per-sample loops here are worth -O2. */
#pragma GCC optimize("O2")

#define DSP_PI 3.14159265358979f

static float fft_buf[2 * DSP_MAX_FFT] __attribute__((aligned(16)));
static float window[DSP_MAX_FFT];
static unsigned window_n;
static float scratch[DSP_MAX_FFT];
float *dsp_scratch(void) { return scratch; }

#ifdef ESP_PLATFORM
#include "dsps_fft2r.h"
#include "dsps_fft4r.h"
/* The scans' 256-point FFTs (4^4) use esp-dsp's radix-4 kernel (a 2048-word
 * snapshot's Welch spectrum: 1.4 ms -> 0.9 ms); it must match radix-2 on a
 * test vector at boot or every size stays on radix-2. */
#define FFT4_N 256u
static float fft_table[DSP_MAX_FFT] __attribute__((aligned(16)));
static bool fft4_ok;
static void fft(float *x, unsigned n) {
    if (n == FFT4_N && fft4_ok) {
        dsps_fft4r_fc32(x, n);
        dsps_bit_rev4r_fc32(x, n);
    } else {
        dsps_fft2r_fc32(x, n);
        dsps_bit_rev_fc32(x, n);
    }
}
void dsp_init(void) {
    dsps_fft2r_init_fc32(fft_table, DSP_MAX_FFT);
    if (dsps_fft4r_init_fc32(NULL, FFT4_N) != ESP_OK) return;
    float *a = fft_buf, *b = scratch; /* 2 x 256 complex = both buffers */
    uint32_t seed = 12345;
    for (unsigned j = 0; j < 2 * FFT4_N; j++) {
        seed = seed * 1103515245u + 12345u;
        a[j] = b[j] = (float)((seed >> 16) & 1023) - 512.0f;
    }
    fft4_ok = true;
    fft(a, FFT4_N);
    fft4_ok = false;
    fft(b, FFT4_N);
    float err = 0, peak = 0;
    for (unsigned j = 0; j < 2 * FFT4_N; j++) {
        err = fmaxf(err, fabsf(a[j] - b[j]));
        peak = fmaxf(peak, fabsf(b[j]));
    }
    fft4_ok = err <= 1e-4f * peak;
}
#else
void dsp_init(void) {}
static void fft(float *x, unsigned n) {
    for (unsigned i = 1, j = 0; i < n; i++) {
        unsigned bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j)
            for (unsigned c = 0; c < 2; c++) {
                float t = x[2 * i + c];
                x[2 * i + c] = x[2 * j + c];
                x[2 * j + c] = t;
            }
    }
    for (unsigned len = 2; len <= n; len <<= 1) {
        float a = -2.0f * DSP_PI / (float)len;
        for (unsigned k = 0; k < len / 2; k++) {
            float wr = cosf(a * k), wi = sinf(a * k);
            for (unsigned b = 0; b < n; b += len) {
                float *p = x + 2 * (b + k), *q = x + 2 * (b + k + len / 2);
                float tr = q[0] * wr - q[1] * wi, ti = q[0] * wi + q[1] * wr;
                q[0] = p[0] - tr;
                q[1] = p[1] - ti;
                p[0] += tr;
                p[1] += ti;
            }
        }
    }
}
#endif

/* Periodic Hann with the esp-sdr scale folded in: 64 * hann / n. */
static void window_setup(unsigned n) {
    if (window_n == n) return;
    const float scale = 64.0f / (float)n;
    for (unsigned j = 0; j < n; j++)
        window[j] = scale * 0.5f * (1.0f - cosf(2.0f * DSP_PI * (float)j / (float)n));
    window_n = n;
}

/* esp-sdr spectrum_dc_apply in natural order: remove the DC bin estimate and
 * undo the Hann window's -1/2 spill into the adjacent bins. */
static void dc_apply(dsp_dc_t *dc, float *x, unsigned n) {
    float dr = x[0], di = x[1];
    if (dc->mode) {
        if (!dc->ready) {
            dc->re = dr;
            dc->im = di;
            dc->ready = true;
        } else {
            dc->re += (dr - dc->re) * (1.0f / 64.0f);
            dc->im += (di - dc->im) * (1.0f / 64.0f);
        }
        dr = dc->re;
        di = dc->im;
    }
    x[0] -= dr;
    x[1] -= di;
    x[2] += dr * 0.5f;
    x[3] += di * 0.5f;
    x[2 * (n - 1)] += dr * 0.5f;
    x[2 * (n - 1) + 1] += di * 0.5f;
}

/* Removing the capture mean m before the window only changes bins 0 and +-1:
 * FFT(w * (x - m)) = FFT(w * x) - m * FFT(w), and FFT(w) of the scaled
 * periodic Hann is exactly {32, -16, -16} there (64/n * {n/2, -n/4, -n/4}).
 * Summed powers take the correction once at the end:
 *   sum_s |X_s - a|^2 = sum_s |X_s|^2 - 2 Re(conj(a) sum_s X_s) + S |a|^2
 * so the mean is accumulated while windowing instead of in a separate pass
 * over the capture. Max-hold and the DC tracker need every segment
 * corrected and keep the pass. */
static void mean_correct(float *pow, unsigned n, unsigned segments, float mr, float mim, const float *xs) {
    static const float hann_dft[3] = {32.0f, -16.0f, -16.0f};
    const unsigned bins[3] = {0, 1, n - 1};
    for (unsigned b = 0; b < 3; b++) {
        const float ar = mr * hann_dft[b], ai = mim * hann_dft[b];
        float p = pow[bins[b]] - 2.0f * (ar * xs[2 * b] + ai * xs[2 * b + 1]) + (float)segments * (ar * ar + ai * ai);
        pow[bins[b]] = p > 0 ? p : 0;
    }
}

void dsp_welch(const uint32_t *words, unsigned n, unsigned segments, unsigned flags, float *pow,
               dsp_dc_t *dc) {
    if (n < 4 || n > DSP_MAX_FFT || (n & (n - 1)) || !segments) return;
    window_setup(n);
    const bool deferred = (flags & DSP_REMOVE_MEAN) && !(flags & DSP_MAX) && !dc;
    float mi = 0, mq = 0;
    if ((flags & DSP_REMOVE_MEAN) && !deferred) {
        int32_t si = 0, sq = 0; /* 16380 * 511 fits easily */
        const unsigned total = n * segments;
        for (unsigned j = 0; j < total; j++) {
            si += dsp_i(words[j]);
            sq += dsp_q(words[j]);
        }
        mi = (float)si / (float)total;
        mq = (float)sq / (float)total;
    }
    const float qsign = (flags & DSP_NEGATE_Q) ? -1.0f : 1.0f;
    int32_t si = 0, sq = 0;
    float xs[6] = {0}; /* sum over segments of X at bins 0, 1, n-1 */
    for (unsigned s = 0; s < segments; s++) {
        const uint32_t *w = words + s * n;
        for (unsigned j = 0; j < n; j++) {
            const uint32_t v = w[j];
            const int vi = dsp_i(v), vq = dsp_q(v);
            si += vi;
            sq += vq;
            fft_buf[2 * j] = ((float)vi - mi) * window[j];
            fft_buf[2 * j + 1] = qsign * ((float)vq - mq) * window[j];
        }
        fft(fft_buf, n);
        if (dc) dc_apply(dc, fft_buf, n);
        if (deferred) {
            xs[0] += fft_buf[0];
            xs[1] += fft_buf[1];
            xs[2] += fft_buf[2];
            xs[3] += fft_buf[3];
            xs[4] += fft_buf[2 * (n - 1)];
            xs[5] += fft_buf[2 * (n - 1) + 1];
        }
        if (flags & DSP_MAX) {
            for (unsigned j = 0; j < n; j++) {
                float p = fft_buf[2 * j] * fft_buf[2 * j] + fft_buf[2 * j + 1] * fft_buf[2 * j + 1];
                if (p > pow[j]) pow[j] = p;
            }
        } else {
            for (unsigned j = 0; j < n; j++)
                pow[j] += fft_buf[2 * j] * fft_buf[2 * j] + fft_buf[2 * j + 1] * fft_buf[2 * j + 1];
        }
    }
    if (deferred) {
        const float total = (float)(n * segments);
        mean_correct(pow, n, segments, (float)si / total, qsign * (float)sq / total, xs);
    }
}

/* ---- esp-sdr spectrum_math.h ---- */
static uint8_t log_power_code(unsigned exponent, uint32_t norm) {
    static const uint16_t log2_q12[17] = {0,    358,  696,  1016, 1319, 1607, 1882, 2144, 2396,
                                          2637, 2869, 3092, 3307, 3515, 3715, 3908, 4096};
    unsigned slot = (norm - 256u) >> 4, frac = norm & 15u;
    uint32_t logarithm =
        (exponent << 12) + log2_q12[slot] + ((log2_q12[slot + 1] - log2_q12[slot]) * frac + 8) / 16;
    unsigned code = (logarithm * 1541u + (1u << 19)) >> 20; /* 20 log10(power), rounded */
    return code > 255 ? 255 : code;
}
uint8_t dsp_power_code(float power) {
    union {
        float f;
        uint32_t u;
    } value = {.f = power};
    if (value.u < 0x3f800000u || value.u > 0x7f800000u) return 0;
    unsigned exponent = (value.u >> 23) - 127u;
    if (exponent >= 43) return 255;
    return log_power_code(exponent, 256u + ((value.u & 0x7fffffu) >> 15));
}

float dsp_dbfs(float p) { return 10.0f * log10f(p > 1e-12f ? p : 1e-12f) - DSP_DBFS_OFFSET; }
