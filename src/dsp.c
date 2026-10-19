/* dsp.c — Welch spectra on esp-dsp's optimized float FFT (host builds use a
 * portable radix-2 so tools/test_dsp.c can check scaling and orientation).
 * Power coding follows ESPARGOS/esp-sdr main/common/spectrum_math.h. */
#include "dsp.h"
#include <math.h>
#include <string.h>

#define DSP_PI 3.14159265358979f

static float fft_buf[2 * DSP_MAX_FFT] __attribute__((aligned(16)));
static float window[DSP_MAX_FFT];
static unsigned window_n;
static float scratch[DSP_MAX_FFT];
float *dsp_scratch(void) { return scratch; }

#ifdef ESP_PLATFORM
#include "dsps_fft2r.h"
static float fft_table[DSP_MAX_FFT] __attribute__((aligned(16)));
void dsp_init(void) { dsps_fft2r_init_fc32(fft_table, DSP_MAX_FFT); }
static void fft(float *x, unsigned n) {
    dsps_fft2r_fc32(x, n);
    dsps_bit_rev_fc32(x, n);
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

void dsp_welch(const uint32_t *words, unsigned n, unsigned segments, unsigned flags, float *pow,
               dsp_dc_t *dc) {
    if (n < 4 || n > DSP_MAX_FFT || (n & (n - 1)) || !segments) return;
    window_setup(n);
    float mi = 0, mq = 0;
    if (flags & DSP_REMOVE_MEAN) {
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
    for (unsigned s = 0; s < segments; s++) {
        const uint32_t *w = words + s * n;
        for (unsigned j = 0; j < n; j++) {
            fft_buf[2 * j] = ((float)dsp_i(w[j]) - mi) * window[j];
            fft_buf[2 * j + 1] = qsign * ((float)dsp_q(w[j]) - mq) * window[j];
        }
        fft(fft_buf, n);
        if (dc) dc_apply(dc, fft_buf, n);
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
