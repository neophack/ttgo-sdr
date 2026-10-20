/* dsp.c — Welch spectra on esp-dsp's optimized float FFT (host builds use a
 * portable radix-2 so tools/test_dsp.c can check scaling and orientation).
 * Power coding follows ESPARGOS/esp-sdr main/common/spectrum_math.h.
 *
 * 256-point Welch (the scans, the screen mirror, SPEC n=256) takes a fast
 * path: radix-4 FFTs without the per-FFT digit reversal (|X|^2 accumulates
 * in the FFT's own order and is permuted once per call, which also absorbs
 * the Q negation: conj(x) only mirrors the power spectrum), with the
 * segments shared between both cores when no DC tracker orders them. */
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
#include <stdlib.h>
#include "dsps_fft2r.h"
#include "dsps_fft4r.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* The scans' 256-point FFTs (4^4) use esp-dsp's radix-4 kernel (a 2048-word
 * snapshot's Welch spectrum: 1.4 ms -> 0.9 ms, ~0.4 ms on the fast path
 * below); it must match radix-2 on a test vector at boot or every size stays
 * on radix-2. */
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
static bool fast_ok;
bool dsp_fast(void) { return fast_ok; }
static void welch_fast(const uint32_t *words, unsigned segments, unsigned flags, float *pow, dsp_dc_t *dc);
static bool fast_selftest(void);
static void worker_task(void *arg);
static TaskHandle_t worker;
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
    if (!fft4_ok) return;
    xTaskCreatePinnedToCore(worker_task, "dsp_worker", 1536, NULL, 4, &worker, 1);
    fast_ok = fast_selftest();
}
#else
void dsp_init(void) {}
bool dsp_fast(void) { return false; }
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
static void dc_apply_at(dsp_dc_t *dc, float *x, unsigned b0, unsigned b1, unsigned bm1) {
    float dr = x[2 * b0], di = x[2 * b0 + 1];
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
    x[2 * b0] -= dr;
    x[2 * b0 + 1] -= di;
    x[2 * b1] += dr * 0.5f;
    x[2 * b1 + 1] += di * 0.5f;
    x[2 * bm1] += dr * 0.5f;
    x[2 * bm1 + 1] += di * 0.5f;
}
static void dc_apply(dsp_dc_t *dc, float *x, unsigned n) { dc_apply_at(dc, x, 0, 1, n - 1); }

/* Removing the capture mean m before the window only changes bins 0 and +-1:
 * FFT(w * (x - m)) = FFT(w * x) - m * FFT(w), and FFT(w) of the scaled
 * periodic Hann is exactly {32, -16, -16} there (64/n * {n/2, -n/4, -n/4}).
 * Summed powers take the correction once at the end:
 *   sum_s |X_s - a|^2 = sum_s |X_s|^2 - 2 Re(conj(a) sum_s X_s) + S |a|^2
 * so the mean is accumulated while windowing instead of in a separate pass
 * over the capture. Max-hold and the DC tracker need every segment
 * corrected and keep the pass. */
static void mean_correct_at(float *pow, const unsigned *bins, unsigned segments, float mr, float mim,
                            const float *xs) {
    static const float hann_dft[3] = {32.0f, -16.0f, -16.0f};
    for (unsigned b = 0; b < 3; b++) {
        const float ar = mr * hann_dft[b], ai = mim * hann_dft[b];
        float p = pow[bins[b]] - 2.0f * (ar * xs[2 * b] + ai * xs[2 * b + 1]) + (float)segments * (ar * ar + ai * ai);
        pow[bins[b]] = p > 0 ? p : 0;
    }
}
static void mean_correct(float *pow, unsigned n, unsigned segments, float mr, float mim, const float *xs) {
    const unsigned bins[3] = {0, 1, n - 1};
    mean_correct_at(pow, bins, segments, mr, mim, xs);
}

void dsp_welch(const uint32_t *words, unsigned n, unsigned segments, unsigned flags, float *pow,
               dsp_dc_t *dc) {
    if (n < 4 || n > DSP_MAX_FFT || (n & (n - 1)) || !segments) return;
#ifdef ESP_PLATFORM
    if (n == FFT4_N && fast_ok) {
        welch_fast(words, segments, flags, pow, dc);
        return;
    }
#endif
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

#ifdef ESP_PLATFORM
/* ---- 256-point fast path ----
 * Natural bin k of an FFT left in radix-4 digit-reversed order sits at
 * position fast_src[k] (read off dsps_bit_rev4r_fc32 once at boot). Each
 * core works in a lane of its own inside fft_buf (6 x 256 of its 4096
 * floats; the radix-2 path that needs the rest never runs meanwhile): FFT
 * buffer, |X|^2 accumulator, the mean-correction sums. Segments are claimed one at a time under job_mux;
 * the caller closes the job when none are left and waits out the worker's
 * segment in flight, so a worker that wakes late only ever claims from a job
 * that is open to it. */
static uint8_t fast_src[FFT4_N];

typedef struct {
    float *buf, *acc;
    float xs[6]; /* sum over segments of X at natural bins 0, 1, n-1 */
    int32_t si, sq;
} lane_t;
static lane_t lanes[2] = {{fft_buf, fft_buf + 4 * FFT4_N}, {fft_buf + 2 * FFT4_N, fft_buf + 5 * FFT4_N}};
static struct {
    const uint32_t *words;
    dsp_dc_t *dc;
    float mi, mq;
    unsigned segments, next, flags;
    bool open, shared, busy, max, deferred;
} job;
static portMUX_TYPE job_mux = portMUX_INITIALIZER_UNLOCKED;

static void fast_segment(lane_t *L, const uint32_t *w) {
    float *buf = L->buf;
    const float mi = job.mi, mq = job.mq;
    int32_t si = 0, sq = 0;
    /* unrolled by 4: independent chains keep the FPU pipeline full */
    for (unsigned j = 0; j < FFT4_N; j += 4) {
        const uint32_t v0 = w[j], v1 = w[j + 1], v2 = w[j + 2], v3 = w[j + 3];
        const int i0 = dsp_i(v0), i1 = dsp_i(v1), i2 = dsp_i(v2), i3 = dsp_i(v3);
        const int q0 = dsp_q(v0), q1 = dsp_q(v1), q2 = dsp_q(v2), q3 = dsp_q(v3);
        si += i0 + i1 + i2 + i3;
        sq += q0 + q1 + q2 + q3;
        const float h0 = window[j], h1 = window[j + 1], h2 = window[j + 2], h3 = window[j + 3];
        float *b = buf + 2 * j;
        b[0] = ((float)i0 - mi) * h0;
        b[1] = ((float)q0 - mq) * h0;
        b[2] = ((float)i1 - mi) * h1;
        b[3] = ((float)q1 - mq) * h1;
        b[4] = ((float)i2 - mi) * h2;
        b[5] = ((float)q2 - mq) * h2;
        b[6] = ((float)i3 - mi) * h3;
        b[7] = ((float)q3 - mq) * h3;
    }
    L->si += si;
    L->sq += sq;
    dsps_fft4r_fc32(buf, FFT4_N);
    const unsigned b0 = fast_src[0], b1 = fast_src[1], bm1 = fast_src[FFT4_N - 1];
    if (job.dc) dc_apply_at(job.dc, buf, b0, b1, bm1);
    if (job.deferred) {
        L->xs[0] += buf[2 * b0];
        L->xs[1] += buf[2 * b0 + 1];
        L->xs[2] += buf[2 * b1];
        L->xs[3] += buf[2 * b1 + 1];
        L->xs[4] += buf[2 * bm1];
        L->xs[5] += buf[2 * bm1 + 1];
    }
    float *acc = L->acc;
    if (job.max) {
        for (unsigned j = 0; j < FFT4_N; j++) {
            const float p = buf[2 * j] * buf[2 * j] + buf[2 * j + 1] * buf[2 * j + 1];
            if (p > acc[j]) acc[j] = p;
        }
    } else {
        for (unsigned j = 0; j < FFT4_N; j += 4) {
            const float *b = buf + 2 * j;
            const float p0 = b[0] * b[0] + b[1] * b[1], p1 = b[2] * b[2] + b[3] * b[3];
            const float p2 = b[4] * b[4] + b[5] * b[5], p3 = b[6] * b[6] + b[7] * b[7];
            acc[j] += p0;
            acc[j + 1] += p1;
            acc[j + 2] += p2;
            acc[j + 3] += p3;
        }
    }
}

static bool claim(unsigned *seg, bool by_worker) {
    taskENTER_CRITICAL(&job_mux);
    bool ok = job.open && (!by_worker || job.shared) && job.next < job.segments;
    if (ok) {
        *seg = job.next++;
        if (by_worker) job.busy = true;
    }
    taskEXIT_CRITICAL(&job_mux);
    return ok;
}

static void worker_task(void *arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        unsigned s;
        while (claim(&s, true)) {
            fast_segment(&lanes[1], job.words + s * FFT4_N);
            taskENTER_CRITICAL(&job_mux);
            job.busy = false;
            taskEXIT_CRITICAL(&job_mux);
        }
    }
}

/* Post a job: the worker starts on it while the caller does other work
 * (fast_end joins in). The caller must not touch fft_buf until fast_end. */
static void fast_begin(const uint32_t *words, unsigned segments, unsigned flags, dsp_dc_t *dc) {
    const unsigned n = FFT4_N;
    window_setup(n);
    const bool max = flags & DSP_MAX;
    const bool deferred = (flags & DSP_REMOVE_MEAN) && !max && !dc;
    float mi = 0, mq = 0;
    if ((flags & DSP_REMOVE_MEAN) && !deferred) {
        int32_t si = 0, sq = 0;
        const unsigned total = n * segments;
        for (unsigned j = 0; j < total; j++) {
            si += dsp_i(words[j]);
            sq += dsp_q(words[j]);
        }
        mi = (float)si / (float)total;
        mq = (float)sq / (float)total;
    }
    for (unsigned l = 0; l < 2; l++) {
        memset(lanes[l].acc, 0, n * sizeof(float));
        memset(lanes[l].xs, 0, sizeof(lanes[l].xs));
        lanes[l].si = lanes[l].sq = 0;
    }
    /* the DC tracker is a running average over the segments in order */
    const bool shared = !dc && segments > 1 && worker;
    taskENTER_CRITICAL(&job_mux);
    job.words = words;
    job.dc = dc;
    job.mi = mi;
    job.mq = mq;
    job.segments = segments;
    job.next = 0;
    job.max = max;
    job.deferred = deferred;
    job.flags = flags;
    job.shared = shared;
    job.busy = false;
    job.open = true;
    taskEXIT_CRITICAL(&job_mux);
    if (shared) xTaskNotifyGive(worker);
}

static void fast_end(float *pow) {
    const unsigned n = FFT4_N, segments = job.segments, flags = job.flags;
    const bool max = job.max, deferred = job.deferred;
    const uint32_t *words = job.words;
    unsigned s;
    while (claim(&s, false)) fast_segment(&lanes[0], words + s * n);
    bool busy;
    taskENTER_CRITICAL(&job_mux);
    job.open = false;
    busy = job.busy;
    taskEXIT_CRITICAL(&job_mux);
    while (busy) { /* at most the one segment in flight, ~50 us */
        taskENTER_CRITICAL(&job_mux);
        busy = job.busy;
        taskEXIT_CRITICAL(&job_mux);
    }
    float *acc = lanes[0].acc;
    const float *acc1 = lanes[1].acc;
    if (max)
        for (unsigned j = 0; j < n; j++) acc[j] = fmaxf(acc[j], acc1[j]);
    else
        for (unsigned j = 0; j < n; j++) acc[j] += acc1[j];
    if (deferred) {
        float xs[6];
        for (unsigned k = 0; k < 6; k++) xs[k] = lanes[0].xs[k] + lanes[1].xs[k];
        const float total = (float)(n * segments);
        const unsigned bins[3] = {fast_src[0], fast_src[1], fast_src[n - 1]};
        mean_correct_at(acc, bins, segments, (float)(lanes[0].si + lanes[1].si) / total,
                        (float)(lanes[0].sq + lanes[1].sq) / total, xs);
    }
    /* conj(x) mirrors the power spectrum: |FFT(conj x)[k]| = |FFT(x)[-k]| */
    const unsigned mirror = (flags & DSP_NEGATE_Q) ? n - 1 : 0;
    for (unsigned k = 0; k < n; k++) {
        const float p = acc[fast_src[mirror ? (n - k) & mirror : k]];
        if (max) {
            if (p > pow[k]) pow[k] = p;
        } else
            pow[k] += p;
    }
}

static void welch_fast(const uint32_t *words, unsigned segments, unsigned flags, float *pow, dsp_dc_t *dc) {
    fast_begin(words, segments, flags, dc);
    fast_end(pow);
}

/* The fast path must reproduce the reference path (radix-4 + digit reversal,
 * per-FFT Q negation) for every flag combination the callers use. */
static bool fast_selftest(void) {
    for (unsigned j = 0; j < FFT4_N; j++) {
        fft_buf[2 * j] = (float)j;
        fft_buf[2 * j + 1] = 0;
    }
    dsps_bit_rev4r_fc32(fft_buf, FFT4_N);
    for (unsigned k = 0; k < FFT4_N; k++) {
        const float v = fft_buf[2 * k];
        if (v < 0 || v > FFT4_N - 1 || v != (float)(unsigned)v) return false;
        fast_src[k] = (uint8_t)v;
    }
    const unsigned segs = 8, nw = segs * FFT4_N;
    uint32_t *w = malloc(nw * sizeof(uint32_t));
    float *a = malloc(2 * FFT4_N * sizeof(float)), *b = a ? a + FFT4_N : NULL;
    bool ok = w && a;
    uint32_t seed = 777;
    for (unsigned j = 0; ok && j < nw; j++) {
        seed = seed * 1103515245u + 12345u;
        /* noise, an offset (exercises the mean correction) and a tone */
        int vi = (int)((seed >> 8) & 255) - 128 + 60 + (int)(200 * cosf(0.7f * j));
        int vq = (int)((seed >> 20) & 255) - 128 - 35 + (int)(200 * sinf(0.7f * j));
        w[j] = ((uint32_t)vi & 1023u) | (((uint32_t)vq & 1023u) << 10);
    }
    static const struct {
        unsigned flags;
        int dc_mode; /* -1 = no tracker */
    } cases[] = {{DSP_NEGATE_Q | DSP_REMOVE_MEAN, -1}, {0, -1}, {DSP_REMOVE_MEAN | DSP_MAX, -1},
                 {DSP_NEGATE_Q, -1}, {0, 1}, {DSP_MAX, 0}};
    for (unsigned c = 0; ok && c < sizeof(cases) / sizeof(cases[0]); c++) {
        const bool use_dc = cases[c].dc_mode >= 0;
        dsp_dc_t da = {0, 0, false, use_dc ? (unsigned)cases[c].dc_mode : 0}, db = da;
        memset(a, 0, 2 * FFT4_N * sizeof(float));
        welch_fast(w, segs, cases[c].flags, a, use_dc ? &da : NULL);
        dsp_welch(w, FFT4_N, segs, cases[c].flags, b, use_dc ? &db : NULL); /* fast_ok still false */
        float err = 0, peak = 0;
        for (unsigned k = 0; k < FFT4_N; k++) {
            err = fmaxf(err, fabsf(a[k] - b[k]));
            peak = fmaxf(peak, b[k]);
        }
        ok = peak > 0 && err <= 1e-4f * peak;
    }
    free(w);
    free(a);
    return ok;
}
#endif

static struct {
    const uint32_t *words;
    unsigned n, segments, flags;
    dsp_dc_t *dc;
    bool pending, fast;
} later;
void dsp_welch_begin(const uint32_t *words, unsigned n, unsigned segments, unsigned flags, dsp_dc_t *dc) {
    later.words = words;
    later.n = n;
    later.segments = segments;
    later.flags = flags;
    later.dc = dc;
    later.pending = true;
    later.fast = false;
#ifdef ESP_PLATFORM
    later.fast = n == FFT4_N && segments && fast_ok;
    if (later.fast) fast_begin(words, segments, flags, dc);
#endif
}
void dsp_welch_end(float *pow) {
    if (!later.pending) return;
    later.pending = false;
#ifdef ESP_PLATFORM
    if (later.fast) {
        fast_end(pow);
        return;
    }
#endif
    dsp_welch(later.words, later.n, later.segments, later.flags, pow, later.dc);
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
