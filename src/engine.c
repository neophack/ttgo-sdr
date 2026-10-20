/* engine.c — local spectrum scans for the TTGO screen.
 *
 * Band scans (2400..2500 or 2300..2500 MHz, 240 columns): LOs 10 MHz apart
 * at 80 MS/s (1800..2800 MHz: 30 MHz apart, see sweep_xwide). Every frame
 * visits each LO SCAN_PASSES times, one 2048-sample snapshot (25.6 us) per
 * visit, each split into 8 Hann-windowed 256-point FFTs (Welch). Frames are
 * kept short (few snapshots each) and the screen averages them with a time
 * constant (ui_tft.cpp), so a fast frame rate costs no smoothing: what sets
 * the trace's stability is snapshots per second, and spreading them in time
 * matters more than the FFT count. In a busy band the variation comes from
 * whether WiFi packets happen to be on air during a snapshot, so many
 * independent snapshots beat one long one of the same total length.
 * Each snapshot's mean is removed (LO leakage), Q is negated so +f is above
 * the LO, and every column is the mean power of all bins from all LOs that
 * land in it, skipping +-1 MHz around each LO and down-weighting the analog
 * filter edges.
 * Channel zoom (ch1/6/11): one LO, 40 MS/s, ZOOM_PASSES 2048-sample
 * snapshots; 240 of the 256 bins (37.5 MHz) map 1:1 to the screen columns.
 *
 * A sweep keeps the radio for a whole pass: each LO is tuned while the DSP
 * worker on core 1 starts on the previous snapshot's FFTs, the engine joins
 * in while the PLL settles, then captures. The access point gets the radio
 * back after each pass and between frames (engine_task sizes that gap).
 *
 * "legacy" reproduces the previous firmware's sweep (one 1024-point FFT per
 * LO, no DC removal, no Q negation, max-merge, 2-sweep average) so its
 * frame-to-frame jitter can be measured on the same hardware (TTGO STAT?).
 */
#include "engine.h"
#include "sdr_rx.h"
#include "dsp.h"
#include "link.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_wifi_types.h"

#define SCAN_FFT    256u
#define SCAN_SEGS   8u /* FFTs per snapshot: 2048 samples */
#define SCAN_PASSES 2u /* snapshots per LO per frame (narrow bands) */
#define ZOOM_PASSES 2u /* snapshots per channel zoom frame */
/* Sweep LOs 10 MHz apart; each covers its +-1..15 MHz (+-19 down-weighted).
 * 2412..2484 are WiFi channels (fast PHY path); 2312..2402 use the ROM PLL
 * retune, inside esp-sdr's qualified 2210..2651 MHz direct-LO envelope. */
static const unsigned los_24[] = {2412, 2422, 2432, 2442, 2452, 2462, 2472, 2484};
static const unsigned los_wide[] = {2312, 2322, 2332, 2342, 2352, 2362, 2372, 2382, 2392,
                                    2402, 2412, 2422, 2432, 2442, 2452, 2462, 2472, 2484};
/* 1800..2800 MHz: 240 columns of 4.2 MHz, so LOs 30 MHz apart suffice (each
 * covers +-15 MHz at full weight; a column still averages ~13 bins per LO)
 * and one snapshot per LO keeps the frame near 0.1 s for 34 LOs. Below
 * 2210 MHz the receiver uses esp-sdr's alternate 6/5 LO plan, which costs a
 * 3 ms settle per LO (sdr_rx_slow_retune); esp-sdr measured ESP32 reception
 * 1850..2850 MHz, so the first few columns may show no real signal. */
#define XWIDE_LOS 34
static unsigned los_xwide[XWIDE_LOS]; /* 1815, 1845, ... 2805 (engine_start) */
typedef struct {
    float f0, f1;
    const unsigned *los;
    unsigned count;
    unsigned passes;
} sweep_t;
static const sweep_t sweep_24 = {2400.0f, 2500.0f, los_24, sizeof(los_24) / sizeof(los_24[0]), SCAN_PASSES};
static const sweep_t sweep_wide = {2300.0f, 2500.0f, los_wide, sizeof(los_wide) / sizeof(los_wide[0]),
                                   SCAN_PASSES};
static const sweep_t sweep_xwide = {1800.0f, 2800.0f, los_xwide, XWIDE_LOS, 1};
static const unsigned zoom_los[BAND_COUNT] = {0, 0, 0, 2412, 2437, 2462};

/* ---------------- frame mailbox ---------------- */
static ui_frame_t mailbox;
static portMUX_TYPE mailbox_mux = portMUX_INITIALIZER_UNLOCKED;
static local_cfg_t local = {.band = BAND_SCAN, .agc = 0, .gain = 40, .legacy = 0};
static portMUX_TYPE local_mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t engine_latest(ui_frame_t *out) {
    taskENTER_CRITICAL(&mailbox_mux);
    uint32_t seq = mailbox.seq;
    if (seq) *out = mailbox;
    taskEXIT_CRITICAL(&mailbox_mux);
    return seq;
}
void engine_get_local(local_cfg_t *out) {
    taskENTER_CRITICAL(&local_mux);
    *out = local;
    taskEXIT_CRITICAL(&local_mux);
}
void engine_set_local(const local_cfg_t *in) {
    taskENTER_CRITICAL(&local_mux);
    local = *in;
    if (local.band >= BAND_COUNT) local.band = BAND_SCAN;
    taskEXIT_CRITICAL(&local_mux);
}

/* ---------------- stability statistics ---------------- */
static struct {
    uint32_t frames;
    double sum_raw;
    float prev[UI_COLS];
    bool have_prev;
    uint8_t prev_source;
    uint16_t prev_lo;
    float floor_db;
    double frame_us, radio_us;
    uint32_t frame_count;
    uint32_t agc_moves, gain_min, gain_max;
} st = {.gain_min = 255};
static portMUX_TYPE stats_mux = portMUX_INITIALIZER_UNLOCKED;

static float percentile20(const float *v, unsigned n) {
    float tmp[UI_COLS];
    memcpy(tmp, v, n * sizeof(float));
    unsigned k = n / 5;
    /* partial selection sort: enough for 240 values a few times a second */
    for (unsigned i = 0; i <= k; i++) {
        unsigned m = i;
        for (unsigned j = i + 1; j < n; j++)
            if (tmp[j] < tmp[m]) m = j;
        float t = tmp[i];
        tmp[i] = tmp[m];
        tmp[m] = t;
    }
    return tmp[k];
}

static void stats_frame(const ui_frame_t *f) {
    if (f->source != SRC_SCAN && f->source != SRC_ZOOM) return;
    bool same = st.have_prev && st.prev_source == f->source && st.prev_lo == f->lo_mhz;
    double raw = 0;
    for (unsigned c = 0; c < UI_COLS; c++) {
        if (same) raw += fabsf(f->db[c] - st.prev[c]);
        st.prev[c] = f->db[c];
    }
    taskENTER_CRITICAL(&stats_mux);
    if (same) {
        st.frames++;
        st.sum_raw += raw / UI_COLS;
    }
    taskEXIT_CRITICAL(&stats_mux);
    st.have_prev = true;
    st.prev_source = f->source;
    st.prev_lo = f->lo_mhz;
    st.floor_db = percentile20(f->db, UI_COLS);
}

void engine_take_stats(engine_stats_t *out) {
    taskENTER_CRITICAL(&stats_mux);
    out->frames = st.frames;
    out->jitter_raw_db = st.frames ? (float)(st.sum_raw / st.frames) : 0;
    out->floor_db = st.floor_db;
    out->frame_ms = st.frame_count ? (float)(st.frame_us / st.frame_count / 1000.0) : 0;
    out->radio_ms = st.frame_count ? (float)(st.radio_us / st.frame_count / 1000.0) : 0;
    out->agc_moves = st.agc_moves;
    out->gain_min = st.gain_min;
    out->gain_max = st.gain_max;
    st.frames = 0;
    st.sum_raw = 0;
    st.frame_us = 0;
    st.radio_us = 0;
    st.frame_count = 0;
    st.agc_moves = 0;
    st.gain_min = 255;
    st.gain_max = 0;
    taskEXIT_CRITICAL(&stats_mux);
}

static void note_gain(const uint32_t *w, unsigned n) {
    unsigned g0 = (w[0] >> 20) & 0xff, g1 = (w[n / 2] >> 20) & 0xff, g2 = (w[n - 1] >> 20) & 0xff;
    taskENTER_CRITICAL(&stats_mux);
    if (g0 != g1 || g1 != g2) st.agc_moves++;
    if (g0 < st.gain_min) st.gain_min = g0;
    if (g0 > st.gain_max) st.gain_max = g0;
    taskEXIT_CRITICAL(&stats_mux);
}

void engine_publish(ui_frame_t *f) {
    static uint32_t seq;
    f->seq = ++seq;
    stats_frame(f);
    taskENTER_CRITICAL(&mailbox_mux);
    mailbox = *f;
    taskEXIT_CRITICAL(&mailbox_mux);
}

void engine_frame_from_bins(ui_frame_t *f, const float *pow, unsigned n, bool negated_q, float scale,
                            unsigned lo_mhz, unsigned rate_hz) {
    for (unsigned c = 0; c < UI_COLS; c++) {
        unsigned a = c * n / UI_COLS, b = (c + 1) * n / UI_COLS;
        if (b <= a) b = a + 1;
        float sum = 0;
        for (unsigned r = a; r < b; r++) {
            /* RF-ordered index r: offset (r - n/2) bins from the LO */
            unsigned j = negated_q ? (r + n / 2) % n : (n + n / 2 - r) % n;
            sum += pow[j];
        }
        f->db[c] = dsp_dbfs(sum * scale / (float)(b - a));
    }
    f->lo_mhz = lo_mhz;
    f->rate_hz = rate_hz;
    f->f0_mhz = (float)lo_mhz - rate_hz / 2e6f;
    f->f1_mhz = (float)lo_mhz + rate_hz / 2e6f;
}

/* ---------------- local scans ---------------- */
static float col_sum[UI_COLS], col_w[UI_COLS];
static ui_frame_t work;

static int64_t radio_us; /* this frame's time in capture_at */
static bool capture_at(unsigned mhz, unsigned n, unsigned clock, const uint32_t **words, bool hold) {
    const int64_t t0 = esp_timer_get_time();
    sdr_rx_set_frequency(mhz);
    int us = hold ? sdr_rx_capture_held(n, clock, words, AP_HOME_MHZ)
                  : sdr_rx_capture_burst(n, clock, words, AP_HOME_MHZ);
    radio_us += esp_timer_get_time() - t0;
    if (us < 0) return false;
    note_gain(*words, n);
    return true;
}

static void finish_columns(void) {
    for (unsigned c = 0; c < UI_COLS; c++) {
        if (col_w[c] > 0) continue;
        /* an uncovered column (none expected): borrow a neighbour */
        unsigned l = c, r = c;
        while (l > 0 && col_w[l] <= 0) l--;
        while (r < UI_COLS - 1 && col_w[r] <= 0) r++;
        unsigned s = col_w[l] > 0 ? l : r;
        col_sum[c] = col_sum[s];
        col_w[c] = col_w[s] > 0 ? col_w[s] : 1;
    }
    for (unsigned c = 0; c < UI_COLS; c++) work.db[c] = dsp_dbfs(col_sum[c] / col_w[c]);
}

static int column_of(const sweep_t *sw, float mhz) {
    return (int)floorf((mhz - sw->f0) / (sw->f1 - sw->f0) * UI_COLS);
}

static void sweep_meta(const sweep_t *sw) {
    work.source = SRC_SCAN;
    work.lo_mhz = 0;
    work.rate_hz = 80000000;
    work.f0_mhz = sw->f0;
    work.f1_mhz = sw->f1;
}

/* Fold one 80 MS/s snapshot taken at `lo` into the columns: sweep_begin
 * starts its FFTs on the other core, sweep_add finishes them here. */
static void sweep_begin(const uint32_t *w) {
    dsp_welch_begin(w, SCAN_FFT, SCAN_SEGS, DSP_NEGATE_Q | DSP_REMOVE_MEAN, NULL);
}
static void sweep_add(const sweep_t *sw, unsigned lo) {
    float *bins = dsp_scratch();
    const float bin_mhz = 80.0f / SCAN_FFT, cols_per_mhz = UI_COLS / (sw->f1 - sw->f0);
    const float cols_per_bin = bin_mhz * cols_per_mhz;
    memset(bins, 0, SCAN_FFT * sizeof(float));
    dsp_welch_end(bins);
    /* column = floor(lo_col + off * cols_per_bin): no division or floorf
     * per bin, they cost more than the bins' arithmetic */
    const float lo_col = ((float)lo - sw->f0) * cols_per_mhz;
    for (unsigned k = 0; k < SCAN_FFT; k++) {
        int off = k < SCAN_FFT / 2 ? (int)k : (int)k - (int)SCAN_FFT;
        float a = fabsf(off * bin_mhz);
        if (a < 1.0f || a > 19.0f) continue;
        float x = lo_col + off * cols_per_bin;
        if (x < 0 || x >= UI_COLS) continue;
        unsigned c = (unsigned)x; /* x >= 0: truncation is floor */
        float weight = a <= 15.0f ? 1.0f : 0.1f; /* analog filter skirt */
        col_sum[c] += weight * bins[k] * (1.0f / SCAN_SEGS);
        col_w[c] += weight;
    }
}

static bool scan_sweep(const sweep_t *sw) {
    unsigned ok = 0;
    /* the last snapshot, still in the capture bank until the next capture */
    const uint32_t *held = NULL;
    unsigned held_lo = 0;
    memset(col_sum, 0, sizeof(col_sum));
    memset(col_w, 0, sizeof(col_w));
    sdr_rx_set_filter((int)sdr_rx_bandwidth_dcap(67));
    for (unsigned pass = 0; pass < sw->passes; pass++) {
        for (unsigned l = 0; l < sw->count; l++) {
            const unsigned lo = sw->los[l];
            if (held) sweep_begin(held);
            if (held && (sdr_rx_holding() || sdr_rx_slow_retune(lo))) {
                /* retune while the other core starts on the last snapshot,
                 * then finish it while the PLL settles (0.2 ms, 3 ms for
                 * an LO plan switch) */
                const int64_t t0 = esp_timer_get_time();
                sdr_rx_set_frequency(lo);
                sdr_rx_burst_begin();
                radio_us += esp_timer_get_time() - t0;
            }
            if (held) sweep_add(sw, held_lo);
            held = NULL;
            const uint32_t *w;
            /* the radio stays with the sweep for a whole pass and goes back
             * to the access point after it (engine_task leaves it there
             * between frames): two PHY channel switches less per snapshot,
             * and a run of alternate-plan LOs pays the plan switch once */
            const bool hold = l + 1 < sw->count;
            if (!capture_at(lo, SCAN_FFT * SCAN_SEGS, SDR_CLK_80M, &w, hold)) continue;
            if (!ok++) work.gain_seen = (w[0] >> 20) & 0xff;
            held = w;
            held_lo = lo;
        }
    }
    if (held) {
        sweep_begin(held);
        sweep_add(sw, held_lo);
    }
    if (!ok) return false;
    finish_columns();
    sweep_meta(sw);
    return true;
}

/* The previous firmware's sweep, kept only for A/B jitter measurements. */
static bool scan_sweep_legacy(const sweep_t *sw) {
    float *bins = dsp_scratch();
    const unsigned n = 1024;
    const float bin_mhz = 80.0f / n;
    static float step[UI_COLS];
    unsigned frames = 0;
    memset(col_sum, 0, sizeof(col_sum));
    sdr_rx_set_filter(-1);
    for (unsigned rep = 0; rep < 2; rep++) {
        memset(step, 0, sizeof(step));
        for (unsigned l = 0; l < sw->count; l++) {
            const uint32_t *w;
            if (!capture_at(sw->los[l], n, SDR_CLK_80M, &w, false)) continue;
            memset(bins, 0, n * sizeof(float));
            dsp_welch(w, n, 1, 0, bins, NULL);
            for (unsigned k = 0; k < n; k++) {
                int off = k < n / 2 ? (int)k : (int)k - (int)n;
                float off_mhz = off * bin_mhz;
                if (fabsf(off_mhz) > 20.0f) continue;
                int c = column_of(sw, (float)sw->los[l] + off_mhz);
                if (c >= 0 && c < UI_COLS && bins[k] > step[c]) step[c] = bins[k];
            }
            frames++;
        }
        for (unsigned c = 0; c < UI_COLS; c++) col_sum[c] += step[c];
    }
    if (!frames) return false;
    for (unsigned c = 0; c < UI_COLS; c++) col_w[c] = col_sum[c] > 0 ? 2 : 0;
    finish_columns();
    sweep_meta(sw);
    return true;
}

static bool scan_zoom(unsigned lo) {
    float *bins = dsp_scratch();
    const uint32_t *w;
    sdr_rx_set_filter((int)sdr_rx_bandwidth_dcap(38));
    memset(bins, 0, SCAN_FFT * sizeof(float));
    unsigned passes = 0;
    for (unsigned pass = 0; pass < ZOOM_PASSES; pass++) {
        if (pass) vTaskDelay(1); /* spread the snapshots; the AP transmits meanwhile */
        if (!capture_at(lo, SCAN_FFT * SCAN_SEGS, SDR_CLK_40M, &w, false)) continue;
        if (!passes++) work.gain_seen = (w[0] >> 20) & 0xff;
        dsp_welch(w, SCAN_FFT, SCAN_SEGS, DSP_NEGATE_Q | DSP_REMOVE_MEAN, bins, NULL);
    }
    if (!passes) return false;
    /* RF-ordered bins -120..119 -> columns 0..239 (156.25 kHz each) */
    for (unsigned c = 0; c < UI_COLS; c++) {
        int off = (int)c - UI_COLS / 2;
        unsigned k = (unsigned)(off + (int)SCAN_FFT) % SCAN_FFT;
        work.db[c] = dsp_dbfs(bins[k] / (SCAN_SEGS * passes));
    }
    const float half = UI_COLS / 2 * 40.0f / SCAN_FFT;
    work.source = SRC_ZOOM;
    work.lo_mhz = lo;
    work.rate_hz = 40000000;
    work.f0_mhz = lo - half;
    work.f1_mhz = lo + half;
    return true;
}

static void local_frame(const local_cfg_t *cfg) {
    sdr_rx_set_gain(cfg->agc, cfg->gain);
    int64_t t0 = esp_timer_get_time();
    radio_us = 0;
    bool ok;
    if (cfg->band == BAND_SCAN || cfg->band == BAND_WIDE || cfg->band == BAND_XWIDE) {
        const sweep_t *sw = cfg->band == BAND_SCAN ? &sweep_24 : cfg->band == BAND_WIDE ? &sweep_wide : &sweep_xwide;
        ok = cfg->legacy ? scan_sweep_legacy(sw) : scan_sweep(sw);
    } else
        ok = scan_zoom(zoom_los[cfg->band]);
    if (!ok) return;
    taskENTER_CRITICAL(&stats_mux);
    st.frame_us += (double)(esp_timer_get_time() - t0);
    st.radio_us += (double)radio_us;
    st.frame_count++;
    taskEXIT_CRITICAL(&stats_mux);
    work.gain_agc = cfg->agc;
    work.gain_code = cfg->gain;
    engine_publish(&work);
}

/* Sleep up to `ms`, returning early when a protocol command arrives. Always
 * blocks at least one tick: the engine outranks core 0's idle task, and a
 * steady command stream must not starve it into the task watchdog. */
static void idle(unsigned ms) {
    vTaskDelay(1);
    for (unsigned t = 1; t < ms; t += 5) {
        if (link_input_pending()) return;
        vTaskDelay(pdMS_TO_TICKS(ms - t < 5 ? ms - t : 5));
    }
}

/* stations associated with the SoftAP (WiFi event loop task) */
static volatile int ap_stations;
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    (void)base;
    (void)data;
    if (id == WIFI_EVENT_AP_STACONNECTED) ap_stations = ap_stations + 1;
    else if (id == WIFI_EVENT_AP_STADISCONNECTED && ap_stations > 0) ap_stations = ap_stations - 1;
}

static TaskHandle_t engine_handle;
unsigned engine_stack_free(void) { return engine_handle ? uxTaskGetStackHighWaterMark(engine_handle) : 0; }

static void engine_task(void *arg) {
    (void)arg;
    dsp_init();
    for (;;) {
        if (link_service()) {
            vTaskDelay(1);
            continue;
        }
        if (link_lease_active()) {
            vTaskDelay(1);
            continue;
        }
        local_cfg_t cfg;
        engine_get_local(&cfg);
        int64_t t0 = esp_timer_get_time();
        local_frame(&cfg);
        /* A sweep keeps the radio for whole passes, so the access point
         * mostly gets it here. With a station associated (it may be loading
         * the web page) it gets 40% of the time; with none it only needs
         * beacons and probe replies, 2 ms per frame. Frames come at most
         * as fast as the screen draws them (~60/s, ~80 zoomed). */
        const int64_t spent = esp_timer_get_time() - t0;
        const int64_t floor_us = cfg.band == BAND_SCAN || cfg.band == BAND_WIDE || cfg.band == BAND_XWIDE ? 16000 : 12500;
        int64_t rest = ap_stations > 0 ? spent * 2 / 3 : 2000;
        if (spent + rest < floor_us) rest = floor_us - spent;
        idle((unsigned)((rest + 500) / 1000));
    }
}

void engine_start(void) {
    for (unsigned i = 0; i < XWIDE_LOS; i++) los_xwide[i] = 1815 + 30 * i;
    link_init();
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, on_wifi_event, NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, on_wifi_event, NULL);
    /* Core 0, next to the WiFi driver: PHY calls are never truly concurrent
     * with it. Core 1 has the Arduino loop (TFT) and the DSP worker. */
    xTaskCreatePinnedToCore(engine_task, "sdr_engine", 6144, NULL, 3, &engine_handle, 0);
}
