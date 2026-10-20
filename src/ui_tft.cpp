/* ui_tft.cpp — power spectrum / waterfall on the TTGO T-Display.
 *
 * The whole 240x135 screen is composed every frame in 5-row strips and
 * pushed strip by strip: no full framebuffer, no erase-then-draw, so nothing
 * flickers. Text is drawn into the strip. Two strip sprites alternate: one
 * is composed while DMA sends the other, and a strip identical to what the
 * panel already shows (hash) is not sent at all (every strip once a second),
 * so the 40 MHz SPI link only carries the rows that changed.
 *
 * Display processing (the engine already Welch-averages each frame):
 *   - trace averaging: dB-domain EMA, time constant off / 0.5 / 1.5 / 4 s
 *   - decaying peak hold (amber dots)
 *   - "calm" auto scale from esp-web-sdr: floor = 20th percentile - 10 dB
 *     in 5 dB steps with 4 dB hysteresis, top = decaying peak + 5 dB in
 *     10 dB steps, evaluated once a second
 *   - colours and waterfall palette are esp-web-sdr's
 *   - status bar: mode, gain, averaging, and the peak marker's frequency and
 *     level (the marker is a white triangle over the trace)
 *
 * Buttons: GPIO0  short = spectrum / waterfall / split, long = averaging
 *          GPIO35 short = band (2.4G / 2.3-2.5G / 1.8-2.8G scan, ch1 / ch6 /
 *          ch11), long = gain
 */
#include "ui_tft.h"
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "link.h"
#include "sdr_rx.h"

/* The project builds at -Os; composing 32400 pixels a frame is worth -O2. */
#pragma GCC optimize("O2")

#define W        240
#define H        135
#define BAR_H    12
#define AXIS_H   8  /* 2 tick rows, 3x5 labels, a gap */
#define CHAN_H   21 /* WiFi channel strip between the chart and the frequency axis */
#define CHAN_LINES 5 /* rows of channel range lines, 2 px apart, below the numbers */
#define AXIS_Y0  (H - AXIS_H)
#define CHAN_Y0  (AXIS_Y0 - CHAN_H)
#define CHART_Y0 BAR_H
#define CHART_Y1 CHAN_Y0 /* exclusive */
#define WF_ROWS  (CHART_Y1 - CHART_Y0)
#define STRIP    5 /* 27 strips, two 2.4 KB buffers */
#define STRIPS   (H / STRIP)
#define DEFAULT_GAIN 40

enum { VIEW_SPEC, VIEW_WATER, VIEW_SPLIT, VIEW_COUNT };
static const char *const view_names[VIEW_COUNT] = {"SPECTRUM", "WATERFALL", "SPLIT VIEW"};
/* Trace averaging time constants (s). Averaging runs in dB ("video"
 * averaging), so a single WiFi burst nudges the trace instead of yanking it;
 * peak dots and the waterfall keep the raw frames. */
static const float avg_tau[] = {0.0f, 0.5f, 1.5f, 4.0f};
static const char *const avg_names[] = {"AVG OFF", "AVG 0.5s", "AVG 1.5s", "AVG 4s"};
static const char *const avg_short[] = {"AVG 0", "AVG.5", "AVG1.5", "AVG 4"}; /* status bar */
#define AVG_LEVELS 4
#define AVG_DEFAULT 2
static const uint8_t gain_steps[] = {0 /* AGC */, 20, 30, 40, 50, 60, 70};

static TFT_eSPI tft;
static TFT_eSprite strips[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};
static TFT_eSprite *strip = &strips[0]; /* the one being composed */
static uint32_t strip_hash[STRIPS], full_push_ms;
static bool panel_known; /* strip_hash describes what the panel shows */
/* Strips wholly inside the status bar (rows < STATIC_TOP) or the channel
 * strip + axis (rows >= STATIC_BOTTOM) depend only on what static_key()
 * hashes; while that is unchanged they are neither composed nor sent. */
#define STATIC_TOP    (BAR_H / STRIP * STRIP)
#define STATIC_BOTTOM ((CHAN_Y0 + STRIP - 1) / STRIP * STRIP)
static uint32_t static_keys[2];
static Preferences prefs;

static constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}
static constexpr uint16_t sw(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }

/* esp-web-sdr style.css / app.js colours */
static const uint16_t C_BG = rgb(0x18, 0x21, 0x26), C_LEVEL = rgb(0x34, 0x40, 0x47),
                      C_MAJOR = rgb(0x33, 0x42, 0x4a), C_MINOR = rgb(0x20, 0x2a, 0x2f),
                      C_TRACE = rgb(0x37, 0xc9, 0x64), C_HOLD = rgb(0xe6, 0xb9, 0x69),
                      C_TEXT = rgb(0xa1, 0xad, 0xb2), C_BAR = rgb(0x0e, 0x15, 0x18),
                      C_WF_EMPTY = rgb(0x11, 0x19, 0x1e), C_DIV = rgb(0x29, 0x31, 0x35),
                      C_TICK = rgb(0x5b, 0x68, 0x6e), C_MODE = rgb(0xc0, 0xc9, 0xcd),
                      C_REMOTE = rgb(0xe0, 0xa0, 0x40), C_TOAST = rgb(0xff, 0xd1, 0x66),
                      C_MARK = rgb(0xf0, 0xf4, 0xf6);

static uint16_t wf_pal[256];     /* waterfall palette, native order */
static uint16_t wf_lut[256];     /* stored code -> swapped colour for the current scale */

/* ---------------- state ---------------- */
static uint8_t view = VIEW_SPEC, avg_level = AVG_DEFAULT;
static uint32_t last_push_ms;
static volatile float jit_sum;
static volatile uint32_t jit_n;
static float trace[W], peak[W];
static bool have_trace;
static uint32_t trace_key;
/* WF_ROWS raw frames of 0.5 dB codes from -140 dBFS, four per word. The
 * buffer lives in the IRAM heap, which allows only 32-bit word access. */
static uint32_t *ring;
static unsigned ring_head, ring_count;
static float floor_db = -100, range_db = 70;
static float auto_pk;
static bool auto_pk_ok;
static uint32_t auto_t;
static ui_frame_t cur;
static bool have_cur;
/* peak marker: the trace's highest column. It follows the maximum within
 * 2 columns and jumps elsewhere only for 3 dB more, so a burst or noise does
 * not throw it around between nearly equal peaks. */
static int pk_col = -1;
static float pk_db;
static char toast_text[24];
static uint32_t toast_until;
static local_cfg_t local;
volatile bool ui_shot_request;

static uint8_t wf_code(float db) {
    float c = (db + 140.0f) * 2.0f;
    return c <= 0 ? 0 : c >= 255 ? 255 : (uint8_t)lrintf(c);
}

static void palette_init() {
    static const uint8_t stops[][3] = {{3, 8, 23},     {18, 31, 81},   {38, 63, 153},  {30, 139, 181},
                                       {66, 213, 174}, {217, 237, 103}, {255, 162, 61}, {255, 244, 228}};
    const unsigned n = sizeof(stops) / sizeof(stops[0]) - 1;
    for (unsigned i = 0; i < 256; i++) {
        float p = i / 255.0f * n;
        unsigned j = (unsigned)p < n ? (unsigned)p : n - 1;
        float t = p - j;
        uint8_t c[3];
        for (unsigned k = 0; k < 3; k++) c[k] = (uint8_t)lrintf(stops[j][k] * (1 - t) + stops[j + 1][k] * t);
        wf_pal[i] = rgb(c[0], c[1], c[2]);
    }
}

static void toast(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(toast_text, sizeof(toast_text), fmt, ap);
    va_end(ap);
    toast_until = millis() + 1500;
}

static void save_prefs() {
    prefs.putUChar("view", view);
    prefs.putUChar("avg2", avg_level);
    prefs.putUChar("band", local.band);
    prefs.putUChar("agc", local.agc);
    prefs.putUChar("gain", local.gain);
}

/* ---------------- frame intake ---------------- */
static void autoscale(const float *values) {
    uint32_t now = millis();
    if (auto_t && now - auto_t < 1000) return;
    auto_t = now | 1;
    float v[W];
    memcpy(v, values, sizeof(v));
    /* insertion sort, 240 values once a second */
    for (int i = 1; i < W; i++) {
        float x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
    float nf = v[W / 5] - 10, pk = v[W - 1];
    auto_pk = auto_pk_ok ? fmaxf(pk, auto_pk - 2) : pk;
    auto_pk_ok = true;
    float f0 = floor_db, r0 = range_db;
    float floor = fabsf(nf - f0) > 4 ? roundf(nf / 5) * 5 : f0, top = f0 + r0;
    if (auto_pk + 5 > top || auto_pk + 25 < top) top = ceilf((auto_pk + 5) / 10) * 10;
    floor = fmaxf(-140, fminf(-40, floor));
    floor_db = floor;
    /* multiples of 20 dB: the four grid steps land on multiples of 5 dB */
    range_db = fmaxf(20, fminf(120, ceilf((top - floor) / 20) * 20));
}

void ui_push(const ui_frame_t &f) {
    uint32_t key = f.source * 1000003u + (uint32_t)lrintf(f.f0_mhz * 8) * 31u + (uint32_t)lrintf(f.f1_mhz * 8);
    if (key != trace_key) {
        trace_key = key;
        have_trace = false;
        pk_col = -1;
        ring_count = 0;
        auto_t = 0;
        auto_pk_ok = false;
    }
    const uint32_t now = millis();
    float dt = (now - last_push_ms) / 1000.0f;
    dt = dt < 0.02f ? 0.02f : dt > 2.0f ? 2.0f : dt;
    last_push_ms = now;
    const float tau = avg_tau[avg_level], a = tau > 0 ? expf(-dt / tau) : 0.0f;
    const float decay = 10.0f * dt; /* peak dots fall 10 dB/s */
    /* ring stays NULL when both allocations in ui_init() failed: the
     * waterfall stays empty (rendered as the empty colour), the trace,
     * peaks and averaging keep working */
    uint32_t *row = ring ? ring + ring_head * (W / 4) : nullptr;
    float moved = 0;
    for (int c = 0; c < W; c++) {
        float v = f.db[c];
        float t = have_trace ? a * trace[c] + (1 - a) * v : v;
        moved += fabsf(t - trace[c]);
        trace[c] = t;
        peak[c] = have_trace ? fmaxf(peak[c] - decay, v) : v;
        if (row) {
            if ((c & 3) == 0) row[c / 4] = 0;
            row[c / 4] |= (uint32_t)wf_code(v) << (8 * (c & 3));
        }
    }
    if (have_trace) { /* what the screen trace does frame to frame (TTGO STAT?) */
        jit_sum = jit_sum + moved / W;
        jit_n = jit_n + 1;
    }
    have_trace = true;
    if (row) {
        ring_head = (ring_head + 1) % WF_ROWS;
        if (ring_count < WF_ROWS) ring_count++;
    }
    autoscale(trace);
    int best = 0;
    for (int c = 1; c < W; c++)
        if (trace[c] > trace[best]) best = c;
    if (pk_col < 0 || abs(best - pk_col) <= 2 || trace[best] > trace[pk_col] + 3) pk_col = best;
    pk_db = trace[pk_col];
    cur = f;
    have_cur = true;
}

/* ---------------- composition ---------------- */
struct label_t {
    int16_t x, y;
    uint16_t color;
    uint8_t datum; /* TFT_eSPI datum; 255 = centred 3x5 digits */
    char text[20];
};
static label_t labels[40];
static int nlabels;
static void label(int x, int y, uint16_t color, uint8_t datum, const char *fmt, ...) {
    if (nlabels >= (int)(sizeof(labels) / sizeof(labels[0]))) return;
    label_t *l = &labels[nlabels++];
    l->x = x;
    l->y = y;
    l->color = color;
    l->datum = datum;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(l->text, sizeof(l->text), fmt, ap);
    va_end(ap);
}

/* 5-pixel-tall labels for the frequency axis and the channel / band strip
 * where 6x8 text would collide, and the dB scale: digits, '.', '-' and the
 * letters of "n41" and "WiFi". Rows are bit maps
 * of the glyph width, MSB = leftmost column. */
#define TINY_DATUM 255
struct tiny_glyph_t {
    char c;
    uint8_t w, rows[5];
};
static const tiny_glyph_t tiny_font[] = {
    {'0', 3, {7, 5, 5, 5, 7}}, {'1', 3, {2, 6, 2, 2, 7}}, {'2', 3, {7, 1, 7, 4, 7}},
    {'3', 3, {7, 1, 7, 1, 7}}, {'4', 3, {5, 5, 7, 1, 1}}, {'5', 3, {7, 4, 7, 1, 7}},
    {'6', 3, {7, 4, 7, 5, 7}}, {'7', 3, {7, 1, 1, 1, 1}}, {'8', 3, {7, 5, 7, 5, 7}},
    {'9', 3, {7, 5, 7, 1, 7}}, {'n', 3, {0, 6, 5, 5, 5}}, {'i', 1, {1, 0, 1, 1, 1}},
    {'F', 3, {7, 4, 6, 4, 4}}, {'W', 5, {17, 17, 21, 21, 10}}, {'.', 1, {0, 0, 0, 0, 1}},
    {'-', 2, {0, 0, 3, 0, 0}},
};
static const tiny_glyph_t *tiny_glyph(char c) {
    for (const tiny_glyph_t &g : tiny_font)
        if (g.c == c) return &g;
    return &tiny_font[0];
}
static int tiny_width(const char *text) {
    int w = -1;
    for (; *text; text++) w += tiny_glyph(*text)->w + 1;
    return w < 0 ? 0 : w;
}
static void draw_tiny(const label_t *l, int strip_y0) {
    int x = l->x - tiny_width(l->text) / 2;
    for (const char *t = l->text; *t; t++) {
        const tiny_glyph_t *g = tiny_glyph(*t);
        for (int r = 0; r < 5; r++)
            for (int c = 0; c < g->w; c++)
                if (g->rows[r] >> (g->w - 1 - c) & 1) strip->drawPixel(x + c, l->y + r - strip_y0, l->color);
        x += g->w + 1;
    }
}

static int spec_y0, spec_h, wf_y0, wf_h, div_y;
/* down-pointing 5x3 triangles: [0] over the peak on the trace (clipped to
 * the spectrum), [1] in front of the peak readout; x < 0 = not drawn */
static int16_t mark_x[2], mark_y[2];
static int16_t ty[W], seg_top[W], seg_bot[W], py[W];
/* per column, one load per pixel: ty | seg_top << 8 | seg_bot << 16 | py << 24
 * (all rows < 135; py -1 = 255 matches no row) */
static uint32_t col_pack[W];
static uint8_t colflag[W]; /* 0 none, 1 minor, 2 major frequency gridline */
static int16_t level_rows[5];
static uint8_t chan_mask[W]; /* bit r: a channel range line in row r covers this column */
static const uint16_t chan_colors[CHAN_LINES] = {rgb(0x4f, 0xa3, 0xff), rgb(0x37, 0xc9, 0x64),
                                                 rgb(0xe0, 0xa0, 0x40), rgb(0xd0, 0x70, 0xb0),
                                                 rgb(0x40, 0xc8, 0xd0)};
static uint8_t fill_alpha[WF_ROWS];

/* 1.8-2.8G view: cellular NR bands (3GPP TS 38.101-1) and WiFi instead of
 * the 14 channel lines. Three tracks of (5 px label, range line below);
 * tracks are assigned by hand so neither labels nor lines collide at
 * 0.24 px/MHz. FDD bands show both halves: uplink dotted, downlink solid. */
#define BAND_TRACKS 3
#define BAND_TRACK_H 7 /* 5 label rows, the line, one gap */
struct nr_band_t {
    const char *name;
    float lo, hi;
    uint8_t track, color;
    bool uplink;
};
static const uint16_t band_colors[] = {
    rgb(0x4f, 0xa3, 0xff) /* n1 */,  rgb(0x37, 0xc9, 0x64) /* n2 */,  rgb(0xe0, 0xa0, 0x40) /* n3 */,
    rgb(0xd0, 0x70, 0xb0) /* n7 */,  rgb(0x40, 0xc8, 0xd0) /* n30 */, rgb(0xe8, 0xe0, 0x60) /* n34 */,
    rgb(0xb0, 0x90, 0xff) /* n38 */, rgb(0x90, 0xe0, 0x80) /* n39 */, rgb(0xff, 0x70, 0x60) /* n40 */,
    rgb(0x40, 0xe0, 0xb0) /* n41 */, rgb(0xe8, 0xee, 0xf0) /* WiFi */};
static const nr_band_t nr_bands[] = {
    {"n3", 1805, 1880, 0, 2, false},   /* DL; UL 1710-1785 is off screen */
    {"n2", 1850, 1910, 1, 1, true},    {"n39", 1880, 1920, 2, 7, false}, {"n1", 1920, 1980, 0, 0, true},
    {"n2", 1930, 1990, 1, 1, false},   {"n34", 2010, 2025, 2, 5, false}, {"n1", 2110, 2170, 0, 0, false},
    {"n40", 2300, 2400, 0, 8, false},  {"", 2305, 2315, 1, 4, true}, /* n30 UL: labelled once, on DL */  {"n30", 2350, 2360, 1, 4, false},
    {"WiFi", 2400, 2483.5f, 2, 10, false}, {"n41", 2496, 2690, 0, 9, false}, {"n7", 2500, 2570, 1, 3, true},
    {"n38", 2570, 2620, 2, 6, false},  {"n7", 2620, 2690, 1, 3, false},
};
static bool band_mode;
static uint8_t band_line[BAND_TRACKS][W]; /* 1 + band_colors index, 0 = none */

static int db_to_y(float db) {
    float t = (db - floor_db) / range_db;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return spec_y0 + (int)lrintf((1 - t) * (spec_h - 1));
}

static void layout() {
    spec_y0 = wf_y0 = div_y = -1;
    spec_h = wf_h = 0;
    if (view == VIEW_SPEC) {
        spec_y0 = CHART_Y0;
        spec_h = CHART_Y1 - CHART_Y0;
    } else if (view == VIEW_WATER) {
        wf_y0 = CHART_Y0;
        wf_h = CHART_Y1 - CHART_Y0;
    } else {
        spec_y0 = CHART_Y0;
        spec_h = 50;
        div_y = spec_y0 + spec_h;
        wf_y0 = div_y + 1;
        wf_h = CHART_Y1 - wf_y0;
    }
}

static void prepare_frequency_axis() {
    memset(colflag, 0, sizeof(colflag));
    float f0 = cur.f0_mhz, f1 = cur.f1_mhz, span = f1 - f0;
    static const float steps[] = {1, 2, 5, 10, 20, 50, 100, 200, 500};
    float step = steps[8];
    for (float s : steps)
        if (s / span * W >= 44) {
            step = s;
            break;
        }
    const float minor = step / 2;
    /* 100 MHz and coarser steps read as GHz (1.8 2.0 .. 2.8), finer ones as MHz */
    const bool ghz = step >= 100;
    for (float f = ceilf(f0 / minor) * minor; f <= f1 + 1e-3f; f += minor) {
        int x = (int)lrintf((f - f0) / span * W);
        if (x == W) x = W - 1; /* f1 is the right edge of the last column */
        if (x < 0 || x >= W) continue;
        bool major = fabsf(f / step - roundf(f / step)) < 1e-3f;
        colflag[x] = major ? 2 : 1;
        if (major) {
            char text[8];
            if (ghz) snprintf(text, sizeof(text), "%.1f", lrintf(f) / 1000.0f);
            else snprintf(text, sizeof(text), "%d", (int)lrintf(f));
            const int half = tiny_width(text) / 2 + 1;
            label(x < half ? half : x > W - 1 - half ? W - 1 - half : x, AXIS_Y0 + 2, C_TEXT, TINY_DATUM, "%s", text);
        }
    }
}

/* Channel strip (1.8-2.8G view: NR bands, see nr_bands).
 * WiFi channel strip: every channel 1..14 numbered at its centre frequency
 * (2412 + 5*(n-1) MHz, ch14 2484 MHz) with its own 22 MHz range line below
 * (ch1 = 2401..2423 MHz, the usual 802.11 channel chart). Overlapping
 * channels go to separate rows: {1,6,11} {2,7,12} {3,8,13} {4,9,14} {5,10}
 * never overlap within a row. A number and its line share the row colour.
 * Numbers use the 6x8 font when channels are at least 14 px apart (two
 * digits plus a gap), otherwise 3x5 digits, staggered on two rows below
 * 9 px. */
static int mhz_to_x(float mhz) { return (int)lrintf((mhz - cur.f0_mhz) / (cur.f1_mhz - cur.f0_mhz) * W); }
static void prepare_channels() {
    memset(chan_mask, 0, sizeof(chan_mask));
    const float span = cur.f1_mhz - cur.f0_mhz, ch_px = 5.0f * W / span;
    const bool big = ch_px >= 14.0f, stagger = ch_px < 9.0f;
    /* 1.8-2.8G: channels ~1 px apart; show the NR bands and WiFi instead */
    band_mode = ch_px < 5.0f;
    memset(band_line, 0, sizeof(band_line));
    if (band_mode) {
        for (const nr_band_t &b : nr_bands) {
            const int a = mhz_to_x(b.lo) + 1, e = mhz_to_x(b.hi) - 1;
            if (e < 0 || a >= W) continue;
            for (int x = a < 0 ? 0 : a; x <= e && x < W; x++)
                if (!b.uplink || !(x & 1)) band_line[b.track][x] = b.color + 1;
            if (!*b.name) continue;
            const int half = tiny_width(b.name) / 2 + 1, xc = (a + e) / 2;
            label(xc < half ? half : xc > W - 1 - half ? W - 1 - half : xc, CHAN_Y0 + b.track * BAND_TRACK_H,
                  band_colors[b.color], TINY_DATUM, "%s", b.name);
        }
        return;
    }
    for (int n = 1; n <= 14; n++) {
        const float fc = n == 14 ? 2484.0f : 2407.0f + 5.0f * n;
        if (fc + 11 < cur.f0_mhz || fc - 11 > cur.f1_mhz) continue;
        const int row = n == 14 ? 3 : (n - 1) % CHAN_LINES;
        /* 1 px trimmed at both ends so touching ranges stay distinguishable */
        const int a = mhz_to_x(fc - 11) + 1, b = mhz_to_x(fc + 11) - 1;
        for (int x = a < 0 ? 0 : a; x <= b && x < W; x++) chan_mask[x] |= 1u << row;
        const int xc = mhz_to_x(fc);
        if (xc < 0 || xc >= W) continue;
        if (big)
            label(xc < 6 ? 6 : xc > W - 7 ? W - 7 : xc, CHAN_Y0 + 1, chan_colors[row], TC_DATUM, "%d", n);
        else
            label(xc < 4 ? 4 : xc > W - 5 ? W - 5 : xc, CHAN_Y0 + (!stagger ? 4 : n & 1 ? 0 : 5), chan_colors[row],
                  TINY_DATUM, "%d", n);
    }
}

static void prepare_spectrum() {
    for (int i = 0; i <= 4; i++) level_rows[i] = spec_y0 + (int16_t)lrintf(i * (spec_h - 1) / 4.0f);
    for (int i = 0; i < 4; i++) { /* 3x5, left-aligned at x = 2 */
        char text[8];
        snprintf(text, sizeof(text), "%d", (int)lrintf(floor_db + range_db - i * range_db / 4));
        label(2 + tiny_width(text) / 2, level_rows[i] + 2, C_TEXT, TINY_DATUM, "%s", text);
    }
    for (int x = 0; x < W; x++) {
        ty[x] = have_trace ? db_to_y(trace[x]) : spec_y0 + spec_h;
        py[x] = have_trace ? db_to_y(peak[x]) : -1;
    }
    for (int x = 0; x < W; x++) {
        int a = ty[x], b = x ? ty[x - 1] : ty[x];
        seg_top[x] = a < b ? a : b;
        seg_bot[x] = a < b ? b : a;
        if (py[x] >= seg_top[x] - 1) py[x] = -1; /* the dot would touch the trace */
        col_pack[x] = (uint32_t)(ty[x] & 255) | (uint32_t)(seg_top[x] & 255) << 8 | (uint32_t)(seg_bot[x] & 255) << 16 |
                      (uint32_t)(py[x] & 255) << 24;
    }
    if (have_trace && pk_col >= 0) { /* tip 2 px above the trace's top at the peak */
        mark_x[0] = pk_col;
        mark_y[0] = seg_top[pk_col] - 5;
    }
    for (int r = 0; r < spec_h; r++) /* gradient 0x45 -> 0x03 alpha */
        fill_alpha[r] = (uint8_t)(0x45 + (0x03 - 0x45) * r / (spec_h > 1 ? spec_h - 1 : 1));
}

static void prepare_waterfall() {
    for (int code = 0; code < 256; code++) {
        float q = ((code / 2.0f - 140.0f) - floor_db) * 255.0f / range_db;
        int i = q < 0 ? 0 : q > 255 ? 255 : (int)q;
        wf_lut[code] = sw(wf_pal[i]);
    }
}

static void prepare_status() {
    char mode[20] = "WAITING";
    if (have_cur) {
        unsigned span = (unsigned)lrintf(cur.rate_hz / 1e6f);
        switch (cur.source) {
        case SRC_SCAN:
            snprintf(mode, sizeof(mode), cur.f0_mhz < 2000 ? "SCAN 1.8-2.8G" : cur.f0_mhz < 2350 ? "SCAN 2.3-2.5G" : "SCAN 2.4G");
            break;
        case SRC_ZOOM: /* channel number and its 20 MHz range */
            snprintf(mode, sizeof(mode), "CH%u %u-%u", (cur.lo_mhz - 2407) / 5, cur.lo_mhz - 11, cur.lo_mhz + 11);
            break;
        case SRC_WEB: snprintf(mode, sizeof(mode), "WEB %u/%uM", cur.lo_mhz, span); break;
        default: snprintf(mode, sizeof(mode), "USB %u/%uM", cur.lo_mhz, span); break;
        }
    }
    /* mode, gain, averaging from the left, peak readout from the right; a
     * toast takes the place of everything but the mode. 6 px per glyph. */
    bool remote = have_cur && (cur.source == SRC_WEB || cur.source == SRC_USB);
    label(2, 2, remote ? C_REMOTE : C_MODE, TL_DATUM, "%s", mode);
    int x = 2 + 6 * (int)strlen(mode) + 6;
    x = x < 90 ? 90 : x; /* gain stays put across the usual modes */
    if (toast_until && (int32_t)(millis() - toast_until) < 0) {
        label(W - 2, 2, C_TOAST, TR_DATUM, "%s", toast_text);
        return;
    }
    if (!have_cur) return;
    if (cur.gain_agc)
        label(x, 2, C_TRACE, TL_DATUM, "AGC %u", cur.gain_seen);
    else
        label(x, 2, C_TRACE, TL_DATUM, "G%u", cur.gain_code);
    x += 6 * (int)strlen(labels[nlabels - 1].text) + 6;
    int right = W - 2;
    if (have_trace && pk_col >= 0) {
        const float col_mhz = (cur.f1_mhz - cur.f0_mhz) / W, f = cur.f0_mhz + (pk_col + 0.5f) * col_mhz;
        label(right, 2, C_MARK, TR_DATUM, col_mhz < 0.2f ? "%.1f %d" : "%.0f %d", f, (int)lrintf(pk_db));
        right -= 6 * (int)strlen(labels[nlabels - 1].text) + 2;
        mark_x[1] = right - 3;
        mark_y[1] = 4;
        right -= 5;
    }
    const char *avg = avg_short[avg_level];
    if (x + 6 * (int)strlen(avg) + 4 <= right) label(x, 2, C_TEXT, TL_DATUM, "%s", avg);
}

static uint32_t fnv(uint32_t h, const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    while (len--) h = (h ^ *p++) * 16777619u;
    return h;
}
/* region 0: the status bar, 1: channel strip and axis */
static uint32_t static_key(int region) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < nlabels; i++) {
        const label_t *l = &labels[i];
        if (region == 0 ? l->y >= STATIC_TOP : l->y + 8 <= STATIC_BOTTOM) continue;
        h = fnv(h, l, offsetof(label_t, text));
        h = fnv(h, l->text, strlen(l->text));
    }
    if (region == 1) {
        h = fnv(h, &band_mode, sizeof(band_mode));
        h = fnv(h, colflag, sizeof(colflag));
        h = fnv(h, band_mode ? (const void *)band_line : (const void *)chan_mask,
                band_mode ? sizeof(band_line) : sizeof(chan_mask));
    }
    return h;
}

static void compose_row(int y, uint16_t *out) {
    if (y < BAR_H) {
        for (int x = 0; x < W; x++) out[x] = sw(C_BAR);
    } else if (y >= CHAN_Y0 && y < AXIS_Y0 && band_mode) {
        const int d = y - CHAN_Y0, k = d / BAND_TRACK_H;
        const bool line = k < BAND_TRACKS && d % BAND_TRACK_H == 5;
        for (int x = 0; x < W; x++)
            out[x] = sw(line && band_line[k][x] ? band_colors[band_line[k][x] - 1] : C_BAR);
    } else if (y >= CHAN_Y0 && y < AXIS_Y0) {
        /* lines on every other row of the strip's bottom half */
        const int d = y - (AXIS_Y0 - 2 * CHAN_LINES), line = d >= 0 && !(d & 1) ? d / 2 : -1;
        for (int x = 0; x < W; x++)
            out[x] = sw(line >= 0 && (chan_mask[x] >> line & 1) ? chan_colors[line] : C_BAR);
    } else if (y >= AXIS_Y0) {
        int d = y - AXIS_Y0;
        for (int x = 0; x < W; x++) {
            uint16_t c = C_BAR;
            if (colflag[x] == 2 && d < 2) c = C_TEXT;
            if (colflag[x] == 1 && d < 1) c = C_TICK;
            out[x] = sw(c);
        }
    } else if (y == div_y) {
        for (int x = 0; x < W; x++) out[x] = sw(C_DIV);
    } else if (spec_h && y >= spec_y0 && y < spec_y0 + spec_h) {
        bool level = false;
        for (int i = 0; i <= 4; i++) level |= y == level_rows[i];
        const uint8_t alpha = fill_alpha[y - spec_y0];
        /* background by colflag, and the same under the trace's fill */
        const uint16_t base[3] = {level ? C_LEVEL : C_BG, level ? C_LEVEL : C_MINOR, level ? C_LEVEL : C_MAJOR};
        /* byte-swapped once per row, not per pixel */
        const uint16_t back[3] = {sw(base[0]), sw(base[1]), sw(base[2])};
        const uint16_t fill[3] = {sw(tft.alphaBlend(alpha, C_TRACE, base[0])), sw(tft.alphaBlend(alpha, C_TRACE, base[1])),
                                  sw(tft.alphaBlend(alpha, C_TRACE, base[2]))};
        const uint16_t pal[6] = {back[0], fill[0], back[1], fill[1], back[2], fill[2]};
        const unsigned trace_c = sw(C_TRACE), hold_c = sw(C_HOLD);
        /* branch-free (table + conditional moves, one packed load per
         * column): 37 -> 24 cycles a pixel against if/else
         * on the LX6, which has no branch prediction */
        for (int x = 0; x < W; x++) {
            const uint32_t v = col_pack[x];
            const int t = v & 255, top = (v >> 8) & 255, bot = (v >> 16) & 255, p = v >> 24;
            const unsigned below = (unsigned)(t - y) >> 31; /* y > ty[x] */
            unsigned c = pal[2 * colflag[x] + below];
            c = (unsigned)(y - top) <= (unsigned)(bot - top) ? trace_c : c;
            c = y == p ? hold_c : c; /* py < seg_top - 1 or none: never on the trace */
            out[x] = (uint16_t)c;
        }
    } else if (wf_h && y >= wf_y0 && y < wf_y0 + wf_h) {
        int r = y - wf_y0;
        if (r < (int)ring_count) {
            const uint32_t *src = ring + ((ring_head + WF_ROWS - 1 - r) % WF_ROWS) * (W / 4);
            for (int x = 0; x < W; x += 4) {
                uint32_t w = src[x / 4];
                out[x] = wf_lut[w & 255];
                out[x + 1] = wf_lut[(w >> 8) & 255];
                out[x + 2] = wf_lut[(w >> 16) & 255];
                out[x + 3] = wf_lut[w >> 24];
            }
        } else
            for (int x = 0; x < W; x++) out[x] = sw(C_WF_EMPTY);
    } else
        for (int x = 0; x < W; x++) out[x] = sw(C_BG);
}

static volatile uint32_t render_us, render_n;
float ui_take_render(unsigned *count) {
    const uint32_t us = render_us, n = render_n;
    render_us = 0;
    render_n = 0;
    *count = n;
    return n ? us / 1000.0f / n : 0;
}

void ui_render() {
    const uint32_t t0 = micros();
    nlabels = 0;
    mark_x[0] = mark_x[1] = -1;
    layout();
    if (have_cur) prepare_frequency_axis();
    else memset(colflag, 0, sizeof(colflag));
    if (spec_h) prepare_spectrum();
    if (wf_h) prepare_waterfall();
    if (have_cur) prepare_channels();
    else {
        memset(chan_mask, 0, sizeof(chan_mask));
        band_mode = false;
    }
    prepare_status();
    const bool shot = ui_shot_request;
    if (shot) Serial.printf("TTGO SHOT %d %d\n", W, H);
    const uint32_t now = millis();
    const bool full = !panel_known || shot || now - full_push_ms >= 1000;
    if (full) full_push_ms = now;
    bool region_same[2];
    for (int k = 0; k < 2; k++) {
        const uint32_t key = static_key(k);
        region_same[k] = !full && key == static_keys[k];
        static_keys[k] = key;
    }
    unsigned next = 0;
    tft.startWrite();
    for (int s = 0; s < H; s += STRIP) {
        /* the other buffer may still be on its way to the panel; this one's
         * transfer finished before that one was queued */
        if ((s + STRIP <= STATIC_TOP && region_same[0]) || (s >= STATIC_BOTTOM && region_same[1])) continue;
        strip = &strips[next];
        uint16_t *buf = (uint16_t *)strip->getPointer();
        for (int r = 0; r < STRIP; r++) compose_row(s + r, buf + r * W);
        for (int i = 0; i < nlabels; i++) {
            const label_t *l = &labels[i];
            if (l->y + 8 <= s || l->y >= s + STRIP) continue;
            if (l->datum == TINY_DATUM) {
                draw_tiny(l, s);
                continue;
            }
            strip->setTextColor(l->color);
            strip->setTextDatum(l->datum);
            strip->drawString(l->text, l->x, l->y - s, 1);
        }
        for (int m = 0; m < 2; m++) {
            if (mark_x[m] < 0) continue;
            for (int r = 0; r < 3; r++) {
                const int y = mark_y[m] + r;
                if (y < s || y >= s + STRIP || (m == 0 && (y < spec_y0 || y >= spec_y0 + spec_h))) continue;
                strip->drawFastHLine(mark_x[m] - 2 + r, y - s, 5 - 2 * r, C_MARK);
            }
        }
        const uint32_t *words = (const uint32_t *)buf;
        /* FNV-1a over 32-bit words in two interleaved chains (half the
         * multiply latency of one) */
        uint32_t h0 = 2166136261u, h1 = 0x9e3779b9u;
        for (int i = 0; i < W * STRIP / 2; i += 2) {
            h0 = (h0 ^ words[i]) * 16777619u;
            h1 = (h1 ^ words[i + 1]) * 16777619u;
        }
        const uint32_t hash = h0 ^ (h1 * 0x85ebca6bu);
        if (full || hash != strip_hash[s / STRIP]) {
            tft.pushImageDMA(0, s, W, STRIP, (const uint16_t *)buf);
            strip_hash[s / STRIP] = hash;
            next ^= 1; /* a skipped strip's buffer is free for the next one */
        }
        if (shot) Serial.write((const uint8_t *)buf, W * STRIP * 2);
    }
    tft.dmaWait();
    tft.endWrite();
    panel_known = true;
    if (shot) ui_shot_request = false;
    render_us = render_us + (micros() - t0);
    render_n = render_n + 1;
}

/* ---------------- buttons ---------------- */
static void apply_local(const char *what) {
    engine_set_local(&local);
    save_prefs();
    /* link_owner() is a plain volatile read, safe from this loop task;
     * link_lease_active() mutates lease state and is engine-task-only */
    int owner = link_owner();
    if (owner != LINK_NONE)
        toast("%s (after %s)", what, owner == LINK_WS ? "WEB" : "USB");
    else
        toast("%s", what);
}

static void on_short(int pin) {
    if (pin == 0) {
        view = (view + 1) % VIEW_COUNT;
        save_prefs();
        toast("%s", view_names[view]);
    } else {
        local.band = (local.band + 1) % BAND_COUNT;
        static const char *const names[] = {"SCAN 2.4G",     "SCAN 2.3-2.5G", "SCAN 1.8-2.8G",
                                            "CH1 2401-2423", "CH6 2426-2448", "CH11 2451-2473"};
        apply_local(names[local.band]);
    }
}
static void on_long(int pin) {
    if (pin == 0) {
        avg_level = (avg_level + 1) % AVG_LEVELS;
        save_prefs();
        toast("%s", avg_names[avg_level]);
    } else {
        const unsigned count = sizeof(gain_steps) / sizeof(gain_steps[0]);
        unsigned i = 0;
        if (!local.agc)
            for (i = 1; i < count - 1 && gain_steps[i] < local.gain; i++) {}
        unsigned max = sdr_rx_gain_max();
        do i = (i + 1) % count;
        while (i && gain_steps[i] > max);
        local.agc = i == 0;
        if (i) local.gain = gain_steps[i];
        char text[16];
        if (local.agc) snprintf(text, sizeof(text), "GAIN AGC");
        else snprintf(text, sizeof(text), "GAIN %u", local.gain);
        apply_local(text);
    }
}

struct button_t {
    int pin;
    bool down, long_done;
    uint32_t since;
};
static button_t buttons[2] = {{0}, {35}};

void ui_poll(uint32_t now) {
    for (button_t &b : buttons) {
        bool down = digitalRead(b.pin) == LOW;
        if (down != b.down) engine_get_local(&local); /* may have changed over serial */
        if (down && !b.down) {
            b.since = now;
            b.long_done = false;
        } else if (down && !b.long_done && now - b.since >= 700) {
            b.long_done = true;
            on_long(b.pin);
        } else if (!down && b.down && !b.long_done && now - b.since >= 30) {
            on_short(b.pin);
        }
        b.down = down;
    }
}

/* ---------------- setup ---------------- */
void ui_init() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    pinMode(0, INPUT_PULLUP);
    pinMode(35, INPUT); /* external pull-up on the board */
    tft.init();
    tft.setRotation(1); /* landscape 240x135 */
    tft.fillScreen(C_BG);
    ring = (uint32_t *)heap_caps_malloc(WF_ROWS * W, MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
    if (!ring) ring = (uint32_t *)heap_caps_malloc(WF_ROWS * W, MALLOC_CAP_8BIT);
    for (TFT_eSprite &sp : strips) {
        sp.setColorDepth(16);
        sp.createSprite(W, STRIP);
    }
    tft.initDMA();
    palette_init();

    prefs.begin("ttgo-sdr", false);
    view = prefs.getUChar("view", VIEW_SPEC) % VIEW_COUNT;
    avg_level = prefs.getUChar("avg2", AVG_DEFAULT) % AVG_LEVELS;
    engine_get_local(&local);
    local.band = prefs.getUChar("band", BAND_SCAN) % BAND_COUNT;
    local.agc = prefs.getUChar("agc", 0) ? 1 : 0;
    local.gain = prefs.getUChar("gain", DEFAULT_GAIN);
    local.legacy = 0;
    engine_set_local(&local);

    tft.setTextColor(C_TRACE, C_BG);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("TTGO  ESP32  SDR", W / 2, 22, 4);
    tft.setTextDatum(TL_DATUM);
}

void ui_boot_line(const char *text) {
    static int y = 56;
    tft.setTextColor(C_TEXT, C_BG);
    tft.drawString(text, 16, y, 2);
    y += 16;
}

float ui_take_jitter(void) {
    float mean = jit_n ? jit_sum / jit_n : 0;
    jit_sum = 0;
    jit_n = 0;
    return mean;
}

void ui_set_averaging(unsigned level) {
    if (level < AVG_LEVELS) avg_level = (uint8_t)level;
}

void ui_set_view(unsigned v) {
    if (v < VIEW_COUNT) view = (uint8_t)v;
}
