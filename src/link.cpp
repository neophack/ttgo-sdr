/* link.cpp — esp-sdr burst protocol server (ESP32 profile, protocol 6).
 *
 * Command set and replies follow ESPARGOS/esp-sdr (GPL-3.0-or-later):
 * main/common/burst_commands.h, burst_capture.h, spectrum.c and
 * main/targets/esp32/receiver.c, so the unmodified esp-web-sdr radio.js
 * negotiates it like a USB-attached ESP32 running ESP-SDR.
 *
 * Deviations, all within what the protocol and viewer allow:
 *   - SPEC snapshots capture n*k samples and report k FFTs per frame
 *     (upstream ESP32: one FFT per capture), lowering per-frame variance
 *   - every capture hands the radio back to the SoftAP before the FFTs
 *   - "TTGO ..." extension commands (diagnostics, local screen settings)
 *     never take the lease
 */
#include "link.h"
#include <Arduino.h>
#include <stdarg.h>
#include "esp_rom_crc.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "dsp.h"
#include "engine.h"
#include "sdr_rx.h"
#include "ui_tft.h"

#define LEASE_US       5000000
#define CMD_LINE_MAX   128
#define SPEC_SAMPLES   8192u /* per snapshot: 32x256 .. 4x2048 FFTs */
#define UI_MIRROR_US   66000 /* screen mirror of client spectra: <=15 fps */
#define UART_BAUD_BOOT 2000000

/* ---------------- ports ---------------- */
typedef struct port {
    int id;                /* LINK_UART / LINK_WS */
    const char *transport; /* TRANSPORT? answer */
    int (*read)(uint8_t *buf, size_t max);
    bool (*send)(const void *data, size_t len);
    bool (*connected)(void);
    unsigned spec_interval_us; /* minimum SPEC frame spacing (airtime for WiFi) */
    char line[CMD_LINE_MAX];
    size_t used;
    bool overflow;
    bool held; /* line[] was completed by stop_requested(), not yet run */
    int64_t last_byte;
} port_t;

static unsigned uart_baud = UART_BAUD_BOOT;
static int64_t uart_activity_us = -LEASE_US;
static int uart_read(uint8_t *buf, size_t max) {
    size_t n = 0;
    while (n < max && Serial.available() > 0) buf[n++] = (uint8_t)Serial.read();
    if (n) uart_activity_us = esp_timer_get_time();
    return (int)n;
}
static bool uart_send(const void *data, size_t len) {
    return Serial.write((const uint8_t *)data, len) == len;
}
static bool uart_connected(void) { return true; }

static StreamBufferHandle_t ws_rx;
static volatile int ws_fd = -1;
static volatile int64_t ws_activity_us = -LEASE_US;
static int ws_read(uint8_t *buf, size_t max) {
    return ws_rx ? (int)xStreamBufferReceive(ws_rx, buf, max, 0) : 0;
}
static bool ws_send(const void *data, size_t len) {
    int fd = ws_fd;
    return fd >= 0 && web_ws_send(fd, data, len);
}
static bool ws_connected(void) { return ws_fd >= 0; }

static port_t uart_port = {LINK_UART, "UART", uart_read, uart_send, uart_connected, 0};
static port_t ws_port = {LINK_WS, "USB", ws_read, ws_send, ws_connected, 30000};
static port_t *const ports[] = {&uart_port, &ws_port};

/* ---------------- lease ---------------- */
static port_t *owner;
static int64_t lease_until;
static volatile int owner_id;

bool link_lease_active(void) {
    if (!owner) return false;
    if (esp_timer_get_time() < lease_until && owner->connected()) return true;
    owner = nullptr;
    owner_id = LINK_NONE;
    return false;
}
int link_owner(void) { return owner_id; }
bool link_uart_quiet(void) {
    return owner_id == LINK_UART || esp_timer_get_time() - uart_activity_us < LEASE_US;
}
bool link_input_pending(void) {
    return Serial.available() > 0 || (ws_rx && xStreamBufferBytesAvailable(ws_rx) > 0);
}

/* httpd task. A second browser gets "ERR busy" per line while the current
 * WebSocket client is active; an idle one is replaced. */
void link_ws_input(int fd, const uint8_t *data, size_t len) {
    int64_t now = esp_timer_get_time();
    /* a client stays quiet while a long command (SPEC) streams to it, but
     * it still holds the lease then; owner_id is a plain volatile read */
    if (fd != ws_fd && (ws_fd < 0 || (now - ws_activity_us > LEASE_US && owner_id != LINK_WS))) {
        ws_fd = fd;
        xStreamBufferReset(ws_rx);
    }
    if (fd != ws_fd) {
        for (size_t i = 0; i < len; i++)
            if (data[i] == '\n') web_ws_send_now(fd, "ERR busy\n");
        return;
    }
    ws_activity_us = now;
    xStreamBufferSend(ws_rx, data, len, 0);
}
void link_ws_closed(int fd) {
    if (fd == ws_fd) ws_fd = -1;
}

static void reply(port_t *p, const char *fmt, ...) {
    char text[384]; /* TTGO STAT is the longest reply */
    va_list ap;
    va_start(ap, fmt);
    int length = vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    if (length > 0) p->send(text, (size_t)length < sizeof(text) ? (size_t)length : sizeof(text) - 1);
}

/* ---------------- receiver session (the client's view) ---------------- */
static struct {
    unsigned freq = 2412;
    bool agc = true;
    unsigned gain = 40;
    int filter = -1;
    unsigned dc_mode = 1;
} sess;

static void session_apply(void) {
    sdr_rx_set_frequency(sess.freq);
    sdr_rx_set_gain(sess.agc, sess.gain);
    sdr_rx_set_filter(sess.filter);
}

/* Rate codes 0, 1 and 6 (80, 40 and 16 MS/s) are dump clocks 1, 2 and 0. */
static unsigned rate_hz(unsigned code) {
    return code == 0 ? 80000000 : code == 1 ? 40000000 : code == 6 ? 16000000 : 0;
}
static unsigned rate_clock(unsigned code) {
    return code == 0 ? SDR_CLK_80M : code == 1 ? SDR_CLK_40M : SDR_CLK_16M;
}

/* esp-sdr burst_parse.h: strict unsigned decimal fields. */
static int parse_uints(const char *s, unsigned *v, int max) {
    int n = 0;
    for (;;) {
        while (*s == ' ') s++;
        if (!*s) return n;
        if (n == max || *s < '0' || *s > '9') return -1;
        unsigned x = 0;
        do {
            unsigned d = (unsigned)(*s++ - '0');
            if (x > (~0u - d) / 10u) return -1;
            x = x * 10u + d;
        } while (*s >= '0' && *s <= '9');
        if (*s && *s != ' ') return -1;
        v[n++] = x;
    }
}

/* ---------------- screen mirror ---------------- */
static ui_frame_t mirror;
static int64_t mirror_at;
static void mirror_publish(port_t *p, const float *pow, unsigned n, bool negated, float scale,
                           unsigned fs, unsigned gain) {
    engine_frame_from_bins(&mirror, pow, n, negated, scale, sess.freq, fs);
    mirror.source = p->id == LINK_UART ? SRC_USB : SRC_WEB;
    mirror.gain_agc = sess.agc;
    mirror.gain_code = sess.gain;
    mirror.gain_seen = gain;
    engine_publish(&mirror);
    mirror_at = esp_timer_get_time();
}
static bool mirror_due(void) { return esp_timer_get_time() - mirror_at >= UI_MIRROR_US; }

/* ---------------- CAP / CAP16 / CAP20 (burst_capture.h) ---------------- */
static void pack_iq(uint32_t *buf, unsigned n) {
    uint8_t *p = (uint8_t *)buf;
    for (unsigned j = 0; j < n; j += 2, p += 5) {
        uint32_t a = buf[j] & 0xfffffu;
        uint32_t b = j + 1 < n ? buf[j + 1] & 0xfffffu : 0;
        p[0] = a;
        p[1] = a >> 8;
        p[2] = (a >> 16) | (b << 4);
        if (j + 1 < n) {
            p[3] = b >> 4;
            p[4] = b >> 12;
        }
    }
}
static void pack_iq8(uint32_t *buf, unsigned n) {
    uint8_t *p = (uint8_t *)buf;
    for (unsigned j = 0; j < n; j++) {
        uint32_t w = buf[j];
        p[2 * j] = (w >> 2) & 255;
        p[2 * j + 1] = (w >> 12) & 255;
    }
}

static void cmd_capture(port_t *p, unsigned n, unsigned rate, unsigned format) {
    if (rate != 0 && rate != 1 && rate != 6) {
        reply(p, "ERR rate\n");
        return;
    }
    const uint32_t *words;
    session_apply();
    int us = sdr_rx_capture_burst(n, rate_clock(rate), &words, AP_HOME_MHZ);
    if (us < 0) {
        reply(p, us == -2 ? "ERR capture_timeout\n" : "ERR capture_memory 0\n");
        return;
    }
    if (mirror_due() && n >= 1024) {
        unsigned segs = n / 256 < 16 ? n / 256 : 16;
        float *mirror_pow = dsp_scratch();
        memset(mirror_pow, 0, 256 * sizeof(float));
        dsp_welch(words, 256, segs, DSP_NEGATE_Q | DSP_REMOVE_MEAN, mirror_pow, NULL);
        mirror_publish(p, mirror_pow, 256, true, 1.0f / segs, rate_hz(rate), (words[0] >> 20) & 0xff);
    }
    uint32_t *buf = sdr_rx_buffer();
    size_t bytes = format == 16 ? n * 2 : format == 20 ? (n * 20u + 7u) / 8u : n * 4;
    if (format == 16)
        pack_iq8(buf, n);
    else if (format == 20)
        pack_iq(buf, n);
    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)buf, bytes);
    reply(p, "DATA %u %08lx %u\n", n, (unsigned long)crc, (unsigned)us);
    p->send(buf, bytes);
}

/* ---------------- SPEC (spectrum.c, snapshot profile) ---------------- */
static uint8_t frame[28 + DSP_MAX_FFT + 4];
static void put16(unsigned at, uint16_t v) {
    frame[at] = v;
    frame[at + 1] = v >> 8;
}
static void put32(unsigned at, uint32_t v) {
    put16(at, v);
    put16(at + 2, v >> 16);
}
static void put64(unsigned at, uint64_t v) {
    put32(at, (uint32_t)v);
    put32(at + 4, (uint32_t)(v >> 32));
}

static void spec_capabilities(port_t *p) {
    char text[640];
    size_t at = snprintf(text, sizeof(text),
                         "SPECINFO {\"continuous\":false,\"transports\":[\"USB\",\"UART\"],\"profiles\":[");
    static const unsigned codes[] = {0, 1, 6};
    bool first = true;
    for (unsigned r = 0; r < 3; r++)
        for (unsigned n = 256; n <= DSP_MAX_FFT; n *= 2) {
            at += snprintf(text + at, sizeof(text) - at, "%s[%u,%u,%u,1,1,0]", first ? "" : ",",
                           rate_hz(codes[r]), codes[r], n);
            first = false;
        }
    at += snprintf(text + at, sizeof(text) - at, "]}\n");
    p->send(text, at);
}

static void spec_stats(port_t *p, int64_t dt_us, uint64_t busy_us, uint32_t ffts, unsigned n, unsigned fs) {
    uint8_t b[40];
    memset(b, 0, sizeof(b));
    uint32_t load = (uint32_t)(busy_us * 1000 / (uint64_t)dt_us);
    uint64_t coverage = (uint64_t)ffts * n * 1000000000ull / ((uint64_t)dt_us * fs);
    uint32_t v32;
    uint16_t v16;
    memcpy(b, "SPS1", 4);
    v16 = load > 1000 ? 1000 : load;
    memcpy(b + 4, &v16, 2);
    v16 = coverage > 1000 ? 1000 : (uint16_t)coverage;
    memcpy(b + 8, &v16, 2);
    v32 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    memcpy(b + 12, &v32, 4);
    v32 = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    memcpy(b + 16, &v32, 4);
    v32 = (uint32_t)((uint64_t)ffts * 1000000 / (uint64_t)dt_us);
    memcpy(b + 32, &v32, 4);
    v32 = esp_rom_crc32_le(0, b, 36);
    memcpy(b + 36, &v32, 4);
    p->send(b, sizeof(b));
}

/* Line assembly shared by poll_line() and stop_requested(). Feeds one byte;
 * true when '\n' ended the line. */
static bool line_feed(port_t *p, uint8_t ch) {
    if (ch == '\n') return true;
    if (ch == '\r') return false;
    if (p->used < CMD_LINE_MAX - 1)
        p->line[p->used++] = (char)ch;
    else
        p->overflow = true;
    return false;
}
/* After '\n': 1 = command in line[], -1 = too long, 0 = empty line. */
static int line_done(port_t *p) {
    size_t used = p->used;
    bool overflow = p->overflow;
    p->used = 0;
    p->overflow = false;
    if (overflow) return -1;
    if (!used) return 0;
    p->line[used] = 0;
    return 1;
}

/* Any input stops a SPEC stream (radio.js sends an empty line). The bytes
 * still go through the port's line assembly, so a command sent with the stop
 * runs after the stream (poll_line picks it up) and one split across the
 * stream's end is completed by poll_line instead of being cut in two.
 * cmd_spec() has parsed its arguments out of line[] before this rewrites it. */
static bool stop_requested(port_t *p) {
    uint8_t c;
    bool stopped = false;
    for (unsigned j = 0; j < CMD_LINE_MAX && p->read(&c, 1) == 1; j++) {
        stopped = true;
        p->last_byte = esp_timer_get_time();
        if (line_feed(p, c)) {
            p->held = p->used || p->overflow;
            break;
        }
    }
    return stopped || !p->connected();
}

static void cmd_spec(port_t *p, const char *args) {
    unsigned v[7] = {0};
    int fields = parse_uints(args, v, 7);
    unsigned ms = v[0], stride = v[1], units = v[2], det = v[3], rate = v[4], n = v[5],
             stats = fields == 7 ? v[6] : 0;
    if ((fields != 6 && fields != 7) || stats > 1 || ms > 86400000u || stride != 1 || !units ||
        units > 8 || det > 1 || !rate_hz(rate) || n < 256 || n > DSP_MAX_FFT || (n & (n - 1))) {
        reply(p, "ERR spec_args\n");
        return;
    }
    const unsigned fs = rate_hz(rate), segs = SPEC_SAMPLES / n;
    unsigned log2n = 0;
    while ((1u << log2n) < n) log2n++;
    reply(p, "SPEC %u %u %u %u\n", n, fs, n, sess.freq);
    const int64_t start = esp_timer_get_time();
    int64_t stats_at = start, last_frame = start - p->spec_interval_us;
    uint64_t pairs = 0, busy_us = 0;
    uint32_t frames = 0, ffts = 0, ffts_at_stats = 0, status = 0, failures = 0;
    bool stopped = false;
    dsp_dc_t dc = {0, 0, false, sess.dc_mode};
    float *spec_pow = dsp_scratch();
    do {
        if (stop_requested(p)) {
            stopped = true;
            break;
        }
        /* leave the access point airtime to send the previous frame */
        while (esp_timer_get_time() - last_frame < (int64_t)p->spec_interval_us) vTaskDelay(1);
        last_frame = esp_timer_get_time();
        const uint64_t index = (uint64_t)(last_frame - start) * fs / 1000000u;
        const uint32_t *words;
        session_apply();
        int us = sdr_rx_capture_burst(n * segs, rate_clock(rate), &words, AP_HOME_MHZ);
        if (us < 0) {
            /* snapshots are independent: skip a failed one (flag bit 3 already
             * tells the viewer there are gaps); give up only if the receiver
             * stays unusable */
            if (++failures < 20) continue;
            status = 1;
            break;
        }
        failures = 0;
        const int64_t t0 = esp_timer_get_time();
        memset(spec_pow, 0, n * sizeof(float));
        dsp_welch(words, n, segs, det ? DSP_MAX : 0, spec_pow, &dc);
        const float scale = det ? 1.0f : 1.0f / segs;
        memcpy(frame, "SPC1", 4);
        put32(4, frames);
        put64(8, index);
        put32(16, n * segs);
        put16(20, segs);
        frame[22] = 8 | (det ? 1 : 0); /* bit 3: snapshot gaps */
        frame[23] = (words[0] >> 20) & 0xff;
        put16(24, 0);
        frame[26] = log2n;
        frame[27] = 2;
        for (unsigned j = 0; j < n; j++) frame[28 + j] = dsp_power_code(spec_pow[j] * scale);
        put32(28 + n, esp_rom_crc32_le(0, frame, 28 + n));
        ffts += segs;
        pairs += n * segs;
        if (mirror_due()) mirror_publish(p, spec_pow, n, false, scale, fs, frame[23]);
        busy_us += (uint64_t)(esp_timer_get_time() - t0) + (unsigned)us;
        if (stats && esp_timer_get_time() - stats_at >= 250000) {
            int64_t now = esp_timer_get_time();
            spec_stats(p, now - stats_at, busy_us, ffts - ffts_at_stats, n, fs);
            stats_at = now;
            busy_us = 0;
            ffts_at_stats = ffts;
        }
        if (!p->send(frame, 28 + n + 4)) {
            status = 7;
            break;
        }
        frames++;
        vTaskDelay(1);
    } while (!ms || esp_timer_get_time() - start < (int64_t)ms * 1000);
    reply(p, "SPECEND %u 0 %lu %llu %lld 0 0 %lu 0 0 %lu %u\n", (unsigned)status, (unsigned long)ffts,
          (unsigned long long)pairs, (long long)(esp_timer_get_time() - start), (unsigned long)frames,
          (unsigned long)ffts, stopped ? 1u : 0u);
}

/* ---------------- VERSION? ---------------- */
static void cmd_version(port_t *p) {
    /* __DATE__ "Oct 19 2026", __TIME__ "13:45:10" -> ISO 8601 UTC-labelled */
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *d = __DATE__;
    char name[4] = {d[0], d[1], d[2], 0};
    int month = (int)(strstr(months, name) - months) / 3 + 1;
    int day = (d[4] == ' ' ? 0 : d[4] - '0') * 10 + (d[5] - '0');
    reply(p,
          "VERSION {\"revision\":\"unknown\",\"build_date\":\"%.4s-%02d-%02d\","
          "\"build_timestamp\":\"%.4s-%02d-%02dT%.8sZ\",\"profile\":\"esp32\"}\n",
          d + 7, month, day, d + 7, month, day, __TIME__);
}

/* ---------------- TTGO extensions (no lease) ---------------- */
static bool ttgo_command(port_t *p, const char *line) {
    if (strncmp(line, "TTGO ", 5)) return false;
    line += 5;
    local_cfg_t cfg;
    engine_get_local(&cfg);
    unsigned v[4];
    if (!strcmp(line, "STAT?")) {
        engine_stats_t es;
        engine_take_stats(&es);
        const sdr_stats_t *rs = sdr_rx_stats();
        unsigned renders;
        const float render_ms = ui_take_render(&renders);
        reply(p,
              "TTGO STAT {\"frames\":%lu,\"jitter_raw\":%.2f,\"jitter_screen\":%.2f,\"floor\":%.1f,"
              "\"frame_ms\":%.1f,\"radio_ms\":%.1f,\"render_ms\":%.1f,\"renders\":%u,\"dsp_fast\":%d,"
              "\"agc_moves\":%lu,\"gain\":[%lu,%lu],\"captures\":%lu,\"errors\":%lu,"
              "\"count_mismatch\":%lu,\"stuck\":%lu,\"retune_us\":%lu,\"heap\":%lu,\"stack_free\":%u,\"owner\":%d}\n",
              (unsigned long)es.frames, es.jitter_raw_db, ui_take_jitter(), es.floor_db, es.frame_ms,
              es.radio_ms, render_ms, renders, (int)dsp_fast(),
              (unsigned long)es.agc_moves, (unsigned long)es.gain_min, (unsigned long)es.gain_max,
              (unsigned long)rs->captures, (unsigned long)rs->capture_errors,
              (unsigned long)rs->count_mismatch, (unsigned long)rs->stuck_captures, (unsigned long)rs->last_retune_us,
              (unsigned long)esp_get_free_heap_size(), engine_stack_free(), owner_id);
    } else if (!strcmp(line, "SHOT?")) {
        /* the UI task writes the reply to UART0: from the WebSocket it would
         * land in a USB client's byte stream, not here */
        if (p->id != LINK_UART) {
            reply(p, "ERR shot_transport\n");
            return true;
        }
        /* stay off UART0 until the UI task is done */
        ui_shot_request = true;
        for (int t = 0; t < 300 && ui_shot_request; t++) vTaskDelay(pdMS_TO_TICKS(10));
        if (ui_shot_request) {
            ui_shot_request = false;
            reply(p, "ERR shot\n");
        }
    } else if (!strcmp(line, "FRAME?")) {
        /* newest display frame: "TTGO FRAME seq source f0 f1 dB*10 x240" */
        static ui_frame_t f;
        static char text[64 + UI_COLS * 7]; /* " -1400" worst case per column */
        engine_latest(&f);
        size_t at = snprintf(text, sizeof(text), "TTGO FRAME %lu %u %.3f %.3f", (unsigned long)f.seq, f.source,
                             f.f0_mhz, f.f1_mhz);
        for (unsigned c = 0; c < UI_COLS; c++)
            at += snprintf(text + at, sizeof(text) - at, " %d", (int)lrintf(f.db[c] * 10));
        at += snprintf(text + at, sizeof(text) - at, "\n");
        p->send(text, at);
    } else if (!strncmp(line, "VIEW ", 5) && parse_uints(line + 5, v, 1) == 1 && v[0] < 3) {
        ui_set_view(v[0]);
        reply(p, "OK\n");
    } else if (!strncmp(line, "AVG ", 4) && parse_uints(line + 4, v, 1) == 1 && v[0] < 4) {
        ui_set_averaging(v[0]);
        reply(p, "OK\n");
    } else if (!strcmp(line, "LOCAL?")) {
        reply(p, "TTGO LOCAL %u %u %u %u\n", cfg.band, cfg.agc, cfg.gain, cfg.legacy);
    } else if (!strncmp(line, "LOCAL ", 6) && parse_uints(line + 6, v, 4) == 4 && v[0] < BAND_COUNT &&
               v[1] <= 1 && v[2] <= sdr_rx_gain_max() && v[3] <= 1) {
        cfg = {(uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3]};
        engine_set_local(&cfg);
        reply(p, "OK\n");
    } else
        reply(p, "ERR command\n");
    return true;
}

/* ---------------- dispatcher ---------------- */
static void handle(port_t *p, char *line) {
    unsigned v[4];
    bool iq8 = false;
    if (!strncmp(line, "CAP16 ", 6)) { /* CAP16 is CAP20's capture packed as 8-bit I/Q */
        memcpy(line, "CAP20", 5);
        iq8 = true;
    }
    if (!strcmp(line, "INFO"))
        reply(p, "ESP32SDR 6 burst %u\n", SDR_MAX_SAMPLES);
    else if (!strcmp(line, "CAPS"))
        reply(p, "CAPS VERSION SPEC SPECN SPECCAPS SPECSTAT DCT UARTBAUD RXLIMITS SERIALLEASE TUNEEXT "
                 "RX40 RX16 LPFANA GAIN HWAGC IQ8\n");
    else if (!strncmp(line, "SYNC ", 5) && line[5] >= '0' && line[5] <= '9' &&
             strspn(line + 5, "0123456789") == strlen(line + 5))
        reply(p, "SYNC %s\n", line + 5);
    else if (!strcmp(line, "VERSION?"))
        cmd_version(p);
    else if (!strcmp(line, "TRANSPORT?"))
        reply(p, "TRANSPORT %s %u\n", p->transport, p->id == LINK_UART ? uart_baud : 0);
    else if (!strcmp(line, "RANGE?"))
        reply(p, "RANGE %u %u 1\n", 1850u, 2850u);
    else if (!strncmp(line, "FREQ ", 5) && parse_uints(line + 5, v, 1) == 1 && v[0] >= 1850 &&
             v[0] <= 2850) {
        sess.freq = v[0];
        reply(p, "OK\n");
    } else if (!strcmp(line, "LIMITS?"))
        reply(p,
              "LIMITS {\"gain\":[0,%u,1],\"bandwidth\":[%u,%u,1,0],\"rates\":[80000000,40000000,"
              "16000000],\"bits\":[8,10]}\n",
              sdr_rx_gain_max(), SDR_BANDWIDTH_MIN, SDR_BANDWIDTH_MAX);
    else if (!strcmp(line, "GAIN?"))
        reply(p, "GAIN %s %d 0 %u %u\n", sess.agc ? "HARDWARE" : "MANUAL", sess.agc ? -1 : (int)sess.gain,
              sdr_rx_gain_max(), sess.agc ? 0u : 1u);
    else if (!strcmp(line, "GAIN HARDWARE")) {
        sess.agc = true;
        reply(p, "OK\n");
    } else if (!strncmp(line, "GAIN MANUAL ", 12) && parse_uints(line + 12, v, 1) == 1 &&
               v[0] <= sdr_rx_gain_max()) {
        sess.agc = false;
        sess.gain = v[0];
        reply(p, "OK\n");
    } else if (!strncmp(line, "BANDWIDTH ", 10) && parse_uints(line + 10, v, 1) == 1 &&
               (!v[0] || (v[0] >= SDR_BANDWIDTH_MIN && v[0] <= SDR_BANDWIDTH_MAX))) {
        sess.filter = (int)sdr_rx_bandwidth_dcap(v[0]);
        reply(p, "OK\n");
    } else if (!strcmp(line, "LPF AUTO")) {
        sess.filter = -1;
        reply(p, "OK\n");
    } else if (!strncmp(line, "LPF ", 4) && parse_uints(line + 4, v, 1) == 1 && v[0] <= 127) {
        sess.filter = (int)v[0];
        reply(p, "OK\n");
    } else if (!strcmp(line, "LPF?"))
        reply(p, "LPF %d %d %d\n", sess.filter, sess.filter, sess.filter);
    else if (!strcmp(line, "DC?"))
        reply(p, "DC %u\n", sess.dc_mode);
    else if (!strncmp(line, "DC ", 3) && parse_uints(line + 3, v, 1) == 1 && v[0] < 2) {
        sess.dc_mode = v[0];
        reply(p, "OK\n");
    } else if (!strcmp(line, "SPECINFO?"))
        spec_capabilities(p);
    else if (!strncmp(line, "SPEC ", 5))
        cmd_spec(p, line + 5);
    else if ((!strncmp(line, "CAP20 ", 6) && parse_uints(line + 6, v, 2) == 2) ||
             (!strncmp(line, "CAP ", 4) && parse_uints(line + 4, v, 2) == 2)) {
        if (v[0] < 256 || v[0] > SDR_MAX_SAMPLES || v[1] > 6)
            reply(p, "ERR command\n");
        else
            cmd_capture(p, v[0], v[1], line[3] == '2' ? (iq8 ? 16 : 20) : 0);
    } else if (!strcmp(line, "BAUD?")) {
        if (p->id == LINK_UART)
            reply(p, "BAUD %u\n", uart_baud);
        else
            reply(p, "ERR baud_transport\n");
    } else if (!strncmp(line, "BAUD", 4)) {
        unsigned baud = !strcmp(line, "BAUD 1000000") ? 1000000 : !strcmp(line, "BAUD 2000000") ? 2000000 : 0;
        if (p->id != LINK_UART)
            reply(p, "ERR baud_transport\n");
        else if (!baud)
            reply(p, "ERR baud_args\n");
        else {
            reply(p, "OK BAUD %u\n", baud);
            Serial.flush();
            Serial.updateBaudRate(baud);
            uart_baud = baud;
            p->used = 0;
        }
    } else
        reply(p, "ERR command\n");
}

static void process(port_t *p, char *line) {
    if (ttgo_command(p, line)) return;
    bool active = link_lease_active();
    if (active && owner != p) {
        reply(p, "ERR busy\n");
        return;
    }
    if (!strcmp(line, "RELEASE")) {
        reply(p, "OK\n");
        owner = nullptr;
        owner_id = LINK_NONE;
        return;
    }
    owner = p;
    owner_id = p->id;
    lease_until = esp_timer_get_time() + LEASE_US;
    sdr_log_quiet = sdr_log_quiet || p->id == LINK_UART;
    handle(p, line);
    /* ownership covers the whole binary transaction */
    lease_until = esp_timer_get_time() + LEASE_US;
}

/* Assemble one line from a port (esp-sdr burst_serial_poll_line). */
static int poll_line(port_t *p) {
    if (p->held) { /* completed while a SPEC stream ran */
        p->held = false;
        return line_done(p);
    }
    int64_t now = esp_timer_get_time();
    if (now - p->last_byte > 3000000) {
        p->used = 0;
        p->overflow = false;
    }
    uint8_t ch;
    for (unsigned k = 0; k < CMD_LINE_MAX && p->read(&ch, 1) == 1; k++) {
        p->last_byte = now;
        if (!line_feed(p, ch)) continue;
        int status = line_done(p);
        if (status) return status;
    }
    return 0;
}

bool link_service(void) {
    static unsigned next;
    if (!link_uart_quiet()) sdr_log_quiet = false;
    for (unsigned j = 0; j < 2; j++) {
        port_t *p = ports[(next + j) % 2];
        int status = poll_line(p);
        if (!status) continue;
        next = (next + j + 1) % 2;
        if (status < 0)
            reply(p, "ERR command_length\n");
        else
            process(p, p->line);
        return true;
    }
    return false;
}

void link_init(void) {
    if (!ws_rx) ws_rx = xStreamBufferCreate(2048, 1);
}
