/* engine.h — the radio-owning task: local TFT scans when no protocol client
 * holds the lease, plus the frame mailbox the TFT renders from. */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AP_HOME_MHZ 2412 /* SoftAP channel 1; the radio parks here between bursts */
#define UI_COLS 240

enum { SRC_SCAN = 0, SRC_ZOOM = 1, SRC_WEB = 2, SRC_USB = 3 };

/* One display-ready spectrum: UI_COLS columns of mean power in dBFS. */
typedef struct {
    uint32_t seq;
    uint8_t source;    /* SRC_* */
    uint8_t gain_agc;  /* 1 = hardware AGC */
    uint8_t gain_code; /* manual index (gain_agc == 0) */
    uint8_t gain_seen; /* index reported in the first I/Q word */
    uint16_t lo_mhz;
    uint32_t rate_hz;
    float f0_mhz, f1_mhz; /* left edge of column 0, right edge of the last */
    float db[UI_COLS];
} ui_frame_t;

/* Local (button-driven) receiver settings. */
enum { BAND_SCAN = 0, BAND_WIDE, BAND_XWIDE, BAND_CH1, BAND_CH6, BAND_CH11, BAND_COUNT };
/* BAND_SCAN 2400..2500 MHz, BAND_WIDE 2300..2500 MHz, BAND_XWIDE 1800..2800 MHz,
 * BAND_CHn 40 MS/s zoom */
typedef struct {
    uint8_t band;   /* BAND_* */
    uint8_t agc;    /* 1 = hardware AGC */
    uint8_t gain;   /* manual gain index */
    uint8_t legacy; /* 1 = previous firmware's sweep pipeline (A/B diagnostics) */
} local_cfg_t;

void engine_start(void);
uint32_t engine_latest(ui_frame_t *out);
void engine_publish(ui_frame_t *frame); /* engine task only; assigns seq */
void engine_get_local(local_cfg_t *out);
void engine_set_local(const local_cfg_t *in);

/* Frame-to-frame stability of local frames since the last call (resets):
 * mean |dB(t) - dB(t-1)| per column of the raw frames. */
typedef struct {
    uint32_t frames;
    float jitter_raw_db;
    float floor_db; /* 20th percentile of the last frame */
    float frame_ms; /* average local frame compute time */
    uint32_t agc_moves; /* captures whose gain index changed mid-capture */
    uint32_t gain_min, gain_max;
} engine_stats_t;
void engine_take_stats(engine_stats_t *out);
unsigned engine_stack_free(void); /* bytes never used on the engine stack */

/* Helpers shared with the protocol (link.cpp): column-map an n-bin power
 * array in natural FFT order onto a ui_frame (RF-oriented, centred on lo). */
void engine_frame_from_bins(ui_frame_t *f, const float *pow, unsigned n, bool negated_q, float scale,
                            unsigned lo_mhz, unsigned rate_hz);

#ifdef __cplusplus
}
#endif
