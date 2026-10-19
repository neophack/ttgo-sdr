/* sdr_rx.c — ESP32 RF capture backend for the TTGO T-Display SDR.
 *
 * Ported from ESPARGOS/esp-sdr (GPL-3.0-or-later):
 *   main/targets/esp32/receiver.c — dump-engine capture, tuning, gain, filter
 *   main/common/rx_lo.h, rx_bandwidth.h — LO plan, analog filter curve
 * Register sequence originally derived from ESP-IDF's ESP32 librftest.a.
 *
 * Differences from upstream:
 *   - radio sharing with a SoftAP: sdr_rx_radio_restore() re-enables TX
 *     (ROM rom_pbus_xpd_tx_on), releases a forced RX gain and returns to the
 *     AP channel between bursts; upstream keeps TX off forever
 *   - no retune DC recalibration: upstream patches private PHY-archive data
 *     (chip7_sleep_params, DC tables) whose layout differs in the Arduino
 *     core's PHY build. DC offset is removed in the DSP instead (dsp.c)
 *   - only the first n+64 words carry sentinels: the guard still detects a
 *     short or overrunning capture at a fraction of the cost for small n
 */
#include "sdr_rx.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/soc.h"
#include "soc/rtc.h"
#include "soc/dport_reg.h"
#include "heap_memory_layout.h"

volatile bool sdr_log_quiet;
#define SDR_TRACE(fmt, ...)                                                                        \
    do {                                                                                           \
        if (!sdr_log_quiet) esp_rom_printf("[rx] " fmt "\n", ##__VA_ARGS__);                     \
    } while (0)

/* Reserve both SRAM banks, including their instruction aliases. Mode 2 only
 * exposes 32 KiB to the MAC; mode 3 exposes the entire 64 KiB capture
 * aperture. (esp-sdr) */
SOC_RESERVE_MEMORY_REGION(0x3ffe8000, 0x3fff8000, esp32_rf_dump);

#define DUMP_CTRL   0x60033d90u
#define DUMP_STATUS 0x60033d94u
#define DUMP_BYTES  0x60033dc4u
#define RX_GAIN     0x3ff5c02cu
#define CAPACITY    16384u
#define SENTINEL    0xa55aa55au
static uint32_t *const samples = (void *)0x3ffe8000;

/* ---- ROM / libphy entry points (ESP32) ---- */
extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern unsigned ram_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void ram_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(int);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_pbus_xpd_tx_on(void);
extern void rom_set_rxclk_en(int);
extern void set_chanfreq(unsigned mhz, unsigned mode);
extern void rom_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);

/* ---- LO planning (esp-sdr rx_lo.h, ESP32 aliases) ----
 * Plan B divides the ordinary receive LO by 6/5 for 1842..2209 MHz. */
typedef struct {
    unsigned mhz;
    int offset_khz;
    bool alternate;
} rx_lo_plan_t;
static rx_lo_plan_t rx_lo_plan(unsigned mhz) {
    bool alternate = mhz >= 1842 && mhz < 2210;
    unsigned khz = mhz * (alternate ? 1200u : 1000u);
    return (rx_lo_plan_t){khz / 1000u, (int)(khz % 1000u), alternate};
}
static bool rx_lo_select(bool alternate) {
    unsigned old = ram_chip_i2c_readReg(0x65, 4, 0);
    unsigned value = (old & ~0x10u) | (alternate ? 0x10u : 0u);
    if (value != old) ram_chip_i2c_writeReg(0x65, 4, 0, value);
    return value != old;
}

/* ---- analog filter (esp-sdr rx_bandwidth.h, original ESP32 curve:
 * seven-bit BBTOP 1/2, 2472 MHz, gain 72, IQ10). ---- */
unsigned sdr_rx_bandwidth_dcap(unsigned mhz) {
    static const struct {
        uint8_t dcap, mhz;
    } cal[] = {{8, 67},  {12, 55}, {16, 48}, {24, 38},  {32, 32}, {48, 25},
               {64, 20}, {80, 17}, {96, 15}, {112, 14}, {127, 12}};
    const unsigned count = sizeof(cal) / sizeof(cal[0]);
    if (!mhz) return 0;
    if (mhz >= cal[0].mhz) return cal[0].dcap;
    if (mhz <= cal[count - 1].mhz) return cal[count - 1].dcap;
    for (unsigned i = 1; i < count; i++)
        if (mhz >= cal[i].mhz) {
            unsigned span = cal[i - 1].mhz - cal[i].mhz;
            return cal[i - 1].dcap +
                   ((cal[i].dcap - cal[i - 1].dcap) * (cal[i - 1].mhz - mhz) + span / 2) / span;
        }
    return cal[count - 1].dcap;
}

/* ---- state ---- */
static sdr_stats_t stats;
static unsigned gain_max, gain_code = 40;
static bool hardware_agc = true;
static unsigned want_mhz = 2412; /* requested LO */
static unsigned now_mhz;         /* where the PLL physically is */
static bool rx_ready;            /* receive-only PBUS state is active */
static bool lo_alternate;
static int rx_filter = -1;
static unsigned filter_saved[2];
static int64_t settle_until; /* no capture before this (esp_timer us) */

static void filter_apply(void) {
    for (unsigned j = 0; j < 2; j++) {
        filter_saved[j] = rom_chip_i2c_readReg(0x67, 1, 1 + j);
        if (rx_filter >= 0)
            rom_chip_i2c_writeReg(0x67, 1, 1 + j, (filter_saved[j] & ~127u) | (unsigned)rx_filter);
    }
    esp_rom_delay_us(10);
}
static void filter_restore(void) {
    if (rx_filter >= 0)
        for (unsigned j = 0; j < 2; j++) rom_chip_i2c_writeReg(0x67, 1, 1 + j, filter_saved[j]);
}
static void apply_gain(void) {
    uint32_t gain = REG_READ(RX_GAIN);
    if (hardware_agc)
        REG_CLR_BIT(RX_GAIN, BIT(23));
    else
        REG_WRITE(RX_GAIN, (gain & 0x7fffff) | (gain_code << 24) | BIT(23));
}
/* esp-sdr prepare_rx(): receive-only PBUS state. PBUS work-mode setup
 * releases a forced RX gain, so the gain is applied afterwards. */
static void rx_prepare(void) {
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    apply_gain();
    rx_ready = true;
}

static void tune_now(unsigned mhz) {
    int64_t t0 = esp_timer_get_time();
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    rx_lo_select(false);
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    /* Use the PHY entry point even for repeated requests: the Wi-Fi API can
     * skip a channel it thinks is already selected after a direct PLL retune. */
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) {
        unsigned xtal = rtc_clk_xtal_freq_get();
        /* Channel setup pins its capacitor code. Release it before the ROM
         * retune, otherwise the SDM changes but the PLL can remain unlocked. */
        unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
        ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
        /* This ROM uses 1/1024 MHz offsets, unlike the newer kHz API. */
        int offset = (plan.offset_khz * 1024 + 500) / 1000;
        rom_set_rf_freq_offset(xtal == 26 ? 1 : xtal == 24 ? 2 : 0, plan.mhz, offset);
    }
    now_mhz = mhz;
    /* Channel setup reconfigures the PBUS: re-enter receive-only state after
     * every retune (esp-sdr FREQ: rx_ready = false; prepare_rx();). */
    rx_prepare();
    lo_alternate = plan.alternate;
    /* switching the LO plan needs 3 ms before the first capture (esp-sdr);
     * sdr_rx_capture waits it out, so the caller can compute meanwhile */
    if (rx_lo_select(plan.alternate)) settle_until = esp_timer_get_time() + 3000;
    esp_rom_delay_us(150); /* PLL settle before the first capture */
    stats.retunes++;
    stats.last_retune_us = (uint32_t)(esp_timer_get_time() - t0);
}

void sdr_rx_set_frequency(unsigned mhz) { want_mhz = mhz; }
/* sdr_rx_radio_restore always returns to the ordinary plan */
bool sdr_rx_slow_retune(unsigned mhz) { return rx_lo_plan(mhz).alternate; }
unsigned sdr_rx_frequency(void) { return want_mhz; }

void sdr_rx_burst_begin(void) {
    if (!rx_ready || now_mhz != want_mhz)
        tune_now(want_mhz);
    else
        rx_prepare();
}

void sdr_rx_radio_restore(unsigned home_mhz) {
    REG_WRITE(DUMP_CTRL, 0);
    REG_CLR_BIT(RX_GAIN, BIT(23)); /* the AP's receiver needs its own AGC */
    if (lo_alternate) {
        rx_lo_select(false);
        lo_alternate = false;
    }
    rom_pbus_workmode();
    rom_pbus_xpd_tx_on();
    if (home_mhz && now_mhz != home_mhz) {
        set_chanfreq(home_mhz, 0);
        now_mhz = home_mhz;
    }
    rx_ready = false;
    stats.restores++;
}

/* ---- capture (esp-sdr acquire_iq, serial replies removed) ---- */
int sdr_rx_capture(unsigned n, unsigned clock, const uint32_t **words) {
    if (n < 256 || n > SDR_MAX_SAMPLES || clock > 2 || !rx_ready) return -1;
    while (esp_timer_get_time() < settle_until) {}
    const unsigned guard = n + 64 < CAPACITY ? n + 64 : CAPACITY;
    REG_WRITE(DUMP_CTRL, 0);
    for (unsigned j = 0; j < guard; j++) samples[j] = SENTINEL;
    filter_apply();
    uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
    uint32_t byte_select = REG_READ(DUMP_BYTES);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
                    (owner & ~DPORT_MAC_DUMP_MODE_M) | (3 << DPORT_MAC_DUMP_MODE_S));
    REG_WRITE(DUMP_BYTES, 0x03020100); /* Four distinct bytes, as in mac_init. */
    uint32_t ctrl = n | ((clock == 0) << 16) | ((clock == 2) << 15);
    REG_WRITE(DUMP_CTRL, ctrl | BIT(31));
    int64_t start = esp_timer_get_time();
    REG_SET_BIT(DUMP_CTRL, BIT(19));
    REG_CLR_BIT(DUMP_CTRL, BIT(19));
    while (!(REG_READ(DUMP_CTRL) & BIT(18)) && esp_timer_get_time() - start < 20000) {}
    uint32_t result = REG_READ(DUMP_CTRL), status = REG_READ(DUMP_STATUS);
    unsigned elapsed = (unsigned)(esp_timer_get_time() - start);
    REG_WRITE(DUMP_CTRL, 0);
    esp_rom_delay_us(10);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
    REG_WRITE(DUMP_BYTES, byte_select);
    filter_restore();
    if (!(result & BIT(18))) {
        stats.capture_errors++;
        SDR_TRACE("capture timeout (n=%u ctrl=%08x)", n, result);
        return -2;
    }
    /* Upstream also requires the sample count here; it is kept as a counter
     * so a silicon/PHY difference shows up in the stats instead of failing. */
    if ((status & 0x7fff) != n) stats.count_mismatch++;
    for (unsigned j = 0; j < guard; j++)
        if ((j < n) == (samples[j] == SENTINEL)) {
            stats.capture_errors++;
            return -3;
        }
    /* A few percent of captures come back with every sample railed at one
     * value (seen: I=-488, Q=-511 at all LOs, ~3%): the RX chain was not
     * delivering data. Sixteen spread-out words decide it cheaply. */
    bool varied = false;
    for (unsigned k = 1; k < 16 && !varied; k++)
        varied = ((samples[k * (n - 1) / 15] ^ samples[0]) & 0xfffffu) != 0;
    if (!varied) {
        stats.stuck_captures++;
        return -4;
    }
    stats.captures++;
    stats.last_capture_us = elapsed;
    if (stats.captures == 1)
        SDR_TRACE("first capture: %u us, status=%08x w0=%08x (gain %u)", elapsed, status, samples[0],
                  (samples[0] >> 20) & 0xff);
    *words = samples;
    return (int)elapsed;
}

int sdr_rx_capture_burst(unsigned n, unsigned clock, const uint32_t **words, unsigned home_mhz) {
    int us = -1;
    for (unsigned attempt = 0; attempt < 6; attempt++) {
        sdr_rx_burst_begin();
        us = sdr_rx_capture(n, clock, words);
        sdr_rx_radio_restore(home_mhz);
        if (us != -4) break;
        /* let whatever held the RX chain finish; stuck spells can last
         * several milliseconds, so back off a little more each time */
        vTaskDelay(1 + attempt);
    }
    return us;
}

/* ---- gain / filter ---- */
void sdr_rx_set_gain(bool agc, unsigned code) {
    hardware_agc = agc;
    gain_code = code > gain_max ? gain_max : code;
}
unsigned sdr_rx_gain_max(void) { return gain_max; }
void sdr_rx_set_filter(int code) { rx_filter = code > 127 ? 127 : code; }
int sdr_rx_filter(void) { return rx_filter; }
uint32_t *sdr_rx_buffer(void) { return samples; }
const sdr_stats_t *sdr_rx_stats(void) { return &stats; }

bool sdr_rx_begin(void) {
    memset(&stats, 0, sizeof(stats));
    rx_prepare();
    /* AGCPWR_CTRL7 bits 8..14 hold the maximum calibrated low-table index;
     * never expose uncalibrated slots (esp-sdr burst_gain.h). */
    unsigned maximum = (REG_READ(RX_GAIN) >> 8) & 127;
    gain_max = maximum < 80 ? maximum : 0;
    REG_CLR_BIT(RX_GAIN, BIT(23)); /* Hardware AGC is the default. */
    if (gain_code > gain_max) gain_code = gain_max;
    sdr_rx_radio_restore(0);
    SDR_TRACE("begin: gain_max=%u", gain_max);
    return gain_max > 0;
}
