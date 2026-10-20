/* sdr_rx.h — ESP32 RF capture backend for the TTGO T-Display SDR.
 * Ported from ESPARGOS/esp-sdr (GPL-3.0-or-later), main/targets/esp32/receiver.c.
 *
 * The WiFi peripheral's undocumented "MAC dump" debug path copies raw receiver
 * I/Q words into a reserved SRAM bank (16380 words max). Each word holds
 *   bits  9.. 0  I, 10-bit two's complement
 *   bits 19..10  Q, 10-bit two's complement
 *   bits 27..20  receiver gain index active for that sample
 * esp-sdr owns the radio forever (WiFi NULL mode). This port shares it with a
 * SoftAP: every capture burst disables TX, and sdr_rx_radio_restore() hands
 * the radio back to the access point (TX on, home channel, AGC released).
 * All functions must be called from one task (the engine).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dump clock selectors (hardware sampling, not decimation). */
#define SDR_CLK_16M 0
#define SDR_CLK_80M 1
#define SDR_CLK_40M 2

#define SDR_MAX_SAMPLES 16380

bool sdr_rx_begin(void);

/* Requested receive LO in MHz; applied by the next sdr_rx_burst_begin(). */
void sdr_rx_set_frequency(unsigned mhz);
unsigned sdr_rx_frequency(void);

/* Gain and analog filter, applied by the next sdr_rx_burst_begin(). */
void sdr_rx_set_gain(bool hardware_agc, unsigned code);
unsigned sdr_rx_gain_max(void);
void sdr_rx_set_filter(int dcap); /* -1 = PHY default, 0..127 capacitor DAC code */
int sdr_rx_filter(void);
/* esp-sdr's measured ESP32 bandwidth curve: MHz (0 = widest) -> DAC code. */
unsigned sdr_rx_bandwidth_dcap(unsigned mhz);
#define SDR_BANDWIDTH_MIN 12u
#define SDR_BANDWIDTH_MAX 67u

/* Take the radio: tune if needed, TX off, RX on, gain applied (a no-op
 * while the radio is already held at the requested LO). */
void sdr_rx_burst_begin(void);
/* True when tuning to mhz after sdr_rx_radio_restore() switches the LO plan:
 * sdr_rx_burst_begin() then returns while the PLL still needs ~3 ms, which
 * the next capture waits out (the previous capture stays readable until then). */
bool sdr_rx_slow_retune(unsigned mhz);
/* Capture n words at a dump clock into the reserved bank. *words stays valid
 * (CPU-owned) until the next capture. Returns capture time in us, <0 on error
 * (-4: samples stuck at one value, worth retrying). */
int sdr_rx_capture(unsigned n, unsigned clock, const uint32_t **words);
/* sdr_rx_burst_begin + capture + radio_restore, retrying a stuck capture. */
int sdr_rx_capture_burst(unsigned n, unsigned clock, const uint32_t **words, unsigned home_mhz);
/* burst_begin + capture without handing the radio back, for sweeps that
 * keep the receiver across several LOs (two PHY channel switches less per
 * capture, and a run of alternate-plan LOs pays the 3 ms plan switch once);
 * the sweep's last capture uses sdr_rx_capture_burst. A stuck capture falls
 * back to capture_burst's retries (home_mhz). */
int sdr_rx_capture_held(unsigned n, unsigned clock, const uint32_t **words, unsigned home_mhz);
/* True between sdr_rx_burst_begin() and sdr_rx_radio_restore(). */
bool sdr_rx_holding(void);
/* Give the radio back to WiFi: TX on, AGC released, tuned to home_mhz. */
void sdr_rx_radio_restore(unsigned home_mhz);

/* The capture bank, for in-place packing of a finished capture. */
uint32_t *sdr_rx_buffer(void);

typedef struct {
    uint32_t captures;
    uint32_t capture_errors;
    uint32_t count_mismatch; /* DUMP_STATUS sample count != n (diagnostic) */
    uint32_t stuck_captures; /* every sample identical: RX chain not delivering */
    uint32_t retunes;
    uint32_t restores;
    uint32_t last_capture_us;
    uint32_t last_retune_us;
} sdr_stats_t;
const sdr_stats_t *sdr_rx_stats(void);

/* Suppress backend console traces (UART0 carries the binary protocol). */
extern volatile bool sdr_log_quiet;

#ifdef __cplusplus
}
#endif
