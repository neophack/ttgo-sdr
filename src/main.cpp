/* main.cpp — TTGO T-Display ESP32 SDR (spectrum monitor) on ESPARGOS/esp-sdr.
 *
 *   screen: power spectrum / waterfall / split, GPIO0 + GPIO35 buttons
 *   WiFi:   SoftAP "TTGO-SDR" (pass "ttgosdr123"), http://192.168.4.1 serves
 *           the complete esp-web-sdr viewer over a WebSocket transport
 *   USB:    the same esp-sdr protocol on UART0 at 2 Mbaud, so the hosted
 *           viewer (espargos.net/espsdr/app, WebSerial) works too
 *
 * Boot order matters: the WiFi stack must bring the PHY up (SoftAP on
 * channel 1) before the capture backend borrows the receiver.
 */
#include <Arduino.h>
#include <WiFi.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "engine.h"
#include "link.h"
#include "sdr_rx.h"
#include "ui_tft.h"
#include "web_server.h"

SET_LOOP_TASK_STACK_SIZE(6 * 1024);

#define AP_SSID    "TTGO-SDR"
#define AP_PASS    "ttgosdr123"
#define AP_CHANNEL 1 /* AP_HOME_MHZ (engine.h) = 2412 */

/* Console lines share UART0 with the binary protocol: print only while no
 * protocol client is talking to us. */
static void log_line(const char *fmt, ...) {
    if (link_uart_quiet()) return;
    char text[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    Serial.print(text);
}

void setup() {
    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(4096);
    Serial.begin(2000000);
    esp_log_level_set("*", ESP_LOG_NONE);
    ui_init();
    ui_boot_line("WiFi access point");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_promiscuous(true);
    log_line("\n[boot] AP %s at %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

    ui_boot_line("RF capture backend");
    if (!sdr_rx_begin()) log_line("[boot] WARNING: no calibrated gain table\n");
    engine_start();
    log_line("[boot] engine up, gain_max=%u heap=%u\n", sdr_rx_gain_max(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));

    ui_boot_line(("Web  http://" + WiFi.softAPIP().toString()).c_str());
    if (!web_start()) log_line("[boot] WARNING: httpd failed\n");
    else log_line("[boot] http://%s\n", WiFi.softAPIP().toString().c_str());
    delay(600); /* let the splash show */
}

void loop() {
    static uint32_t last_seq, last_render, last_log;
    static ui_frame_t frame;
    uint32_t now = millis();
    uint32_t seq = engine_latest(&frame);
    bool fresh = seq && seq != last_seq;
    if (fresh) {
        ui_push(frame);
        last_seq = seq;
    }
    /* every new frame (a render takes ~20 ms); at least 20 fps for toasts */
    if (fresh || now - last_render >= 50) {
        ui_render();
        last_render = now;
    }
    ui_poll(now);
    if (now - last_log >= 15000) {
        const sdr_stats_t *st = sdr_rx_stats();
        log_line("[stat] frames=%lu caps=%lu err=%lu retune=%luus heap=%u min=%u owner=%d\n",
                 (unsigned long)seq, (unsigned long)st->captures, (unsigned long)st->capture_errors,
                 (unsigned long)st->last_retune_us, (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), link_owner());
        last_log = now;
    }
    delay(1);
}
