/* ui_tft.h — TTGO T-Display (ST7789 240x135 landscape) power spectrum and
 * waterfall views with button control. Runs in the Arduino loop task. */
#pragma once
#include "engine.h"

void ui_init();
void ui_boot_line(const char *text); /* splash progress */
void ui_push(const ui_frame_t &frame); /* feed every new frame (waterfall rows) */
void ui_render();                      /* compose and push one screen */
void ui_poll(uint32_t now_ms);         /* buttons */

/* Set by "TTGO SHOT?": the next ui_render() also streams the screen over
 * UART0 ("TTGO SHOT 240 135" + big-endian RGB565 rows), then clears it. */
extern volatile bool ui_shot_request;

/* Mean per-frame change of the displayed trace (dB) since the last call. */
float ui_take_jitter(void);
/* average ui_render() time in ms since the last call (resets); *count = renders */
float ui_take_render(unsigned *count);
void ui_set_averaging(unsigned level); /* 0 off, 1 0.5 s, 2 1.5 s, 3 4 s */
void ui_set_view(unsigned view);        /* 0 spectrum, 1 waterfall, 2 split */
