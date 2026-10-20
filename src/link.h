/* link.h — ESPARGOS/esp-sdr burst protocol (protocol 6, ESP32 profile)
 * served over two byte streams: UART0 (USB bridge, 2 Mbaud, like upstream)
 * and the /ws WebSocket that the bundled esp-web-sdr page uses in place of
 * WebSerial. See esp-sdr docs/protocol.md and docs/spectrum.md.
 *
 * Only one client controls the radio at a time (upstream's serial lease):
 * the first command takes it, the other port gets "ERR busy" until RELEASE
 * or five seconds of silence. While a client holds the lease the local TFT
 * scan pauses and the screen mirrors the client's spectra instead.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { LINK_NONE = 0, LINK_UART = 1, LINK_WS = 2 };

void link_init(void);
/* Engine task: execute at most one complete pending command (CAP and SPEC
 * run inline). Returns true if a command was handled. */
bool link_service(void);
bool link_input_pending(void);
/* Engine task only (mutates lease state); other tasks read link_owner(). */
bool link_lease_active(void);
int link_owner(void); /* LINK_* of the lease holder, LINK_NONE if free */
/* True while a protocol client is using UART0: console logging must stop. */
bool link_uart_quiet(void);

/* httpd task: WebSocket bytes and closes (web_server.cpp). */
void link_ws_input(int fd, const uint8_t *data, size_t len);
void link_ws_closed(int fd);

/* Provided by web_server.cpp. web_ws_send() runs in the engine task and
 * blocks for bounded backpressure; web_ws_send_now() runs in the httpd task. */
bool web_ws_send(int fd, const void *data, size_t len);
void web_ws_send_now(int fd, const char *text);

#ifdef __cplusplus
}
#endif
