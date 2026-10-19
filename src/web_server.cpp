/* web_server.cpp — HTTP + WebSocket front end.
 *
 *   GET /, /app.js, ...  esp-web-sdr, gzipped into web_assets.h by
 *                        tools/build_web.py (serial transport swapped for
 *                        ws-transport.js)
 *   WS  /ws              the esp-sdr byte stream, in both directions
 *   GET /api/status      JSON diagnostics
 *
 * Outgoing WebSocket data is queued with httpd_ws_send_data_async() from the
 * engine task; each chunk is freed by its completion callback, and at most
 * WS_INFLIGHT bytes are queued so a slow browser throttles the radio instead
 * of exhausting the heap.
 */
#include "web_server.h"
#include <Arduino.h>
#include <atomic>
#include <unistd.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "engine.h"
#include "link.h"
#include "sdr_rx.h"
#include "web_assets.h"

#define WS_CHUNK    2048
#define WS_INFLIGHT 8192 /* ~25 KB of heap is free; lwIP needs its 5.7 KB send buffer too */

static httpd_handle_t server;
static std::atomic<int> inflight{0};

struct chunk_t {
    size_t len;
    uint8_t data[];
};
static void chunk_sent(esp_err_t err, int socket, void *arg) {
    (void)err;
    (void)socket;
    chunk_t *c = (chunk_t *)arg;
    inflight -= (int)c->len;
    free(c);
}

extern "C" bool web_ws_send(int fd, const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    while (len) {
        size_t n = len < WS_CHUNK ? len : WS_CHUNK;
        int64_t deadline = esp_timer_get_time() + 3000000;
        while (inflight.load() + (int)n > WS_INFLIGHT) {
            if (esp_timer_get_time() > deadline) return false;
            vTaskDelay(1);
        }
        chunk_t *c = (chunk_t *)malloc(sizeof(chunk_t) + n);
        if (!c) return false;
        c->len = n;
        memcpy(c->data, p, n);
        httpd_ws_frame_t frame = {};
        frame.final = true;
        frame.type = HTTPD_WS_TYPE_BINARY;
        frame.payload = c->data;
        frame.len = n;
        inflight += (int)n;
        if (httpd_ws_send_data_async(server, fd, &frame, chunk_sent, c) != ESP_OK) {
            inflight -= (int)n;
            free(c);
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

extern "C" void web_ws_send_now(int fd, const char *text) {
    httpd_ws_frame_t frame = {};
    frame.final = true;
    frame.type = HTTPD_WS_TYPE_BINARY;
    frame.payload = (uint8_t *)text;
    frame.len = strlen(text);
    httpd_ws_send_frame_async(server, fd, &frame);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) return ESP_OK; /* handshake done */
    httpd_ws_frame_t pkt = {};
    if (httpd_ws_recv_frame(req, &pkt, 0) != ESP_OK) return ESP_FAIL;
    if (!pkt.len) return ESP_OK;
    if (pkt.len > 1024) return ESP_FAIL; /* commands are short lines */
    uint8_t buf[1024];
    pkt.payload = buf;
    if (httpd_ws_recv_frame(req, &pkt, pkt.len) != ESP_OK) return ESP_FAIL;
    if (pkt.type == HTTPD_WS_TYPE_TEXT || pkt.type == HTTPD_WS_TYPE_BINARY)
        link_ws_input(httpd_req_to_sockfd(req), buf, pkt.len);
    return ESP_OK;
}

static void on_close(httpd_handle_t hd, int fd) {
    (void)hd;
    link_ws_closed(fd);
    close(fd);
}

static esp_err_t asset_handler(httpd_req_t *req) {
    char path[64];
    size_t len = strcspn(req->uri, "?#");
    if (len >= sizeof(path)) len = sizeof(path) - 1;
    memcpy(path, req->uri, len);
    path[len] = 0;
    if (!strcmp(path, "/")) strcpy(path, "/index.html");
    for (size_t i = 0; i < web_asset_count; i++) {
        const web_asset_t *a = &web_assets[i];
        if (strcmp(a->path, path)) continue;
        httpd_resp_set_type(req, a->type);
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        return httpd_resp_send(req, (const char *)a->data, a->len);
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    local_cfg_t cfg;
    engine_get_local(&cfg);
    const sdr_stats_t *st = sdr_rx_stats();
    char json[384];
    snprintf(json, sizeof(json),
             "{\"owner\":%d,\"band\":%u,\"agc\":%u,\"gain\":%u,\"gainMax\":%u,\"captures\":%lu,"
             "\"errors\":%lu,\"retunes\":%lu,\"retuneUs\":%lu,\"heap\":%lu,\"minHeap\":%lu}",
             link_owner(), cfg.band, cfg.agc, cfg.gain, sdr_rx_gain_max(), (unsigned long)st->captures,
             (unsigned long)st->capture_errors, (unsigned long)st->retunes,
             (unsigned long)st->last_retune_us, (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size());
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, strlen(json));
}

bool web_start() {
    if (server) return true;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 4608;
    cfg.close_fn = on_close;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    esp_err_t err = httpd_start(&server, &cfg);
    if (err != ESP_OK) {
        if (!link_uart_quiet())
            Serial.printf("[web] httpd_start: %s, heap %u, largest %u\n", esp_err_to_name(err),
                          (unsigned)esp_get_free_heap_size(),
                          (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        server = nullptr;
        return false;
    }
    static const httpd_uri_t ws = {.uri = "/ws",
                                   .method = HTTP_GET,
                                   .handler = ws_handler,
                                   .user_ctx = nullptr,
                                   .is_websocket = true,
                                   .handle_ws_control_frames = false,
                                   .supported_subprotocol = nullptr};
    static const httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};
    static const httpd_uri_t assets = {.uri = "/*", .method = HTTP_GET, .handler = asset_handler};
    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &status);
    httpd_register_uri_handler(server, &assets);
    return true;
}
