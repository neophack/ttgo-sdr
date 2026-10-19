// ws-transport.js — WebSocket stand-in for esp-web-sdr's serial-transport.js.
//
// The TTGO firmware serves the ESP-SDR burst protocol on /ws as a plain byte
// stream in both directions, so radio.js and app.js run unchanged on top of a
// port object with the SerialPort subset they use: open, close, getInfo,
// setSignals and readable/writable byte streams.
// GPL-3.0-or-later, like the esp-web-sdr files it is bundled with.
(function (root) {
  'use strict';
  // Not a USB device. A fixed identity lets app.js remember the "port" and
  // reconnect on page load, exactly as it does for a granted serial port.
  const identity = { usbVendorId: 0x7467, usbProductId: 0x6f01 };
  const PORT_KEY = 'espSdrPort';

  class WebSocketPort {
    constructor(url) {
      this.url = url;
      this.kind = 'websocket';
      this.readable = null;
      this.writable = null;
      this.ws = null;
    }
    getInfo() {
      return { ...identity };
    }
    async setSignals() {
      // No DTR/RTS: the board never resets on connect.
    }
    async open() {
      if (this.ws) throw Error('The TTGO SDR connection is already open.');
      const ws = new WebSocket(this.url);
      ws.binaryType = 'arraybuffer';
      await new Promise((resolve, reject) => {
        ws.onopen = resolve;
        ws.onerror = () => reject(Error(`Cannot reach the TTGO SDR at ${this.url}.`));
        ws.onclose = () => reject(Error(`Cannot reach the TTGO SDR at ${this.url}.`));
      });
      this.ws = ws;
      const encoder = new TextEncoder();
      let controller;
      this.readable = new ReadableStream({
        start: (c) => {
          controller = c;
        },
        cancel: () => this.close(),
      });
      ws.onmessage = (ev) => {
        const bytes = typeof ev.data === 'string' ? encoder.encode(ev.data) : new Uint8Array(ev.data);
        try {
          controller.enqueue(bytes);
        } catch {
          /* stream already closed */
        }
      };
      ws.onerror = null;
      ws.onclose = () => {
        if (this.ws === ws) this.ws = null;
        try {
          controller.close(); // radio.js reports "Device disconnected"
        } catch {}
      };
      this.writable = new WritableStream({
        write: (bytes) => {
          if (ws.readyState !== WebSocket.OPEN) throw Error('The TTGO SDR connection is closed.');
          ws.send(bytes);
        },
      });
    }
    async close() {
      const ws = this.ws;
      this.ws = null;
      this.readable = null;
      this.writable = null;
      if (ws && ws.readyState <= WebSocket.OPEN) ws.close();
    }
  }

  const url =
    root.TTGO_SDR_WS_URL ||
    (root.location ? (root.location.protocol === 'https:' ? 'wss://' : 'ws://') + root.location.host + '/ws' : '');
  const port = new WebSocketPort(url);
  try {
    // First visit: remember the device so the viewer connects on page load.
    if (!root.localStorage?.getItem(PORT_KEY))
      root.localStorage?.setItem(
        PORT_KEY,
        JSON.stringify({ vid: identity.usbVendorId, pid: identity.usbProductId, kind: 'websocket' }),
      );
  } catch {}

  root.SerialConnection = {
    WebSocketPort,
    // A WebSocket on the device's own access point does not need HTTPS.
    insecureOk: true,
    supported: () => true,
    defaultKind: () => 'websocket',
    kind: (p) => p.kind || 'webserial',
    requestPort: async () => port,
    getPorts: async () => [port],
    reconnect: async (p) => p,
    onConnect() {},
  };
})(globalThis);
