// Drive the bundled esp-web-sdr radio.js + ws-transport.js headless (Node >= 22,
// global WebSocket) against tools/ws_bridge.py, i.e. the same JavaScript the
// browser runs from http://192.168.4.1, talking to the real board.
//
//   python tools/build_web.py --dist build/web
//   python tools/ws_bridge.py COM7 build/web &
//   node tools/web_radio_test.mjs build/web ws://127.0.0.1:8765/ws
import { readFileSync } from 'node:fs';
import vm from 'node:vm';

const [dist = 'build/web', url = 'ws://127.0.0.1:8765/ws'] = process.argv.slice(2);
globalThis.TTGO_SDR_WS_URL = url;
globalThis.isSecureContext = false; // plain http on the device's access point
for (const file of ['ws-transport.js', 'radio.js'])
  vm.runInThisContext(readFileSync(`${dist}/${file}`, 'utf8'), { filename: file });
const radio = vm.runInThisContext('radio');

let failed = 0;
const check = (ok, what) => {
  console.log(`  ${ok ? 'ok  ' : 'FAIL'} ${what}`);
  if (!ok) failed++;
};

const port = await SerialConnection.requestPort();
const info = await radio.connect({ port });
check(info.family === 'ESP32' && radio.maxSamples === 16380, `connect: ${info.identity}`);
// The device answers TRANSPORT USB 0 on its WebSocket, TRANSPORT UART 2000000
// through ws_bridge.py (which uses the USB serial port).
check(['USB', 'UART'].includes(radio.transport) && radio.canStreamSpectrum,
  `transport ${radio.transport}, on-chip spectra available`);
check(radio.gainMax > 0 && radio.hasHardwareAgc, `gain 0..${radio.gainMax}, hardware AGC`);
check(radio.firmwareVersion?.status === 'known', `firmware version ${radio.firmwareVersion?.build_timestamp}`);
check(radio.hasExtendedTune && radio.validFrequency(2350), `tuning range ${radio.tuneRange}`);

for (const bits of [10, 8]) {
  const f = await radio.capture({ frequency: 2437, rate: 80000000, bits, fft: 2048, bandwidth: 0,
    gainMode: 'MANUAL', gain: 40 });
  check(f.crc_ok && f.samples === 16380 && f.spectrum.length === 2048,
    `I/Q capture ${bits}-bit: ${f.samples} samples, ${f.elapsed_ms.toFixed(0)} ms, peak ${(f.peak_hz / 1e6).toFixed(2)} MHz`);
}

for (const [rate, fft] of [[80000000, 2048], [40000000, 1024], [16000000, 256]]) {
  let frames = 0, ffts = 0;
  const t0 = performance.now();
  const end = await radio.spec(
    { frequency: 2442, rate, fft, bandwidth: 20, gainMode: 'HARDWARE', gain: 40, maxHold: false },
    (h, bins) => {
      frames++;
      ffts += h.ffts;
      if (bins.length !== fft) failed++;
    },
    () => performance.now() - t0 > 2000,
  );
  check(frames > 10 && !end.crcErrors && end.report?.startsWith('SPECEND 0'),
    `SPEC ${fft} bins @ ${rate / 1e6} MS/s: ${(frames / 2).toFixed(0)} frames/s, ${ffts / frames} FFT/frame, ` +
      `CRC errors ${end.crcErrors}, stats ${end.stats ? 'yes' : 'no'}`);
}
await radio.run(() => radio.close());
check(!radio.port, 'disconnect (LPF AUTO + RELEASE)');
console.log(failed ? `${failed} FAILED` : 'all passed');
process.exit(failed ? 1 : 0);
