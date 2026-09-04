// Dog Vitals — live IMU plotter and reviewer.
//
// The processing is deliberately re-run over the whole visible window on every
// frame rather than carried incrementally. It costs a few milliseconds at
// 208 Hz and buys two things that matter more than the milliseconds: changing
// a filter corner instantly re-renders the past as well as the future, and the
// live path and the review path are the same code, so a recording analysed
// later cannot disagree with what was on screen at the time.

import { VitalsLink } from './lib/ble.js';
import { BridgeLink, bridgeAvailable } from './lib/bridge.js';
import {
  ODR_CHOICES, ACCEL_CHOICES, GYRO_CHOICES, batteryPercent,
} from './lib/protocol.js';
import * as dsp from './lib/dsp.js';
import { StripChart, SpectrumChart } from './lib/plot.js';
import {
  SampleBuffer, Recorder, saveSession, listSessions, loadSession,
  deleteSession, sessionToCsv, sessionToJson, csvToSession, sessionToArrays,
} from './lib/store.js';

const $ = (id) => document.getElementById(id);

// Bands are starting guesses from the physiology, not settled values -- which
// is the whole reason the spectrum view and the editable fields exist.
const PRESETS = {
  human:   { resp: [0.10, 0.60], card: [4, 25], hr: [40, 150], label: 'Human — neck' },
  dogRest: { resp: [0.15, 0.90], card: [4, 30], hr: [50, 180], label: 'Dog — resting' },
  // A panting dog breathes at up to 5 Hz, which overlaps the bottom of the
  // cardiac band -- so the cardiac high-pass moves up to keep them apart.
  dogPant: { resp: [1.00, 5.00], card: [8, 30], hr: [60, 200], label: 'Dog — panting' },
};

const state = {
  // Replaced at boot by a BridgeLink when this page is served by
  // tools/server.py. Both classes present the same surface, so nothing below
  // this line knows which transport it has.
  link: new VitalsLink(),
  transport: 'webbt',
  recStartedAt: null,
  buf: new SampleBuffer(),
  rec: new Recorder(),
  fs: 208,
  scales: { accelScale: 0.000061, gyroScale: 0.00875 },
  mode: 'live',
  review: null,        // { arrays, session }
  lastDraw: 0,
  dirty: false,
};

// ------------------------------------------------------------------- charts

const charts = {
  resp:  new StripChart($('cvResp'),  { yLabel: 'band-passed', unit: '' }),
  card:  new StripChart($('cvCard'),  { yLabel: 'band-passed', unit: '' }),
  accel: new StripChart($('cvAccel'), { yLabel: 'g', unit: ' g' }),
  gyro:  new StripChart($('cvGyro'),  { yLabel: 'deg/s', unit: ' °/s' }),
  spec:  new SpectrumChart($('cvSpec')),
};

function seriesColor(n) {
  return getComputedStyle(document.body).getPropertyValue(`--series-${n}`).trim();
}

// ------------------------------------------------------------------ helpers

function bands() {
  return {
    resp: [parseFloat($('respLo').value), parseFloat($('respHi').value)],
    card: [parseFloat($('cardLo').value), parseFloat($('cardHi').value)],
    hr:   [parseFloat($('hrLo').value), parseFloat($('hrHi').value)],
  };
}

function applyPreset(key) {
  const p = PRESETS[key];
  if (!p) return;
  $('respLo').value = p.resp[0];
  $('respHi').value = p.resp[1];
  $('cardLo').value = p.card[0];
  $('cardHi').value = p.card[1];
  $('hrLo').value = p.hr[0];
  $('hrHi').value = p.hr[1];
  state.dirty = true;
}

/** Resolves a source selector to a single channel from the window arrays. */
function pickSource(w, key) {
  switch (key) {
    case 'accelPca': return dsp.principalProjection(w.ax, w.ay, w.az);
    case 'gyroPca':  return dsp.principalProjection(w.gx, w.gy, w.gz);
    case 'accelX':   return w.ax;
    case 'accelY':   return w.ay;
    case 'accelZ':   return w.az;
    case 'gyroX':    return w.gx;
    case 'gyroY':    return w.gy;
    case 'gyroZ':    return w.gz;
    default:         return dsp.principalProjection(w.ax, w.ay, w.az);
  }
}

function fmt(v, digits = 1) {
  return (v === null || v === undefined || !isFinite(v)) ? '—' : v.toFixed(digits);
}

function setBanner(msg, show = true) {
  const el = $('banner');
  el.textContent = msg;
  el.classList.toggle('hidden', !show || !msg);
}

// ------------------------------------------------------------------ analysis

/**
 * Runs the full pipeline over one window and updates every view.
 *
 * Both rates come from autocorrelation rather than from counting the marks
 * that get drawn. The marks are there to be looked at -- to see whether the
 * thing being counted looks like a breath or a beat at all -- but a single
 * missed or doubled peak moves a count by a large fraction, while
 * autocorrelation weighs every cycle in the window and reports how periodic
 * the window actually was. When those two disagree, the confidence bar is the
 * one to believe.
 */
function analyse(w, fs) {
  const b = bands();
  const out = {};

  // --- breathing --------------------------------------------------------
  const respSrc = pickSource(w, $('selRespSrc').value);
  const [rLo, rHi] = b.resp;
  const resp = dsp.respirationBand(respSrc, fs, {
    slowSec: 1 / Math.max(0.02, rLo),
    fastSec: 1 / Math.max(0.5, rHi * 2.5),
  });

  // Autocorrelation over a 2500-lag search at 208 Hz is wasted work for a
  // signal that is band-limited to a couple of hertz; decimating first keeps
  // it comfortably inside a frame.
  const decim = Math.max(1, Math.floor(fs / Math.max(4, rHi * 8)));
  const respD = dsp.decimate(resp, decim);
  const respRate = dsp.autocorrRate(respD, fs / decim, rLo, rHi);
  const respPeaks = dsp.findPeaks(resp, fs, {
    minSepSec: 1 / rHi * 0.7,
    threshRms: 0.55,
  });
  out.resp = { signal: resp, rate: respRate, peaks: respPeaks };

  // --- cardiac ----------------------------------------------------------
  const cardSrc = pickSource(w, $('selCardSrc').value);
  const [cLo, cHi] = b.card;
  const card = dsp.bandpass(cardSrc, fs, cLo, Math.min(cHi, fs / 2 - 1));

  // The heartbeat is a burst, not a tone: the band-passed trace oscillates at
  // the mechanical resonance, and it is the *bursts* that repeat once per
  // beat. So the rate is taken from the envelope, with its DC removed.
  const env = dsp.envelope(card, fs, Math.max(3, b.hr[1] / 60 * 3));
  const envAc = dsp.highpass(env, fs, Math.max(0.3, b.hr[0] / 60 * 0.5));
  const hrRate = dsp.autocorrRate(envAc, fs, b.hr[0] / 60, b.hr[1] / 60);
  const hrPeaks = dsp.findPeaks(envAc, fs, {
    minSepSec: 60 / b.hr[1] * 0.8,
    threshRms: 0.7,
  });
  out.card = {
    signal: card, env: envAc, rate: hrRate, peaks: hrPeaks,
    stats: dsp.intervalStats(hrPeaks, fs),
  };

  // --- spectrum ---------------------------------------------------------
  const specSrc = $('selCardSrc').value;
  const specSignal = dsp.highpass(pickSource(w, specSrc), fs, Math.max(0.05, rLo * 0.5));
  out.spec = dsp.spectrum(specSignal, fs, Math.min(40, fs / 2));
  out.specSrcLabel = $('selCardSrc').selectedOptions[0]?.textContent ?? specSrc;

  return out;
}

function render(w, fs, tStart, fills) {
  const a = analyse(w, fs);
  const c1 = seriesColor(1), c2 = seriesColor(2), c3 = seriesColor(3);

  charts.resp.setData({
    series: [{ name: 'resp', data: a.resp.signal, color: c1 }],
    fs, tStart, markers: a.resp.peaks,
  });
  charts.card.setData({
    series: [
      { name: 'card', data: a.card.signal, color: c1 },
      { name: 'env',  data: a.card.env,    color: c2 },
    ],
    fs, tStart, markers: a.card.peaks,
  });
  charts.accel.setData({
    series: [
      { name: 'X', data: w.ax, color: c1 },
      { name: 'Y', data: w.ay, color: c2 },
      { name: 'Z', data: w.az, color: c3 },
    ], fs, tStart,
  });
  charts.gyro.setData({
    series: [
      { name: 'X', data: w.gx, color: c1 },
      { name: 'Y', data: w.gy, color: c2 },
      { name: 'Z', data: w.gz, color: c3 },
    ], fs, tStart,
  });

  const b = bands();
  const cs = getComputedStyle(document.body);
  charts.spec.setData({
    freqs: a.spec.freqs, mags: a.spec.mags, color: c1,
    bands: [
      { lo: b.resp[0], hi: b.resp[1], label: 'breathing', fill: cs.getPropertyValue('--band').trim() },
      { lo: b.card[0], hi: b.card[1], label: 'cardiac',   fill: cs.getPropertyValue('--band-2').trim() },
    ],
  });

  for (const k of Object.keys(charts)) charts[k].draw();

  // --- readouts ---------------------------------------------------------
  const rr = a.resp.rate;
  // An estimate that fails the gate is shown greyed with its reason rather
  // than hidden: knowing the device is not measuring is itself information,
  // and silently showing nothing looks the same as a flat-lining dog.
  const rGate = dsp.gateRate(rr.bpm, {
    lo: bands().resp[0], hi: bands().resp[1], confidence: rr.confidence,
  });
  $('valResp').style.opacity = rGate.ok ? '1' : '0.45';
  $('valResp').textContent = rr.bpm ? fmt(rr.bpm, 1) : '—';
  $('confResp').style.width = `${Math.round(rr.confidence * 100)}%`;
  $('subResp').textContent = !rr.bpm
    ? 'no periodicity found in band'
    : rGate.ok
      ? `${fmt(rr.hz, 3)} Hz · ${a.resp.peaks.length} peaks in window · confidence ${fmt(rr.confidence * 100, 0)}%`
      : `not trusted — ${rGate.reason} · confidence ${fmt(rr.confidence * 100, 0)}%`;

  const hr = a.card.rate;
  $('valHr').textContent = hr.bpm ? fmt(hr.bpm, 1) : '—';
  $('confHr').style.width = `${Math.round(hr.confidence * 100)}%`;
  const st = a.card.stats;
  $('subHr').textContent = hr.bpm
    ? `peak-count ${fmt(st.meanBpm, 1)} bpm · interval SD ${fmt(st.sdMs, 0)} ms · confidence ${fmt(hr.confidence * 100, 0)}%`
    : 'no periodicity found in band';

  $('noteResp').textContent = `${fmt(bands().resp[0], 2)}–${fmt(bands().resp[1], 2)} Hz · ${w.n} samples`;
  $('noteCard').textContent = `${fmt(bands().card[0], 1)}–${fmt(bands().card[1], 1)} Hz`;
  $('noteSpecSrc').textContent = a.specSrcLabel;

  // Interpolated samples are shown, not hidden: a rate computed across
  // invented data is not the same measurement as one computed across real
  // data, and the difference should be visible while deciding what to trust.
  const filled = fills ?? 0;
  const quality = hr.confidence > 0.45 ? 'good' : hr.confidence > 0.22 ? 'weak' : 'poor';
  $('valSig').textContent = `cardiac ${quality}`;
  $('subSig').textContent =
    `${fmt(fs, 2)} Hz · window ${fmt(w.n / fs, 1)} s` +
    (filled ? ` · ${filled} interpolated samples` : ' · no gaps');
}

// --------------------------------------------------------------- live loop

function tick() {
  window.dv = state;      // for poking at from the console while it streams
requestAnimationFrame(tick);
  const now = performance.now();
  if (now - state.lastDraw < 100) return;    // 10 fps is plenty for these rates
  state.lastDraw = now;

  if (state.mode === 'review') {
    if (state.dirty && state.review) { state.dirty = false; drawReview(); }
    return;
  }

  const fs = state.link.measuredOdr || state.fs;
  const secs = parseFloat($('selWindow').value);
  if (state.buf.len < fs * 2) return;

  const w = state.buf.window(secs, fs);
  const fillsInWindow = state.buf.fills
    .filter(([s, e]) => e > w.offset)
    .reduce((acc, [s, e]) => acc + (e - Math.max(s, w.offset)), 0);
  render(w, fs, 0, fillsInWindow);
  updateStats();
}

function updateStats() {
  const l = state.link;
  if (state.rec.recording && state.recStartedAt) {
    const sec = Math.floor((Date.now() - state.recStartedAt) / 1000);
    const t = $('txtRecTime');
    if (t) t.textContent = `${Math.floor(sec / 60)}:${String(sec % 60).padStart(2, '0')}`;
    const n = $('txtRecN2');
    if (n) n.textContent = state.rec.sampleCount.toLocaleString();
  }
  $('txtRate').textContent = l.connected ? `${l.hostRate.toFixed(0)} Hz` : '—';
  $('txtLost').textContent = l.connected ? `${l.lost}` : '—';
  $('txtRecN').textContent = state.rec.sampleCount.toLocaleString();
}

// ----------------------------------------------------------------- review

function drawReview() {
  const { arrays, session } = state.review;
  const fs = session.measuredOdr || session.odrHz || 208;
  if (!arrays.n) return;
  const secs = parseFloat($('selWindow').value);
  const want = Math.min(arrays.n, Math.round(secs * fs));
  const off = Math.max(0, arrays.n - want);
  const w = {
    n: want, offset: off,
    ax: arrays.ax.subarray(off), ay: arrays.ay.subarray(off), az: arrays.az.subarray(off),
    gx: arrays.gx.subarray(off), gy: arrays.gy.subarray(off), gz: arrays.gz.subarray(off),
  };
  const filled = (arrays.fills || [])
    .filter(([s, e]) => e > off)
    .reduce((acc, [s, e]) => acc + (e - Math.max(s, off)), 0);
  render(w, fs, off / fs, filled);
}

// ------------------------------------------------------------------- device

function fillSelect(sel, values, suffix) {
  sel.innerHTML = '';
  values.forEach((v, i) => {
    const o = document.createElement('option');
    o.value = i;
    o.textContent = `${v}${suffix}`;
    sel.appendChild(o);
  });
}

function renderDeviceTable(s) {
  const rows = s ? [
    ['IMU', s.imuOk ? 'ok' : 'FAILED'],
    ['ODR (nominal)', `${s.odrHz} Hz`],
    ['ODR (measured)', state.link.measuredOdr ? `${state.link.measuredOdr.toFixed(2)} Hz` : '—'],
    ['Accel range', `±${s.accelFsG} g`],
    ['Gyro range', `±${s.gyroFsDps} °/s`],
    ['ATT MTU', `${s.mtu} B`],
    ['Battery', `${s.battMv} mV (${batteryPercent(s.battMv)}%)`],
    ['Uptime', `${(s.uptimeMs / 1000).toFixed(0)} s`],
    ['FIFO overruns', `${s.overruns}`],
    ['Notify drops', `${s.txDrops}`],
    ['Packets / samples', `${state.link.packets} / ${state.link.samples}`],
    ['Index gaps', `${state.link.gaps} (${state.link.lost} samples)`],
  ] : [['status', 'not connected']];

  $('tblDevice').querySelector('tbody').innerHTML = rows
    .map(([k, v]) => `<tr><td>${k}</td><td class="num">${v}</td></tr>`)
    .join('');
}

function onStatus(s) {
  state.fs = s.odrHz;
  state.scales = { accelScale: s.accelScale, gyroScale: s.gyroScale };
  $('txtBatt').textContent = `${batteryPercent(s.battMv)}%`;
  $('selOdr').value = String(Math.max(0, ODR_CHOICES.indexOf(s.odrHz)));
  $('selAccelFs').value = String(Math.max(0, ACCEL_CHOICES.indexOf(s.accelFsG)));
  $('selGyroFs').value = String(Math.max(0, GYRO_CHOICES.indexOf(s.gyroFsDps)));
  renderDeviceTable(s);

  if (!s.imuOk) {
    setBanner('The board reports its IMU did not initialise. Check the serial console '
            + "with 'i' for the bus diagnostic — the IMU supply pin needs high drive.");
  } else {
    setBanner('', false);
  }
}

function setLinkUi(connected) {
  $('btnConnect').disabled = connected;
  $('btnDisconnect').disabled = !connected;
  $('dotLink').className = `dot ${connected ? 'ok' : ''}`;
  $('txtLink').textContent = connected
    ? (state.transport === 'bridge' ? 'streaming via server' : 'streaming')
    : (state.transport === 'bridge' ? 'server has no board' : 'not connected');
  if (!connected) {
    $('txtBatt').textContent = '—';
    $('txtRate').textContent = '—';
  }
}

// -------------------------------------------------------------------- wiring

function bindLink(link) {
  link.addEventListener('packet', (e) => {
    const p = e.detail;
    state.buf.add(p, state.scales, link.measuredOdr || state.fs);
    state.rec.add(p);
  });
  link.addEventListener('gap', (e) => state.rec.noteGap(e.detail));
  link.addEventListener('status', (e) => onStatus(e.detail));
  link.addEventListener('connected', () => { setLinkUi(true); setBanner('', false); });
  link.addEventListener('linkdown', () => {
    setLinkUi(false);
    setBanner('The server lost the board. It retries on its own — nothing to do here.');
  });
  link.addEventListener('disconnected', () => {
    setLinkUi(false);
    if (state.transport === 'webbt') {
      setBanner('Link dropped. The board keeps advertising — press Connect to resume.');
    }
  });
  link.addEventListener('serverstate', (e) => {
    const st = e.detail;
    $('txtLost').textContent = String(st.lost ?? 0);
    const el = $('txtServerRec');
    if (el) {
      el.textContent = st.recording
        ? `${(st.recordPath || '').split('/').pop()} · ${st.recordSamples.toLocaleString()}`
        : 'off';
    }
    if (st.error && !st.connected) setBanner(`Bridge: ${st.error}`);
  });
}
bindLink(state.link);

$('btnConnect').addEventListener('click', async () => {
  if (state.transport === 'webbt' && !navigator.bluetooth) {
    setBanner('This browser has no Web Bluetooth. Either use Chrome/Edge, or run '
            + 'tools/server.py, which does the Bluetooth itself and needs nothing '
            + 'from the browser.');
    return;
  }
  try {
    setBanner('', false);
    state.buf.clear();
    await state.link.connect();
    setLinkUi(state.link.connected);
  } catch (err) {
    if (err?.name !== 'NotFoundError') {      // user simply dismissed the picker
      setBanner(`Connect failed: ${err.message}`);
    }
  }
});

$('btnDisconnect').addEventListener('click', async () => {
  await state.link.disconnect();
  setLinkUi(false);
});

$('selPreset').addEventListener('change', (e) => applyPreset(e.target.value));
for (const id of ['respLo', 'respHi', 'cardLo', 'cardHi', 'hrLo', 'hrHi',
                  'selRespSrc', 'selCardSrc', 'selWindow']) {
  $(id).addEventListener('change', () => { state.dirty = true; });
  $(id).addEventListener('input', () => { state.dirty = true; });
}

$('selOdr').addEventListener('change', (e) => state.link.setOdr(+e.target.value));
$('selAccelFs').addEventListener('change', (e) => state.link.setAccelFs(+e.target.value));
$('selGyroFs').addEventListener('change', (e) => state.link.setGyroFs(+e.target.value));
$('btnIdentify').addEventListener('click', () => state.link.identify(3));

$('selMode').addEventListener('change', (e) => {
  state.mode = e.target.value;
  state.dirty = true;
  if (state.mode === 'review' && !state.review) {
    setBanner('Review mode: load a saved session below, or import a CSV.');
  } else {
    setBanner('', false);
  }
});

$('btnTheme').addEventListener('click', () => {
  const cur = document.documentElement.getAttribute('data-theme');
  const next = cur === 'dark' ? 'light' : cur === 'light' ? '' : 'dark';
  if (next) document.documentElement.setAttribute('data-theme', next);
  else document.documentElement.removeAttribute('data-theme');
  try { localStorage.setItem('dv-theme', next); } catch { /* private window */ }
  state.dirty = true;
});

// ------------------------------------------------------------------ sessions

function name0() {
  return `neck_${new Date().toISOString().slice(0, 19).replace(/[:T-]/g, '')}`;
}

async function toggleRecording() {
  if (!state.rec.recording) {
    const meta = {
      odrHz: state.fs,
      measuredOdr: state.link.measuredOdr,
      accelScale: state.scales.accelScale,
      gyroScale: state.scales.gyroScale,
      accelFsG: state.link.status?.accelFsG ?? 2,
      gyroFsDps: state.link.status?.gyroFsDps ?? 245,
    };
    state.rec.start(meta);
    state.recStartedAt = Date.now();
    // On the bridge, also write a CSV server-side. The browser copy lives in
    // IndexedDB and is easy to lose to a cleared profile; the file on disk is
    // the one that survives and that other tools can read.
    state.link.serverRecord?.(true, name0());
  } else {
    const s = state.rec.stop(defaultSessionName());
    s.measuredOdr = state.link.measuredOdr ?? s.measuredOdr;
    if (s.idx.length) await saveSession(s);
    state.link.serverRecord?.(false);
    await refreshSessions();
  }
  syncRecordUi();
}

function defaultSessionName() {
  return new Date().toLocaleString();
}

function syncRecordUi() {
  const on = state.rec.recording;
  for (const id of ['btnRec', 'btnRecTop']) {
    const b = $(id);
    if (!b) continue;
    b.textContent = id === 'btnRecTop'
      ? (on ? '■ Stop' : '● Record')
      : (on ? 'Stop recording' : 'Start recording');
    b.className = on ? 'danger' : 'primary';
  }
  const chip = $('chipRec');
  if (chip) chip.hidden = !on;
}

$('btnRec').addEventListener('click', toggleRecording);
$('btnRecTop').addEventListener('click', toggleRecording);

function download(name, text, type) {
  const url = URL.createObjectURL(new Blob([text], { type }));
  const a = document.createElement('a');
  a.href = url; a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 5000);
}

async function refreshSessions() {
  const rows = await listSessions();
  const tb = $('tblSessions');
  if (!rows.length) {
    tb.innerHTML = '<tr><td colspan="6" style="color:var(--text-muted)">no saved sessions yet</td></tr>';
    return;
  }
  tb.innerHTML = rows.map((r) => {
    const dur = ((r.endedAt - r.startedAt) / 1000).toFixed(0);
    return `<tr>
      <td>${r.name}</td>
      <td>${new Date(r.startedAt).toLocaleString()}</td>
      <td class="num">${r.samples.toLocaleString()}</td>
      <td class="num">${dur}s</td>
      <td class="num">${r.gaps}</td>
      <td>
        <button data-act="review" data-id="${r.id}">Review</button>
        <button data-act="csv"    data-id="${r.id}">CSV</button>
        <button data-act="json"   data-id="${r.id}">JSON</button>
        <button data-act="del"    data-id="${r.id}">Delete</button>
      </td></tr>`;
  }).join('');
}

$('tblSessions').addEventListener('click', async (e) => {
  const btn = e.target.closest('button');
  if (!btn) return;
  const id = +btn.dataset.id;
  const s = await loadSession(id);
  if (!s) return;
  switch (btn.dataset.act) {
    case 'review':
      state.review = { session: s, arrays: sessionToArrays(s) };
      state.mode = 'review';
      $('selMode').value = 'review';
      state.dirty = true;
      setBanner(`Reviewing "${s.name}" — ${s.idx.length.toLocaleString()} samples`, true);
      break;
    case 'csv':  download(`${s.name}.csv`, sessionToCsv(s), 'text/csv'); break;
    case 'json': download(`${s.name}.json`, sessionToJson(s), 'application/json'); break;
    case 'del':
      if (confirm(`Delete "${s.name}"?`)) { await deleteSession(id); await refreshSessions(); }
      break;
  }
});

$('fileImport').addEventListener('change', async (e) => {
  const f = e.target.files?.[0];
  if (!f) return;
  const text = await f.text();
  let s;
  try {
    if (f.name.endsWith('.json')) {
      const j = JSON.parse(text);
      s = { ...j, idx: Uint32Array.from(j.idx), accel: Int16Array.from(j.accel),
            gyro: Int16Array.from(j.gyro) };
    } else {
      s = csvToSession(text, f.name);
    }
  } catch (err) {
    setBanner(`Could not read ${f.name}: ${err.message}`);
    return;
  }
  if (!s.idx.length) { setBanner(`No samples found in ${f.name}`); return; }
  state.review = { session: s, arrays: sessionToArrays(s) };
  state.mode = 'review';
  $('selMode').value = 'review';
  state.dirty = true;
  setBanner(`Reviewing "${s.name}" — ${s.idx.length.toLocaleString()} samples`, true);
});

// ---------------------------------------------------------------------- boot

fillSelect($('selOdr'), ODR_CHOICES, ' Hz');
fillSelect($('selAccelFs'), ACCEL_CHOICES, ' g');
fillSelect($('selGyroFs'), GYRO_CHOICES, ' °/s');
applyPreset('human');
renderDeviceTable(null);
setLinkUi(false);
syncRecordUi();
refreshSessions();

try {
  const t = localStorage.getItem('dv-theme');
  if (t) document.documentElement.setAttribute('data-theme', t);
} catch { /* storage blocked; the OS preference still applies */ }

// Prefer the bridge whenever this page came from the bridge server: it needs
// no browser Bluetooth support, no permission prompt and no device picker, and
// it keeps streaming across a page reload because the radio link belongs to
// the server rather than to the tab.
(async () => {
  if (await bridgeAvailable()) {
    state.transport = 'bridge';
    state.link = new BridgeLink();
    bindLink(state.link);
    $('txtLink').textContent = 'connecting via server…';
    $('btnConnect').textContent = 'Reconnect';
    try {
      await state.link.connect();
      setLinkUi(state.link.connected);
    } catch (err) {
      setBanner(`Could not reach the bridge server: ${err.message}`);
    }
  } else if (!navigator.bluetooth) {
    setBanner('No Web Bluetooth in this browser, and no bridge server. Run '
            + 'tools/server.py and reload — it does the Bluetooth itself, so any '
            + 'browser works.');
  }
})();

new ResizeObserver(() => { state.dirty = true; for (const k in charts) charts[k].draw(); })
  .observe(document.body);
window.dv = state;      // for poking at from the console while it streams
requestAnimationFrame(tick);
