// Replays recorded sessions through the real analysis chain and reports what
// the gate accepts. The point is to check the gate against the failure it was
// written for -- the band-edge pin -- using the same code the page runs.
import { readFileSync, readdirSync } from 'node:fs';
import * as dsp from '../web/lib/dsp.js';

const FS = 208, WIN = 30, STEP = 5;
const BAND = { lo: 0.10, hi: 0.60 };          // human neck preset

function load(path) {
  const idx = [], ax = [], ay = [], az = [];
  for (const line of readFileSync(path, 'utf8').split('\n')) {
    if (!line || line[0] === '#' || line.startsWith('idx')) continue;
    const c = line.split(',');
    if (c.length < 7) continue;
    idx.push(+c[0]); ax.push(+c[1]); ay.push(+c[2]); az.push(+c[3]);
  }
  // Fill index gaps so the window length matches wall time, as the page does.
  const out = { ax: [], ay: [], az: [] };
  for (let i = 1; i < idx.length; i++) {
    const gap = idx[i] - idx[i - 1];
    for (let k = 0; k < gap; k++) {
      const t = k / gap;
      out.ax.push(ax[i - 1] + (ax[i] - ax[i - 1]) * t);
      out.ay.push(ay[i - 1] + (ay[i] - ay[i - 1]) * t);
      out.az.push(az[i - 1] + (az[i] - az[i - 1]) * t);
    }
  }
  return out;
}

for (const f of readdirSync('data').filter((f) => f.startsWith('neck_')).sort()) {
  const s = load(`data/${f}`);
  const W = WIN * FS, STEP_N = STEP * FS;
  const accepted = [], reasons = {};
  for (let o = 0; o + W <= s.ax.length; o += STEP_N) {
    const ax = s.ax.slice(o, o + W), ay = s.ay.slice(o, o + W), az = s.az.slice(o, o + W);
    const src = dsp.principalProjection(ax, ay, az);
    const resp = dsp.respirationBand(src, FS, {
      slowSec: 1 / BAND.lo, fastSec: 1 / (BAND.hi * 2.5),
    });
    const dec = Math.max(1, Math.floor(FS / Math.max(4, BAND.hi * 8)));
    const r = dsp.autocorrRate(dsp.decimate(resp, dec), FS / dec, BAND.lo, BAND.hi);
    const motion = dsp.motionLevel(ax, ay, az, FS);
    const g = dsp.gateRate(r.bpm, {
      ...BAND, confidence: r.confidence, motion, maxMotion: 400,
    });
    if (g.ok) accepted.push(r.bpm);
    else reasons[g.reason] = (reasons[g.reason] ?? 0) + 1;
  }
  const total = accepted.length + Object.values(reasons).reduce((a, b) => a + b, 0);
  const { median, n } = dsp.acceptedMedian(accepted);
  const cov = total ? Math.round((100 * n) / total) : 0;
  const why = Object.entries(reasons).map(([k, v]) => `${k}×${v}`).join(', ') || '—';
  console.log(
    `  ${f.padEnd(26)} accepted ${String(n).padStart(2)}/${String(total).padStart(2)}` +
    ` (${String(cov).padStart(3)}%)  median ${median ? median.toFixed(1).padStart(5) : '   — '} br/min` +
    `   rejected: ${why}`);
}
