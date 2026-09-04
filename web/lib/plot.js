// Canvas strip charts and a spectrum view.
//
// Canvas rather than SVG because these redraw continuously at 208 Hz-worth of
// incoming data; a few thousand DOM nodes per frame is not a plot, it is a
// stall. Colours are read from CSS custom properties so light and dark are
// selected palettes rather than an automatic inversion, and every trace is
// direct-labelled at its right edge -- which a strip chart wants regardless,
// and which is also what discharges the low-contrast relief rule for the aqua
// slot on the light surface.

const AXIS_FONT = '11px ui-monospace, SFMono-Regular, Menlo, monospace';

function cssVar(el, name, fallback) {
  const v = getComputedStyle(el).getPropertyValue(name).trim();
  return v || fallback;
}

function niceStep(range, targetTicks) {
  if (!(range > 0)) return 1;
  const raw = range / targetTicks;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / mag;
  const step = norm <= 1 ? 1 : norm <= 2 ? 2 : norm <= 5 ? 5 : 10;
  return step * mag;
}

class BaseChart {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.pad = { l: 64, r: 58, t: 8, b: 20 };
    this.hover = null;

    canvas.addEventListener('pointermove', (e) => {
      const r = canvas.getBoundingClientRect();
      this.hover = { x: e.clientX - r.left, y: e.clientY - r.top };
      this.draw();
    });
    canvas.addEventListener('pointerleave', () => { this.hover = null; this.draw(); });
  }

  theme() {
    const el = this.canvas;
    return {
      surface: cssVar(el, '--surface-1', '#fcfcfb'),
      ink:     cssVar(el, '--text-primary', '#0b0b0b'),
      muted:   cssVar(el, '--text-secondary', '#52514e'),
      grid:    cssVar(el, '--grid', '#e6e5e2'),
      accent:  cssVar(el, '--marker', '#e34948'),
      band:    cssVar(el, '--band', 'rgba(42,120,214,0.08)'),
    };
  }

  // Keeps the backing store matched to CSS pixels; without this every line is
  // soft on a HiDPI display and the 2px marks read as 1px.
  resize() {
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (!w || !h) return false;
    if (this.canvas.width !== Math.round(w * dpr) ||
        this.canvas.height !== Math.round(h * dpr)) {
      this.canvas.width = Math.round(w * dpr);
      this.canvas.height = Math.round(h * dpr);
    }
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = w; this.h = h;
    this.plotW = w - this.pad.l - this.pad.r;
    this.plotH = h - this.pad.t - this.pad.b;
    return this.plotW > 10 && this.plotH > 10;
  }
}

export class StripChart extends BaseChart {
  constructor(canvas, { yLabel = '', unit = '', fixedRange = null } = {}) {
    super(canvas);
    this.yLabel = yLabel;
    this.unit = unit;
    this.fixedRange = fixedRange;
    this.series = [];
    this.markers = [];
    this.fs = 1;
    this.tStart = 0;
  }

  /**
   * @param series  [{ name, data: Float64Array, color }]
   * @param markers indices into the sample arrays, drawn as ticks
   */
  setData({ series, fs, tStart = 0, markers = [] }) {
    this.series = series;
    this.fs = fs || 1;
    this.tStart = tStart;
    this.markers = markers;
  }

  draw() {
    if (!this.resize()) return;
    const { ctx, pad } = this;
    const th = this.theme();

    ctx.clearRect(0, 0, this.w, this.h);
    ctx.fillStyle = th.surface;
    ctx.fillRect(0, 0, this.w, this.h);

    const n = this.series[0]?.data?.length ?? 0;
    if (!n) {
      ctx.fillStyle = th.muted;
      ctx.font = AXIS_FONT;
      ctx.textAlign = 'center';
      ctx.fillText('waiting for data', this.w / 2, this.h / 2);
      return;
    }

    let lo, hi;
    if (this.fixedRange) {
      [lo, hi] = this.fixedRange;
    } else {
      lo = Infinity; hi = -Infinity;
      for (const s of this.series) {
        for (let i = 0; i < s.data.length; i++) {
          const v = s.data[i];
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
      if (!isFinite(lo) || !isFinite(hi)) { lo = -1; hi = 1; }
      if (hi - lo < 1e-12) { hi = lo + 1e-6; }
      const m = (hi - lo) * 0.08;
      lo -= m; hi += m;
    }

    const x = (i) => pad.l + (i / (n - 1)) * this.plotW;
    const y = (v) => pad.t + (1 - (v - lo) / (hi - lo)) * this.plotH;

    // --- recessive grid ----------------------------------------------------
    ctx.strokeStyle = th.grid;
    ctx.lineWidth = 1;
    ctx.fillStyle = th.muted;
    ctx.font = AXIS_FONT;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const yStep = niceStep(hi - lo, 4);
    for (let v = Math.ceil(lo / yStep) * yStep; v <= hi; v += yStep) {
      const py = Math.round(y(v)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(pad.l, py);
      ctx.lineTo(pad.l + this.plotW, py);
      ctx.stroke();
      const digits = yStep < 0.01 ? 4 : yStep < 0.1 ? 3 : yStep < 1 ? 2 : 1;
      ctx.fillText(v.toFixed(digits), pad.l - 6, py);
    }

    const durSec = n / this.fs;
    const tStep = niceStep(durSec, 6);
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let t = 0; t <= durSec; t += tStep) {
      const px = Math.round(x(t * this.fs)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(px, pad.t);
      ctx.lineTo(px, pad.t + this.plotH);
      ctx.stroke();
      ctx.fillText(`${(this.tStart + t).toFixed(tStep < 1 ? 1 : 0)}s`, px, pad.t + this.plotH + 4);
    }

    // --- markers (detected beats or breaths) -------------------------------
    if (this.markers.length) {
      ctx.strokeStyle = th.accent;
      ctx.lineWidth = 1;
      ctx.globalAlpha = 0.55;
      for (const m of this.markers) {
        if (m < 0 || m >= n) continue;
        const px = Math.round(x(m)) + 0.5;
        ctx.beginPath();
        ctx.moveTo(px, pad.t);
        ctx.lineTo(px, pad.t + 8);
        ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }

    // --- traces ------------------------------------------------------------
    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.l, pad.t, this.plotW, this.plotH);
    ctx.clip();

    // More samples than pixels is the normal case at 208 Hz, so draw a
    // min/max envelope per column instead of a polyline through every point:
    // same picture, no aliasing away of the sharp cardiac transients, and it
    // does not get slower as the window grows.
    const cols = Math.max(1, Math.round(this.plotW));
    for (const s of this.series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.beginPath();
      if (n <= cols * 1.5) {
        for (let i = 0; i < n; i++) {
          const px = x(i), py = y(s.data[i]);
          i ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
        }
      } else {
        const per = n / cols;
        for (let c = 0; c < cols; c++) {
          const a = Math.floor(c * per);
          const b = Math.min(n, Math.floor((c + 1) * per));
          let mn = Infinity, mx = -Infinity;
          for (let i = a; i < b; i++) {
            const v = s.data[i];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
          }
          if (!isFinite(mn)) continue;
          const px = pad.l + c;
          c ? ctx.lineTo(px, y(mx)) : ctx.moveTo(px, y(mx));
          ctx.lineTo(px, y(mn));
        }
      }
      ctx.stroke();
    }
    ctx.restore();

    // --- direct labels -----------------------------------------------------
    //
    // Two traces that happen to end at the same value would otherwise print on
    // top of each other -- which is exactly what a band-passed signal and its
    // envelope do, since both sit near zero between events. Nudging them apart
    // keeps identity readable, and identity here is not carried by colour
    // alone precisely so that it survives.
    ctx.font = AXIS_FONT;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    const labels = this.series.map((s) => ({
      name: s.name, color: s.color,
      y: Math.max(pad.t + 7, Math.min(pad.t + this.plotH - 7, y(s.data[n - 1]))),
    }));
    labels.sort((a, b) => a.y - b.y);
    const minGap = 13;
    for (let i = 1; i < labels.length; i++) {
      if (labels[i].y - labels[i - 1].y < minGap) labels[i].y = labels[i - 1].y + minGap;
    }
    const overflow = labels.length
      ? labels[labels.length - 1].y - (pad.t + this.plotH - 7) : 0;
    if (overflow > 0) for (const l of labels) l.y -= overflow;
    for (const l of labels) {
      ctx.fillStyle = l.color;
      ctx.fillText(l.name, pad.l + this.plotW + 6, l.y);
    }

    if (this.yLabel) {
      ctx.save();
      ctx.translate(11, pad.t + this.plotH / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.fillStyle = th.muted;
      ctx.textAlign = 'center';
      ctx.fillText(this.yLabel, 0, 0);
      ctx.restore();
    }

    this.#drawCrosshair(x, y, lo, hi, n, th);
  }

  #drawCrosshair(x, y, lo, hi, n, th) {
    const hv = this.hover;
    if (!hv) return;
    const { ctx, pad } = this;
    if (hv.x < pad.l || hv.x > pad.l + this.plotW) return;

    const i = Math.round(((hv.x - pad.l) / this.plotW) * (n - 1));
    if (i < 0 || i >= n) return;
    const px = Math.round(x(i)) + 0.5;

    ctx.strokeStyle = th.muted;
    ctx.globalAlpha = 0.5;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px, pad.t);
    ctx.lineTo(px, pad.t + this.plotH);
    ctx.stroke();
    ctx.globalAlpha = 1;

    const lines = [`t ${(this.tStart + i / this.fs).toFixed(2)}s`];
    for (const s of this.series) {
      lines.push(`${s.name} ${s.data[i].toFixed(4)}${this.unit}`);
    }
    ctx.font = AXIS_FONT;
    const wBox = Math.max(...lines.map((l) => ctx.measureText(l).width)) + 12;
    const hBox = lines.length * 14 + 8;
    let bx = px + 8;
    if (bx + wBox > pad.l + this.plotW) bx = px - wBox - 8;
    const by = pad.t + 6;

    ctx.fillStyle = th.surface;
    ctx.globalAlpha = 0.94;
    ctx.fillRect(bx, by, wBox, hBox);
    ctx.globalAlpha = 1;
    ctx.strokeStyle = th.grid;
    ctx.strokeRect(bx + 0.5, by + 0.5, wBox, hBox);

    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    lines.forEach((l, k) => {
      ctx.fillStyle = k === 0 ? th.muted : this.series[k - 1].color;
      ctx.fillText(l, bx + 6, by + 5 + k * 14);
    });
  }
}

export class SpectrumChart extends BaseChart {
  constructor(canvas) {
    super(canvas);
    this.pad = { l: 64, r: 14, t: 8, b: 22 };
    this.freqs = [];
    this.mags = [];
    this.bands = [];
    this.color = '#2a78d6';
  }

  /** @param bands [{ lo, hi, label }] shaded to show where we are looking */
  setData({ freqs, mags, bands = [], color }) {
    this.freqs = freqs; this.mags = mags; this.bands = bands;
    if (color) this.color = color;
  }

  draw() {
    if (!this.resize()) return;
    const { ctx, pad } = this;
    const th = this.theme();
    ctx.clearRect(0, 0, this.w, this.h);
    ctx.fillStyle = th.surface;
    ctx.fillRect(0, 0, this.w, this.h);

    const n = this.freqs.length;
    if (!n) {
      ctx.fillStyle = th.muted;
      ctx.font = AXIS_FONT;
      ctx.textAlign = 'center';
      ctx.fillText('waiting for data', this.w / 2, this.h / 2);
      return;
    }

    const fMax = this.freqs[n - 1];

    // Decibels, not linear magnitude.
    //
    // On a linear axis the breathing peak is two or three orders of magnitude
    // above everything cardiac, so the whole 4-30 Hz region -- the part this
    // view exists to let you inspect -- renders as a flat line on the axis.
    // A 60 dB window shows both at once, which is the only way to compare them.
    const FLOOR_DB = -60;
    let mMax = 0;
    for (let i = 1; i < n; i++) if (this.mags[i] > mMax) mMax = this.mags[i];
    if (mMax <= 0) mMax = 1;
    const db = (m) => Math.max(FLOOR_DB, 20 * Math.log10(Math.max(m, 1e-12) / mMax));

    const x = (f) => pad.l + (f / fMax) * this.plotW;
    const y = (m) => pad.t + (1 - (db(m) - FLOOR_DB) / -FLOOR_DB) * this.plotH;

    // Band shading first, so the trace draws over it.
    const drawnLabels = [];
    for (const b of this.bands) {
      const x0 = x(Math.max(0, b.lo));
      const x1 = x(Math.min(fMax, b.hi));
      if (x1 <= x0) continue;
      ctx.fillStyle = b.fill || th.band;
      ctx.fillRect(x0, pad.t, Math.max(2, x1 - x0), this.plotH);
      drawnLabels.push({ text: b.label, x: x0 + 3 });
    }

    // Band labels along the bottom, skipped when the previous one would run
    // into them. A narrow band on a 0-40 Hz axis is only a few pixels wide, so
    // "breathing" and "cardiac" printed at their left edges collided into one
    // unreadable word.
    ctx.font = AXIS_FONT;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillStyle = th.muted;
    let lastEnd = -Infinity;
    for (const l of drawnLabels) {
      const w = ctx.measureText(l.text).width;
      const lx = Math.min(l.x, pad.l + this.plotW - w);
      if (lx < lastEnd + 6) continue;
      ctx.fillText(l.text, lx, pad.t + this.plotH - 3);
      lastEnd = lx + w;
    }

    // --- axes -------------------------------------------------------------
    ctx.strokeStyle = th.grid;
    ctx.lineWidth = 1;
    ctx.fillStyle = th.muted;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let d = 0; d >= FLOOR_DB; d -= 20) {
      const py = Math.round(pad.t + (1 - (d - FLOOR_DB) / -FLOOR_DB) * this.plotH) + 0.5;
      ctx.beginPath();
      ctx.moveTo(pad.l, py);
      ctx.lineTo(pad.l + this.plotW, py);
      ctx.stroke();
      ctx.fillText(`${d}`, pad.l - 6, py);
    }
    ctx.save();
    ctx.translate(11, pad.t + this.plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = 'center';
    ctx.fillText('dB', 0, 0);
    ctx.restore();

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const fStep = niceStep(fMax, 8);
    for (let f = 0; f <= fMax + 1e-9; f += fStep) {
      const px = Math.round(x(f)) + 0.5;
      ctx.beginPath();
      ctx.moveTo(px, pad.t);
      ctx.lineTo(px, pad.t + this.plotH);
      ctx.stroke();
      ctx.fillText(`${f.toFixed(fStep < 1 ? 1 : 0)}`, px, pad.t + this.plotH + 4);
    }
    ctx.textAlign = 'right';
    ctx.fillText('Hz', pad.l + this.plotW, pad.t + this.plotH + 4);

    // --- trace ------------------------------------------------------------
    ctx.strokeStyle = this.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 1; i < n; i++) {
      const px = x(this.freqs[i]), py = y(this.mags[i]);
      i === 1 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Label the dominant line -- the number the whole view exists to produce.
    let bi = 1;
    for (let i = 2; i < n; i++) if (this.mags[i] > this.mags[bi]) bi = i;
    const bpx = x(this.freqs[bi]), bpy = y(this.mags[bi]);
    ctx.fillStyle = th.accent;
    ctx.beginPath();
    ctx.arc(bpx, bpy, 4, 0, Math.PI * 2);
    ctx.fill();

    const text = `${this.freqs[bi].toFixed(2)} Hz · ${(this.freqs[bi] * 60).toFixed(0)}/min`;
    const tw = ctx.measureText(text).width;
    let tx = bpx + 8;
    if (tx + tw > pad.l + this.plotW) tx = bpx - tw - 8;
    tx = Math.max(pad.l + 2, tx);
    const ty = Math.max(pad.t + 12, bpy - 6);
    ctx.fillStyle = th.surface;
    ctx.globalAlpha = 0.85;
    ctx.fillRect(tx - 3, ty - 11, tw + 6, 13);
    ctx.globalAlpha = 1;
    ctx.fillStyle = th.ink;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(text, tx, ty + 1);
  }
}
