// Signal processing for respiration and cardiac extraction from IMU data.
//
// Everything here works on whole arrays rather than sample-by-sample state.
// That is deliberate: the live view and the review view then run identical
// code, and changing a filter corner re-renders the entire window instead of
// only affecting samples that arrive afterwards -- which is what you want when
// the whole exercise is working out what the right corner is.
//
// What we are looking for, and why the defaults are where they are:
//
//   Respiration  0.1-1 Hz resting (6-60 breaths/min), but a panting dog runs
//                to 5 Hz. Appears as slow tilt of the gravity vector, so it
//                lives in the accelerometer and is large -- tens of milli-g.
//
//   Cardiac      Carotid pulsation picked up as micro-motion, roughly 4-25 Hz,
//                and small: single milli-g. At the neck the gyroscope often
//                sees it better than the accelerometer, because the motion is
//                a rotation of the skin surface rather than a translation.

// ---------------------------------------------------------------- filtering

function biquad(x, b0, b1, b2, a1, a2) {
  const y = new Float64Array(x.length);
  let x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  for (let i = 0; i < x.length; i++) {
    const xi = x[i];
    const yi = b0 * xi + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = xi; y2 = y1; y1 = yi;
    y[i] = yi;
  }
  return y;
}

function reversed(x) {
  const y = new Float64Array(x.length);
  for (let i = 0; i < x.length; i++) y[i] = x[x.length - 1 - i];
  return y;
}

// Runs a section forwards then backwards. Doubles the roll-off and, more
// importantly, cancels phase entirely -- so a detected beat sits where it
// actually happened rather than a filter delay later. That matters here
// because beat-to-beat timing is the measurement, not a by-product.
function filtfilt(x, coeffs) {
  const [b0, b1, b2, a1, a2] = coeffs;
  const fwd = biquad(x, b0, b1, b2, a1, a2);
  return reversed(biquad(reversed(fwd), b0, b1, b2, a1, a2));
}

function lowpassCoeffs(fs, fc) {
  const w0 = 2 * Math.PI * Math.min(fc, fs * 0.49) / fs;
  const alpha = Math.sin(w0) / (2 * Math.SQRT1_2);
  const cw = Math.cos(w0);
  const a0 = 1 + alpha;
  return [(1 - cw) / 2 / a0, (1 - cw) / a0, (1 - cw) / 2 / a0,
          (-2 * cw) / a0, (1 - alpha) / a0];
}

function highpassCoeffs(fs, fc) {
  const w0 = 2 * Math.PI * Math.max(fc, 1e-4) / fs;
  const alpha = Math.sin(w0) / (2 * Math.SQRT1_2);
  const cw = Math.cos(w0);
  const a0 = 1 + alpha;
  return [(1 + cw) / 2 / a0, -(1 + cw) / a0, (1 + cw) / 2 / a0,
          (-2 * cw) / a0, (1 - alpha) / a0];
}

export function lowpass(x, fs, fc)  { return filtfilt(x, lowpassCoeffs(fs, fc)); }
export function highpass(x, fs, fc) { return filtfilt(x, highpassCoeffs(fs, fc)); }

/** Band-pass by cascading a high-pass and a low-pass, both zero-phase. */
export function bandpass(x, fs, lo, hi) {
  return lowpass(highpass(x, fs, lo), fs, hi);
}

/**
 * Respiration band, built from a difference of moving averages rather than a
 * biquad.
 *
 * At 208 Hz a 0.1 Hz corner is fc/fs = 0.0005, where a direct-form biquad's
 * coefficients crowd together and the filter becomes numerically fragile --
 * it can ring or drift on exactly the slow signal it is meant to pass. Two
 * boxcar averages differenced have no such problem, cost one add per sample,
 * and give a perfectly serviceable passband between their two corners.
 */
export function respirationBand(x, fs, { slowSec = 6, fastSec = 0.35 } = {}) {
  const slow = movingAverage(x, Math.max(3, Math.round(slowSec * fs)));
  const fast = movingAverage(x, Math.max(1, Math.round(fastSec * fs)));
  const y = new Float64Array(x.length);
  for (let i = 0; i < x.length; i++) y[i] = fast[i] - slow[i];
  return y;
}

/** Centred boxcar average, edge-clamped. */
export function movingAverage(x, n) {
  const y = new Float64Array(x.length);
  if (n <= 1) { y.set(x); return y; }
  const half = n >> 1;
  let sum = 0;
  const cum = new Float64Array(x.length + 1);
  for (let i = 0; i < x.length; i++) { sum += x[i]; cum[i + 1] = sum; }
  for (let i = 0; i < x.length; i++) {
    const a = Math.max(0, i - half);
    const b = Math.min(x.length, i + half + 1);
    y[i] = (cum[b] - cum[a]) / (b - a);
  }
  return y;
}

/** Rectified and smoothed magnitude -- the "how much is happening" trace. */
export function envelope(x, fs, smoothHz = 3) {
  const r = new Float64Array(x.length);
  for (let i = 0; i < x.length; i++) r[i] = Math.abs(x[i]);
  return lowpass(r, fs, smoothHz);
}

/** Drops the sample rate by an integer factor. Assumes x is already band-limited. */
export function decimate(x, factor) {
  if (factor <= 1) return x;
  const n = Math.floor(x.length / factor);
  const y = new Float64Array(n);
  for (let i = 0; i < n; i++) y[i] = x[i * factor];
  return y;
}

// ------------------------------------------------------------- axis choice

/**
 * Projects three axes onto their dominant direction of variation.
 *
 * Which axis carries the signal depends entirely on how the device ended up
 * sitting on the neck, and on a collar it will differ every time it is put on.
 * Picking the largest-variance axis throws away whatever landed in the other
 * two; projecting onto the first principal component keeps all of it and needs
 * no calibration step. The power iteration converges in a handful of rounds
 * for a 3x3 covariance.
 */
export function principalProjection(ax, ay, az) {
  const n = ax.length;
  if (!n) return new Float64Array(0);
  const mean = [0, 0, 0];
  for (let i = 0; i < n; i++) { mean[0] += ax[i]; mean[1] += ay[i]; mean[2] += az[i]; }
  mean[0] /= n; mean[1] /= n; mean[2] /= n;

  const c = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < n; i++) {
    const v = [ax[i] - mean[0], ay[i] - mean[1], az[i] - mean[2]];
    for (let r = 0; r < 3; r++) for (let s = 0; s < 3; s++) c[r][s] += v[r] * v[s];
  }

  let v = [1, 1, 1];
  for (let it = 0; it < 24; it++) {
    const w = [
      c[0][0] * v[0] + c[0][1] * v[1] + c[0][2] * v[2],
      c[1][0] * v[0] + c[1][1] * v[1] + c[1][2] * v[2],
      c[2][0] * v[0] + c[2][1] * v[1] + c[2][2] * v[2],
    ];
    const norm = Math.hypot(w[0], w[1], w[2]);
    if (norm < 1e-18) break;
    v = [w[0] / norm, w[1] / norm, w[2] / norm];
  }

  const y = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    y[i] = (ax[i] - mean[0]) * v[0] + (ay[i] - mean[1]) * v[1] + (az[i] - mean[2]) * v[2];
  }
  return y;
}

export function magnitude(ax, ay, az) {
  const y = new Float64Array(ax.length);
  for (let i = 0; i < ax.length; i++) y[i] = Math.hypot(ax[i], ay[i], az[i]);
  return y;
}

// ------------------------------------------------------------ rate estimate

/**
 * Rate from autocorrelation, with a confidence figure.
 *
 * Counting peaks is the obvious approach and it is fragile: one missed or one
 * doubled peak moves the answer by a large fraction, and a noisy cardiac trace
 * produces both. Autocorrelation uses every cycle in the window at once, so a
 * periodicity that is real but buried still shows up, and -- the part that
 * matters for deciding whether to believe the number at all -- the height of
 * the peak relative to zero lag says how periodic the signal actually was.
 *
 * Returns { hz, bpm, confidence, lags, curve } with confidence in 0..1.
 */
export function autocorrRate(x, fs, minHz, maxHz) {
  const n = x.length;
  const minLag = Math.max(2, Math.floor(fs / maxHz));
  const maxLag = Math.min(n - 2, Math.ceil(fs / minHz));
  if (maxLag <= minLag) return { hz: null, bpm: null, confidence: 0, lags: [], curve: [] };

  let mean = 0;
  for (let i = 0; i < n; i++) mean += x[i];
  mean /= n;

  let denom = 0;
  for (let i = 0; i < n; i++) { const d = x[i] - mean; denom += d * d; }
  if (denom < 1e-20) return { hz: null, bpm: null, confidence: 0, lags: [], curve: [] };

  const lags = [];
  const curve = [];
  let best = -Infinity, bestLag = -1;
  for (let lag = minLag; lag <= maxLag; lag++) {
    let acc = 0;
    for (let i = 0; i + lag < n; i++) acc += (x[i] - mean) * (x[i + lag] - mean);
    // Normalising by the full-window energy (rather than the overlap) biases
    // long lags downward, which is the right prior: half the window is not
    // enough evidence for a periodicity claim.
    const r = acc / denom;
    lags.push(lag); curve.push(r);
    if (r > best) { best = r; bestLag = lag; }
  }
  if (bestLag < 0) return { hz: null, bpm: null, confidence: 0, lags, curve };

  // Parabolic interpolation around the peak: at 208 Hz one lag step is ~1.5
  // bpm at a 100 bpm heart rate, which is coarser than the measurement
  // deserves.
  const i = bestLag - minLag;
  let refined = bestLag;
  if (i > 0 && i < curve.length - 1) {
    const y0 = curve[i - 1], y1 = curve[i], y2 = curve[i + 1];
    const d = y0 - 2 * y1 + y2;
    if (Math.abs(d) > 1e-12) refined = bestLag + 0.5 * (y0 - y2) / d;
  }

  const hz = fs / refined;
  return {
    hz,
    bpm: hz * 60,
    confidence: Math.max(0, Math.min(1, best)),
    lags,
    curve,
  };
}

/**
 * Peak picker with a refractory period, for marking individual beats/breaths.
 *
 * The threshold is a fraction of the window's RMS rather than a fixed value,
 * because the amplitude depends on how tightly the device is strapped down and
 * changes every time it is repositioned.
 */
export function findPeaks(x, fs, { minSepSec = 0.3, threshRms = 0.6 } = {}) {
  const n = x.length;
  if (!n) return [];
  let sq = 0;
  for (let i = 0; i < n; i++) sq += x[i] * x[i];
  const thresh = Math.sqrt(sq / n) * threshRms;
  const minSep = Math.max(1, Math.round(minSepSec * fs));

  const peaks = [];
  let last = -Infinity;
  for (let i = 1; i < n - 1; i++) {
    if (x[i] <= thresh) continue;
    if (x[i] < x[i - 1] || x[i] < x[i + 1]) continue;
    if (i - last < minSep) {
      // Keep the taller of two peaks inside one refractory window.
      if (peaks.length && x[i] > x[peaks[peaks.length - 1]]) {
        peaks[peaks.length - 1] = i;
        last = i;
      }
      continue;
    }
    peaks.push(i);
    last = i;
  }
  return peaks;
}

/** Beat-to-beat interval variability, as a rough signal-quality hint. */
export function intervalStats(peaks, fs) {
  if (peaks.length < 3) return { meanBpm: null, sdMs: null, n: peaks.length };
  const iv = [];
  for (let i = 1; i < peaks.length; i++) iv.push((peaks[i] - peaks[i - 1]) / fs);
  const mean = iv.reduce((a, b) => a + b, 0) / iv.length;
  const varr = iv.reduce((a, b) => a + (b - mean) ** 2, 0) / iv.length;
  return { meanBpm: 60 / mean, sdMs: Math.sqrt(varr) * 1000, n: peaks.length };
}

// ------------------------------------------------------------------- spectra

function fftRadix2(re, im) {
  const n = re.length;
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j], re[i]];
      [im[i], im[j]] = [im[j], im[i]];
    }
  }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len;
    const wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const ur = re[i + k], ui = im[i + k];
        const vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci;
        const vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
        re[i + k] = ur + vr; im[i + k] = ui + vi;
        re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
        const ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr; cr = ncr;
      }
    }
  }
}

/**
 * Hann-windowed magnitude spectrum.
 *
 * This is the tool for the actual question the hardware is here to answer --
 * where in the spectrum does a breath or a heartbeat show up on this body, at
 * this mounting point, through this much tape. Filter corners chosen from a
 * textbook are a starting guess; the spectrum is the evidence.
 */
export function spectrum(x, fs, maxHz = 40) {
  let n = 1;
  while (n * 2 <= x.length) n *= 2;
  if (n < 64) return { freqs: [], mags: [] };

  const re = new Float64Array(n);
  const im = new Float64Array(n);
  const off = x.length - n;
  let mean = 0;
  for (let i = 0; i < n; i++) mean += x[off + i];
  mean /= n;
  for (let i = 0; i < n; i++) {
    const w = 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (n - 1));
    re[i] = (x[off + i] - mean) * w;
  }
  fftRadix2(re, im);

  const kMax = Math.min(n / 2, Math.ceil((maxHz * n) / fs));
  const freqs = new Float64Array(kMax);
  const mags = new Float64Array(kMax);
  for (let k = 0; k < kMax; k++) {
    freqs[k] = (k * fs) / n;
    mags[k] = Math.hypot(re[k], im[k]) / n;
  }
  return { freqs, mags };
}

// ------------------------------------------------------------------- gating

/**
 * Whether a rate estimate is worth acting on.
 *
 * Written for overnight sleeping-respiratory-rate monitoring, where the point
 * is not to always produce a number but to never produce a wrong one. Three
 * ways an estimate goes bad, in the order they bite:
 *
 * 1. **Band edge.** With no real periodicity in the window, autocorrelation
 *    settles wherever the search happens to peak, and that is very often the
 *    top of the band -- 0.60 Hz reads as 36.3 breaths/min. That is not a
 *    harmless wrong answer: it lands squarely in the range that matters
 *    clinically, so a restless minute looks exactly like the thing the device
 *    exists to detect. Rejected before anything else.
 * 2. **Low confidence.** The autocorrelation peak height already says how
 *    periodic the window was; below ~0.4 it is not saying much.
 * 3. **Motion.** Movement swamps a millimetre-scale chest signal. The
 *    accelerometer measures it directly, so there is no need to infer it.
 *
 * Returns { ok, reason } -- reason is null when accepted, and otherwise names
 * which test failed, so a rejected window can be shown rather than hidden.
 */
export function gateRate(rate, {
  lo, hi, confidence = 0, motion = null, maxMotion = null,
  minConfidence = 0.4, edgeFrac = 0.04,
} = {}) {
  if (rate == null || !Number.isFinite(rate)) return { ok: false, reason: 'no estimate' };
  const hz = rate / 60;
  // A window's worth of tolerance at each end: the parabolic interpolation in
  // autocorrRate can place the peak slightly outside the searched range, so an
  // exact equality test would miss the very case this exists to catch.
  const span = hi - lo;
  if (hz >= hi - span * edgeFrac) return { ok: false, reason: 'pinned to band top' };
  if (hz <= lo + span * edgeFrac) return { ok: false, reason: 'pinned to band bottom' };
  if (confidence < minConfidence) return { ok: false, reason: 'low confidence' };
  if (maxMotion != null && motion != null && motion > maxMotion)
    return { ok: false, reason: 'movement' };
  return { ok: true, reason: null };
}

/**
 * Median of the accepted estimates, for a trend rather than an instant.
 *
 * Vets read sleeping respiratory rate as a sustained figure over a night, not
 * as a single breath count, and a median over accepted windows is both closer
 * to what they want and far harder to fool than any one window. `coverage` is
 * the fraction of windows that passed the gate -- a high median over 3% of the
 * night is not a measurement, and this is what says so.
 */
export function acceptedMedian(values) {
  const ok = values.filter((v) => v != null && Number.isFinite(v)).sort((a, b) => a - b);
  if (!ok.length) return { median: null, n: 0 };
  const m = ok.length % 2
    ? ok[(ok.length - 1) / 2]
    : (ok[ok.length / 2 - 1] + ok[ok.length / 2]) / 2;
  return { median: m, n: ok.length };
}

/** RMS of the low-frequency accelerometer magnitude: a direct movement measure. */
export function motionLevel(ax, ay, az, fs) {
  const n = ax.length;
  const mag = new Float64Array(n);
  for (let i = 0; i < n; i++) mag[i] = Math.hypot(ax[i], ay[i], az[i]);
  let mean = 0;
  for (let i = 0; i < n; i++) mean += mag[i];
  mean /= n;
  // Below the breathing band: this is posture and gross movement, not breath.
  const slow = lowpass(mag.map((v) => v - mean), fs, 3);
  let sq = 0;
  for (let i = 0; i < n; i++) sq += slow[i] * slow[i];
  return Math.sqrt(sq / n);
}
