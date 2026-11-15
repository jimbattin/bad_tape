// The EQ's model (docs/EQ.md section 2), ported from the firmware's tests (tests/eq_model.h): five bands in series, each
// the RBJ cookbook bell or shelf under the bilinear transform prewarped at its frequency, at the firmware's rate, so
// the curve is the pedal's exact response. The editor's EQ graph draws it; eqModel.test.ts checks it against the
// specification.

export type BandId = "low" | "mid1" | "mid2" | "mid3" | "high";
export type Kind = "lowShelf" | "bell" | "highShelf";

/** The firmware's audio rate (Hz). */
const FS = 96000;
/** Every gain's range, +-dB. */
export const GAIN_MAX = 12;
/** The bells' Q range. */
export const Q_MIN = 0.4;
export const Q_MAX = 8;
/** The shelves' fixed Q (no overshoot). */
export const SHELF_Q = Math.SQRT1_2;

export interface BandDef {
  id: BandId;
  kind: Kind;
  /** The handle's letter. */
  letter: string;
  label: string;
  /** The parameters (params.h) behind the band; q is null for the shelves. */
  gain: string;
  freq: string;
  q: string | null;
  /** The frequency parameter's range (Hz). */
  minHz: number;
  maxHz: number;
}

/** The bands in processing order. */
export const BANDS: readonly BandDef[] = [
  {
    id: "low",
    kind: "lowShelf",
    letter: "L",
    label: "Low shelf",
    gain: "eq_low_gain",
    freq: "eq_low_freq",
    q: null,
    minHz: 40,
    maxHz: 600,
  },
  ...([1, 2, 3] as const).map(
    (n): BandDef => ({
      id: `mid${n}`,
      kind: "bell",
      letter: `${n}`,
      label: `Mid ${n}`,
      gain: `eq_mid_gain_${n}`,
      freq: `eq_mid_freq_${n}`,
      q: `eq_mid_q_${n}`,
      minHz: [40, 200, 800][n - 1]!,
      maxHz: [800, 5000, 12000][n - 1]!,
    }),
  ),
  {
    id: "high",
    kind: "highShelf",
    letter: "H",
    label: "High shelf",
    gain: "eq_high_gain",
    freq: "eq_high_freq",
    q: null,
    minHz: 1500,
    maxHz: 16000,
  },
];

/** One band: gain (dB), frequency (Hz: a bell's centre, a shelf's half-gain point) and Q (shelves: SHELF_Q). */
export interface BandSettings {
  gain: number;
  freq: number;
  q: number;
}

export interface EqSettings {
  enable: boolean;
  post: boolean;
  bands: Record<BandId, BandSettings>;
}

/** The settings from parameter values by name; null while any is unknown. */
export function readEqSettings(get: (name: string) => number | undefined): EqSettings | null {
  const read = (name: string): number | null => {
    const x = get(name);
    return x === undefined || !Number.isFinite(x) ? null : x;
  };
  const enable = read("eq_enable");
  const post = read("eq_position");
  if (enable === null || post === null) return null;
  const bands = {} as Record<BandId, BandSettings>;
  for (const b of BANDS) {
    const gain = read(b.gain);
    const freq = read(b.freq);
    const q = b.q === null ? SHELF_Q : read(b.q);
    if (gain === null || freq === null || q === null) return null;
    bands[b.id] = { gain, freq, q };
  }
  return { enable: Math.round(enable) === 1, post: Math.round(post) === 1, bands };
}

/** |a s^2 + b s + c|^2 at s = ju. */
const quad2 = (a: number, b: number, c: number, u: number): number => (c - a * u * u) ** 2 + (b * u) ** 2;

/** One band alone in dB at f (Hz). */
export function bandDb(kind: Kind, f: number, b: BandSettings): number {
  const A = 10 ** (b.gain / 40);
  const r = Math.sqrt(A);
  const u = Math.tan((Math.PI * f) / FS) / Math.tan((Math.PI * b.freq) / FS);
  const q = b.q;
  switch (kind) {
    case "bell":
      return 10 * Math.log10(quad2(1, A / q, 1, u) / quad2(1, 1 / (A * q), 1, u));
    case "lowShelf":
      return 20 * Math.log10(A) + 10 * Math.log10(quad2(1, r / q, A, u) / quad2(A, r / q, 1, u));
    case "highShelf":
      return 20 * Math.log10(A) + 10 * Math.log10(quad2(A, r / q, 1, u) / quad2(1, r / q, A, u));
  }
}

/** The whole EQ in dB (whether or not it is enabled). */
export function eqTotalDb(s: EqSettings, f: number): number {
  let db = 0;
  for (const b of BANDS) db += bandDb(b.kind, f, s.bands[b.id]);
  return db;
}

/** The whole EQ and each band alone (dB) at freqs. */
export function eqCurves(s: EqSettings, freqs: readonly number[]): Record<BandId | "total", number[]> {
  const bands = {} as Record<BandId, number[]>;
  for (const b of BANDS) bands[b.id] = freqs.map((f) => bandDb(b.kind, f, s.bands[b.id]));
  const total = freqs.map((_, i) => BANDS.reduce((db, b) => db + bands[b.id][i]!, 0));
  return { ...bands, total };
}
