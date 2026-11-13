import { expect, test } from "bun:test";
import { readFileSync } from "node:fs";
import {
  BANDS,
  bandDb,
  type EqSettings,
  eqCurves,
  eqTotalDb,
  GAIN_MAX,
  Q_MAX,
  Q_MIN,
  readEqSettings,
  SHELF_Q,
} from "./eqModel";

const doc = readFileSync(new URL("../../docs/MIDI.md", import.meta.url), "utf8");

/** The parameter table of docs/MIDI.md: name → [min, max]. */
function ranges(): Map<string, [number, number]> {
  const rows = new Map<string, [number, number]>();
  for (const line of doc.split("\n")) {
    const c = line.split("|");
    // |Id|`name`|Unit|Min|Max|Default|Taper|Control|CC|Smoothing ms|Group|
    if (c.length === 13 && /^\d+$/.test(c[1]!)) rows.set(c[2]!.replaceAll("`", ""), [Number(c[4]), Number(c[5])]);
  }
  return rows;
}

// Setting A (docs/EQ.md section 3).
const SETTING_A: EqSettings = {
  enable: true,
  post: false,
  bands: {
    low: { gain: 6, freq: 80, q: SHELF_Q },
    mid1: { gain: -4, freq: 250, q: 1.4 },
    mid2: { gain: 3, freq: 1200, q: 0.7 },
    mid3: { gain: 5, freq: 3500, q: 2 },
    high: { gain: -6, freq: 8000, q: SHELF_Q },
  },
};

test("BANDS match docs/MIDI.md", () => {
  const r = ranges();
  for (const b of BANDS) {
    expect(r.get(b.gain)).toEqual([-GAIN_MAX, GAIN_MAX]);
    expect(r.get(b.freq)).toEqual([b.minHz, b.maxHz]);
    if (b.q !== null) expect(r.get(b.q)).toEqual([Q_MIN, Q_MAX]);
    expect(b.q === null).toBe(b.kind !== "bell");
  }
});

test("laws: bells reach their gain at f0, shelves half of it, a cut mirrors its boost", () => {
  for (const b of BANDS) {
    const f0 = Math.sqrt(b.minHz * b.maxHz);
    const q = b.kind === "bell" ? 1.4 : SHELF_Q;
    for (const g of [-12, -6, 6, 12]) {
      const at = bandDb(b.kind, f0, { gain: g, freq: f0, q });
      expect(Math.abs(at - (b.kind === "bell" ? g : g / 2))).toBeLessThan(1e-9);
      for (const f of [20, f0, 10000]) {
        const sum = bandDb(b.kind, f, { gain: g, freq: f0, q }) + bandDb(b.kind, f, { gain: -g, freq: f0, q });
        expect(Math.abs(sum)).toBeLessThan(1e-9);
      }
    }
  }
});

test("whole setting A matches docs/EQ.md section 3", () => {
  const f = [30, 50, 80, 120, 250, 500, 1000, 1200, 2000, 3500, 5000, 8000, 12000, 16000];
  const a = [5.85, 5.09, 2.78, 0.42, -3.66, 0.31, 2.79, 3.09, 2.64, 5.48, 1.14, -2.53, -4.89, -5.63];
  f.forEach((hz, i) => {
    const got = eqTotalDb(SETTING_A, hz);
    if (Math.abs(got - a[i]!) > 0.0051) throw new Error(`A at ${hz} Hz: ${got.toFixed(4)} dB, spec ${a[i]} dB`);
  });
});

test("eqCurves: each band alone and their sum, the whole EQ", () => {
  const f = [20, 80, 250, 1200, 3500, 8000, 20000];
  const c = eqCurves(SETTING_A, f);
  expect(c.total).toEqual(f.map((hz) => eqTotalDb(SETTING_A, hz)));
  for (const b of BANDS) expect(c[b.id]).toEqual(f.map((hz) => bandDb(b.kind, hz, SETTING_A.bands[b.id])));
});

test("readEqSettings needs every parameter", () => {
  const values: Record<string, number> = {
    eq_enable: 1,
    eq_position: 0,
    eq_low_gain: 6,
    eq_low_freq: 80,
    eq_mid_gain_1: -4,
    eq_mid_freq_1: 250,
    eq_mid_q_1: 1.4,
    eq_mid_gain_2: 3,
    eq_mid_freq_2: 1200,
    eq_mid_q_2: 0.7,
    eq_mid_gain_3: 5,
    eq_mid_freq_3: 3500,
    eq_mid_q_3: 2,
    eq_high_gain: -6,
    eq_high_freq: 8000,
  };
  expect(readEqSettings((n) => values[n])).toEqual(SETTING_A);
  delete values.eq_mid_q_3;
  expect(readEqSettings((n) => values[n])).toBeNull();
});
