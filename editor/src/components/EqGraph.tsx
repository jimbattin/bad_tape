import { useMemo, useRef, useState } from "preact/hooks";
import { BANDS, type BandDef, type EqSettings, eqCurves, eqTotalDb, GAIN_MAX, Q_MAX, Q_MIN } from "../eqModel";
import { GroupBox } from "./GroupBox";

interface Props {
  /** The EQ's parameters as shown (pedal values or local edits); null until they are all known. */
  settings: EqSettings | null;
  /** Editing is possible (linked and synced). */
  enabled: boolean;
  /** A parameter (BANDS' names) set to a value in its unit. */
  onSet: (name: string, value: number) => void;
}

// The plot: log frequency 20 Hz - 20 kHz across, -24..+24 dB down, in viewBox units.
const VIEW_W = 800;
const VIEW_H = 320;
const X0 = 48;
const X1 = 788;
const Y0 = 12;
const Y1 = 292;
const F_MIN = 20;
const F_MAX = 20000;
const DB_MAX = 24;
const DB_CLAMP = 25;
const GRID_HZ: [number, string][] = [
  [20, "20"],
  [50, "50"],
  [100, "100"],
  [200, "200"],
  [500, "500"],
  [1000, "1k"],
  [2000, "2k"],
  [5000, "5k"],
  [10000, "10k"],
  [20000, "20k"],
];
const DB_LINES = [-24, -18, -12, -6, 0, 6, 12, 18, 24];
const DB_LABELS = new Set([-24, -12, 0, 12, 24]);
/** 1/24 octave from 20 Hz to 20 kHz. */
const FREQS = Array.from({ length: 241 }, (_, i) => F_MIN * (F_MAX / F_MIN) ** (i / 240));
/** A wheel notch or PageUp/PageDown: Q by a sixth of an octave's ratio. */
const Q_STEP = 2 ** (1 / 6);

const xOf = (f: number) => X0 + ((X1 - X0) * Math.log(f / F_MIN)) / Math.log(F_MAX / F_MIN);
const fOf = (x: number) => F_MIN * (F_MAX / F_MIN) ** ((x - X0) / (X1 - X0));
const yOf = (db: number) => Y0 + ((Y1 - Y0) * (DB_MAX - db)) / (2 * DB_MAX);
const dbOf = (y: number) => DB_MAX - ((y - Y0) * 2 * DB_MAX) / (Y1 - Y0);

const fmtHz = (f: number) => (f < 1000 ? `${Math.round(f)} Hz` : `${(f / 1000).toFixed(f < 10000 ? 2 : 1)} kHz`);
const fmtDb = (db: number) => `${db >= 0 ? "+" : "−"}${Math.abs(db).toFixed(1)} dB`;
const clampDb = (db: number) => Math.max(-DB_CLAMP, Math.min(DB_CLAMP, db));
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const path = (dbs: number[]) =>
  dbs.map((db, i) => `${i === 0 ? "M" : "L"}${xOf(FREQS[i]!).toFixed(1)},${yOf(clampDb(db)).toFixed(1)}`).join("");

/** A change to one band; omitted values stay. */
interface BandChange {
  gain?: number;
  freq?: number;
  q?: number;
}

/** The EQ tab's response graph: the model's curve with a draggable handle per band. */
export function EqGraph({ settings, enabled, onSet }: Props) {
  const svgRef = useRef<SVGSVGElement>(null);
  /** The band being dragged and the pointer's offset from its handle's centre at the grab (viewBox units). */
  const drag = useRef<{ band: BandDef; dx: number; dy: number } | null>(null);
  const [hoverHz, setHoverHz] = useState<number | null>(null);
  const curves = useMemo(() => (settings ? eqCurves(settings, FREQS) : null), [settings]);

  const toPlot = (e: PointerEvent): { x: number; y: number } | null => {
    const svg = svgRef.current;
    const m = svg?.getScreenCTM();
    if (!m) return null;
    const p = new DOMPoint(e.clientX, e.clientY).matrixTransform(m.inverse());
    return { x: p.x, y: p.y };
  };

  /** Sends the band's changed values, each clamped to its range and rounded to the editor's step (gain 0.1 dB;
   * frequency 1 Hz below 1 kHz, 10 Hz from 1 kHz; Q 0.01). Q is ignored for the shelves. */
  const setBand = (s: EqSettings, b: BandDef, c: BandChange) => {
    const cur = s.bands[b.id];
    if (c.gain !== undefined) {
      const g = clamp(Math.round(c.gain * 10) / 10, -GAIN_MAX, GAIN_MAX);
      if (g !== cur.gain) onSet(b.gain, g);
    }
    if (c.freq !== undefined) {
      const f = clamp(c.freq, b.minHz, b.maxHz);
      const r = clamp(f < 1000 ? Math.round(f) : Math.round(f / 10) * 10, b.minHz, b.maxHz);
      if (r !== cur.freq) onSet(b.freq, r);
    }
    if (c.q !== undefined && b.q !== null) {
      const q = clamp(Math.round(c.q * 100) / 100, Q_MIN, Q_MAX);
      if (q !== cur.q) onSet(b.q, q);
    }
  };

  const onMove = (e: PointerEvent) => {
    const p = toPlot(e);
    if (!p) return;
    const d = drag.current;
    if (d) {
      if (settings && enabled) setBand(settings, d.band, { gain: dbOf(p.y + d.dy), freq: fOf(p.x + d.dx) });
      return;
    }
    setHoverHz(p.x >= X0 && p.x <= X1 && p.y >= Y0 && p.y <= Y1 ? fOf(p.x) : null);
  };
  const onKey = (e: KeyboardEvent, s: EqSettings, b: BandDef) => {
    const cur = s.bands[b.id];
    const gainStep = e.shiftKey ? 0.1 : 0.5;
    const freqStep = 2 ** (e.shiftKey ? 1 / 24 : 1 / 6);
    if (e.key === "ArrowLeft") setBand(s, b, { freq: cur.freq / freqStep });
    else if (e.key === "ArrowRight") setBand(s, b, { freq: cur.freq * freqStep });
    else if (e.key === "ArrowUp") setBand(s, b, { gain: cur.gain + gainStep });
    else if (e.key === "ArrowDown") setBand(s, b, { gain: cur.gain - gainStep });
    else if (e.key === "PageUp" && b.q !== null) setBand(s, b, { q: cur.q * Q_STEP });
    else if (e.key === "PageDown" && b.q !== null) setBand(s, b, { q: cur.q / Q_STEP });
    else if (e.key === "Home") setBand(s, b, { gain: 0 });
    else return;
    e.preventDefault();
  };

  const badge = !settings ? "—" : !settings.enable ? "OFF" : settings.post ? "POST" : "PRE";
  const zeroY = yOf(0);
  const totalPath = curves ? path(curves.total) : "";
  const endDrag = () => {
    drag.current = null;
  };
  const shownBands = settings ? BANDS.filter((b) => settings.bands[b.id].gain !== 0) : [];

  return (
    <GroupBox
      class="eq-graph"
      title={
        <>
          Response <span class="badge">{badge}</span>
        </>
      }
    >
      <svg
        ref={svgRef}
        viewBox={`0 0 ${VIEW_W} ${VIEW_H}`}
        onPointerMove={onMove}
        onPointerLeave={() => setHoverHz(null)}
        onPointerUp={endDrag}
        onPointerCancel={endDrag}
      >
        <title>EQ response</title>
        <defs>
          <clipPath id="eq-plot">
            <rect x={X0} y={Y0} width={X1 - X0} height={Y1 - Y0} />
          </clipPath>
        </defs>
        {GRID_HZ.map(([f, label]) => (
          <g key={f}>
            <line class="eq-grid" x1={xOf(f)} x2={xOf(f)} y1={Y0} y2={Y1} />
            <text class="eq-grid-label" x={xOf(f)} y={Y1 + 18} text-anchor="middle">
              {label}
            </text>
          </g>
        ))}
        {DB_LINES.map((db) => (
          <g key={db}>
            <line class={db === 0 ? "eq-zero" : "eq-grid"} x1={X0} x2={X1} y1={yOf(db)} y2={yOf(db)} />
            {DB_LABELS.has(db) && (
              <text class="eq-grid-label" x={X0 - 8} y={yOf(db) + 4} text-anchor="end">
                {db > 0 ? `+${db}` : db < 0 ? `−${-db}` : "0"}
              </text>
            )}
          </g>
        ))}
        {!settings && (
          <text class="eq-readout" x={(X0 + X1) / 2} y={(Y0 + Y1) / 2} text-anchor="middle">
            —
          </text>
        )}
        {settings && curves && (
          <g clip-path="url(#eq-plot)">
            {shownBands.map((b) => (
              <path key={b.id} class="eq-curve-band" data-band={b.id} d={path(curves[b.id])} />
            ))}
            <path class="eq-fill-total" d={`M${X0},${zeroY}${totalPath.replace("M", "L")}L${X1},${zeroY}Z`} />
            <path class="eq-curve-total" data-off={!settings.enable} d={totalPath} />
          </g>
        )}
        {settings && !settings.enable && (
          <text class="eq-off-label" x={X0 + 10} y={Y0 + 20}>
            EQ OFF
          </text>
        )}
        {settings && hoverHz !== null && (
          <g>
            <line class="eq-cursor" x1={xOf(hoverHz)} x2={xOf(hoverHz)} y1={Y0} y2={Y1} />
            <text class="eq-readout" x={X1 - 8} y={Y0 + 20} text-anchor="end">
              {`${fmtHz(hoverHz)} · ${fmtDb(eqTotalDb(settings, hoverHz))}`}
            </text>
          </g>
        )}
        {settings &&
          BANDS.map((band) => {
            // A shelf's handle sits at its half-gain frequency, at its plateau gain.
            const { gain, freq, q } = settings.bands[band.id];
            const x = xOf(freq);
            const y = yOf(clampDb(gain));
            const valueText = `${fmtDb(gain)} at ${fmtHz(freq)}${band.q !== null ? `, Q ${q.toFixed(2)}` : ""}`;
            const interactive = enabled
              ? {
                  role: "slider" as const,
                  tabindex: 0, // an SVG attribute is case-sensitive: preact would write "tabIndex", which focuses nothing
                  "aria-label": band.label,
                  "aria-valuemin": -GAIN_MAX,
                  "aria-valuemax": GAIN_MAX,
                  "aria-valuenow": gain,
                  "aria-valuetext": valueText,
                  onPointerDown: (e: PointerEvent) => {
                    const p = toPlot(e);
                    if (!p) return;
                    (e.currentTarget as Element).setPointerCapture(e.pointerId);
                    drag.current = { band, dx: x - p.x, dy: y - p.y };
                    setHoverHz(null);
                  },
                  onWheel: (e: WheelEvent) => {
                    if (band.q === null || e.deltaY === 0) return;
                    e.preventDefault();
                    setBand(settings, band, { q: e.deltaY < 0 ? q * Q_STEP : q / Q_STEP });
                  },
                  onDblClick: () => setBand(settings, band, { gain: 0 }),
                  onKeyDown: (e: KeyboardEvent) => onKey(e, settings, band),
                }
              : { "aria-hidden": true };
            return (
              <g key={band.id} class="eq-handle" data-band={band.id} data-enabled={enabled} {...interactive}>
                <circle cx={x} cy={y} r={8} />
                <text x={x} y={y - 13} text-anchor="middle">
                  {band.letter}
                </text>
              </g>
            );
          })}
      </svg>
    </GroupBox>
  );
}
