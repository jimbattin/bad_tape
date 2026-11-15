// The editor's view of the pedal: what its messages and the user's edits leave behind (App's reducer).
import { primaryKnob } from "./knobs";
import { type LinkSnapshot, NO_LINK } from "./link";
import { type DeviceMessage, droppedSince, type ParamInfo, type PedalLoad, parseLoad } from "./protocol";
import { fromCode, type ParamTable, paramNamed } from "./values";

const LOG_LINES = 1000;
/** A local edit this young wins over the pedal's value: VALUE echoes of older throttled SETs must not snap a drag back. */
const LOCAL_HOLD_MS = 250;
/** A pedal value within this many codes of the editor's write still counts as the editor's (rounding, knob jitter). */
const OVERRIDE_SLACK = 16;

export interface LogLine {
  t: number;
  text: string;
}

export interface SavedEntry {
  has: boolean;
  code: number;
}

export interface State {
  link: LinkSnapshot;
  /** The pedal's parameter table, as the DUMP delivered it. */
  params: ParamTable;
  /** The DUMP is complete (every parameter has arrived). */
  dumped: boolean;
  /** Current code per id. */
  current: ReadonlyMap<number, number>;
  /** The saved power-up values per saveable id. */
  saved: ReadonlyMap<number, SavedEntry>;
  /** The SAVED? answer is complete. */
  savedDone: boolean;
  /** The user's own edits, until they are LOCAL_HOLD_MS old. */
  local: ReadonlyMap<number, { code: number; t: number }>;
  /** Switch parameters and parameters on a primary knob slot the editor wrote and the pedal still holds: id → the code written. */
  overrides: ReadonlyMap<number, number>;
  load: PedalLoad | null;
  /** A dropout total grew at the last status line. */
  xrun: boolean;
  log: LogLine[];
}

export type Action =
  | { type: "link"; snap: LinkSnapshot }
  | { type: "msg"; msg: DeviceMessage; now: number }
  | { type: "local"; id: number; code: number; now: number }
  | { type: "tick"; now: number };

export const initial: State = {
  link: NO_LINK,
  params: [],
  dumped: false,
  current: new Map(),
  saved: new Map(),
  savedDone: false,
  local: new Map(),
  overrides: new Map(),
  load: null,
  xrun: false,
  log: [],
};

/**
 * The overrides once the pedal reports `code` for id: the mark goes when the pedal's value leaves the editor's write
 * (a knob took over, a switch moved, another MIDI source), but not for echoes of the editor's own throttled SETs.
 */
function released(s: State, id: number, code: number, now: number): ReadonlyMap<number, number> {
  const written = s.overrides.get(id);
  if (written === undefined) return s.overrides;
  const l = s.local.get(id);
  if (l && now - l.t < LOCAL_HOLD_MS) return s.overrides;
  if (Math.abs(code - written) <= OVERRIDE_SLACK) return s.overrides;
  const overrides = new Map(s.overrides);
  overrides.delete(id);
  return overrides;
}

function applyMessage(s: State, m: DeviceMessage, now: number): State {
  switch (m.kind) {
    case "param": {
      const params = s.params.slice();
      params[m.info.id] = m.info;
      // The first PARAM starts a dump: the table is not complete until its DUMP END.
      const dumped = m.info.id === 0 ? false : s.dumped;
      return {
        ...s,
        params,
        dumped,
        current: new Map(s.current).set(m.info.id, m.code),
        overrides: released(s, m.info.id, m.code, now),
      };
    }
    case "dumpEnd":
      return { ...s, dumped: m.count === s.params.filter((p) => p !== undefined).length };
    case "value":
      return {
        ...s,
        current: new Map(s.current).set(m.id, m.code),
        overrides: released(s, m.id, m.code, now),
      };
    case "saved":
      return { ...s, saved: new Map(s.saved).set(m.id, { has: m.has, code: m.code }) };
    case "savedEnd":
      return { ...s, savedDone: true };
    case "store":
      return s; // App awaits it
    case "log": {
      const log = [...s.log, { t: now, text: m.text }].slice(-LOG_LINES);
      const load = parseLoad(m.text);
      return load ? { ...s, log, load, xrun: droppedSince(s.load, load) } : { ...s, log };
    }
  }
}

export function reducer(s: State, a: Action): State {
  switch (a.type) {
    case "link":
      // Offline the page keeps its last values, greyed; a new link starts with a fresh DUMP and SAVED?.
      return a.snap.linked
        ? { ...s, link: a.snap }
        : { ...s, link: a.snap, dumped: false, savedDone: false, local: new Map(), overrides: new Map() };
    case "msg":
      return applyMessage(s, a.msg, a.now);
    case "local": {
      const local = new Map(s.local).set(a.id, { code: a.code, t: a.now });
      const control = s.params[a.id]?.control ?? 0;
      const held =
        (control >= 1 && control <= 3) || primaryKnob(s.params, (id) => shownCode(s, id, a.now), a.id) !== undefined;
      const overrides = held ? new Map(s.overrides).set(a.id, a.code) : s.overrides;
      return { ...s, local, overrides };
    }
    case "tick": {
      if (![...s.local.values()].some((l) => a.now - l.t >= LOCAL_HOLD_MS)) return s;
      const local = new Map([...s.local].filter(([, l]) => a.now - l.t < LOCAL_HOLD_MS));
      return { ...s, local };
    }
  }
}

/** The code to show for id: a young local edit, else the pedal's current value. */
export function shownCode(s: State, id: number, now: number): number | undefined {
  const l = s.local.get(id);
  return l && now - l.t < LOCAL_HOLD_MS ? l.code : s.current.get(id);
}

/** The code the pedal powers up with: the saved value, else the factory default. */
export function effectiveSaved(s: State, id: number): number {
  const e = s.saved.get(id);
  return e?.has ? e.code : (s.params[id]?.defCode ?? 0);
}

/** The current value differs from the power-up value. */
export function isEdited(s: State, id: number): boolean {
  const cur = s.current.get(id);
  return cur !== undefined && cur !== effectiveSaved(s, id);
}

/** The editor set this knob or switch parameter and the pedal still holds the editor's value. */
export const isOverridden = (s: State, id: number): boolean => s.overrides.has(id);

interface ShownParam {
  info: ParamInfo;
  code: number;
}

/** The parameter by name with its shown code (see shownCode); undefined until the DUMP delivers it. */
export function shownParam(s: State, name: string, now: number): ShownParam | undefined {
  const info = paramNamed(s.params, name);
  const code = info && shownCode(s, info.id, now);
  return info && code !== undefined ? { info, code } : undefined;
}

/** The parameter's value as shown (see shownCode) in its own unit. */
export function shownValue(s: State, name: string, now: number): number | undefined {
  const p = shownParam(s, name, now);
  return p && fromCode(p.info, p.code);
}

/** The reverb algorithm shown (reverb_algo: 0 plate, 1 hall, 2 canyon); undefined until the DUMP delivers it. */
export function shownAlgo(s: State, now: number): number | undefined {
  const v = shownValue(s, "reverb_algo", now);
  return v === undefined ? undefined : Math.round(v);
}
