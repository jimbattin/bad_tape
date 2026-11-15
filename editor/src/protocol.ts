// Wire formats between the editor and the pedal (daisy/src/link.h; docs/MIDI.md has the table).

const HEADER = [0xf0, 0x7d, 0x62, 0x02];

/** Host → pedal commands the editor sends (link.h; GET and DFU are not used here). */
const CMD = {
  hello: 0x01,
  set: 0x03,
  dump: 0x04,
  watch: 0x05,
  save: 0x07,
  clear: 0x08,
  savedQuery: 0x09,
} as const;
/** Pedal → host messages (link.h kLog…kSavedEnd). */
const MSG = {
  log: 0x12,
  value: 0x13,
  param: 0x14,
  dumpEnd: 0x15,
  saved: 0x16,
  store: 0x17,
  savedEnd: 0x18,
} as const;

/** The largest 14-bit code. */
export const CODE_MAX = 16383;
/** STORE's status byte. */
export const STORE = { ok: 0, failed: 1, badRequest: 2 } as const;
/** PARAM's group byte. */
export const GROUP = { global: 0, delay: 1, hall: 2, plate: 3, canyon: 4, eq: 5, knobs: 6 } as const;

/** 0 linear, 1 log, 2 enum, 3 audio. */
export type Taper = 0 | 1 | 2 | 3;

/** One table entry of the pedal (a PARAM message; params.h ParamDef). */
export interface ParamInfo {
  id: number;
  taper: Taper;
  /** 0 none, 1-3 SW1-SW3, 4 FS1, 5 FS2. */
  control: number;
  group: number;
  /** The factory default's code. */
  defCode: number;
  name: string;
  unit: string;
  min: number;
  max: number;
}

const msg = (cmd: number, ...payload: number[]): number[] => [...HEADER, cmd, ...payload, 0xf7];
/** An id, a count or a code as two 7-bit bytes, hi = v >> 7, lo = v & 0x7F. */
const u14 = (v: number): number[] => [(v >> 7) & 0x7f, v & 0x7f];
/** The two 7-bit bytes at p[i] as one value (link.h GetU14). */
const u14At = (p: Uint8Array, i: number): number => (p[i]! << 7) | p[i + 1]!;

export const hello = (): number[] => msg(CMD.hello);
/** SET: `code` is clamped to 0..16383 and rounded. */
export const set = (id: number, code: number): number[] =>
  msg(CMD.set, ...u14(id), ...u14(Math.min(Math.max(Math.round(code), 0), CODE_MAX)));
export const dump = (): number[] => msg(CMD.dump);
export const watch = (on: boolean): number[] => msg(CMD.watch, on ? 1 : 0);
const idsMsg = (cmd: number, ids: readonly number[]): number[] => msg(cmd, ...ids.flatMap((id) => u14(id)));
export const save = (ids: readonly number[]): number[] => idsMsg(CMD.save, ids);
export const clear = (ids: readonly number[]): number[] => idsMsg(CMD.clear, ids);
export const savedQuery = (): number[] => msg(CMD.savedQuery);

export type DeviceMessage =
  | { kind: "log"; text: string }
  | { kind: "value"; id: number; code: number }
  | { kind: "param"; info: ParamInfo; code: number }
  | { kind: "dumpEnd"; count: number }
  | { kind: "saved"; id: number; has: boolean; code: number }
  | { kind: "store"; status: number }
  | { kind: "savedEnd"; count: number };

const ascii = (b: Uint8Array): string => String.fromCharCode(...b);

/**
 * A complete message F0 … F7 from the pedal, or null for any other (another device's frame, the identity reply) and
 * for a wrong length.
 */
export function parseMessage(m: Uint8Array): DeviceMessage | null {
  if (m.length < 6 || m[0] !== 0xf0 || m[m.length - 1] !== 0xf7) return null;
  if (m[1] !== HEADER[1] || m[2] !== HEADER[2] || m[3] !== HEADER[3]) return null;
  const p = m.subarray(5, m.length - 1);
  switch (m[4]) {
    case MSG.log:
      return { kind: "log", text: ascii(p) };
    case MSG.value:
      return p.length === 4 ? { kind: "value", id: u14At(p, 0), code: u14At(p, 2) } : null;
    case MSG.param: {
      if (p.length < 11 || p[2]! > 3) return null;
      const parts = ascii(p.subarray(10)).split("|");
      if (parts.length < 4) return null;
      const info: ParamInfo = {
        id: u14At(p, 0),
        taper: p[2]! as Taper,
        control: p[3]!,
        // p[4]: the CC number (unused here)
        group: p[5]!,
        defCode: u14At(p, 6),
        name: parts[0]!,
        unit: parts.slice(1, -2).join("|"),
        min: Number(parts[parts.length - 2]),
        max: Number(parts[parts.length - 1]),
      };
      return { kind: "param", info, code: u14At(p, 8) };
    }
    case MSG.dumpEnd:
      return p.length === 2 ? { kind: "dumpEnd", count: u14At(p, 0) } : null;
    case MSG.saved:
      return p.length === 5 && p[2]! <= 1
        ? { kind: "saved", id: u14At(p, 0), has: p[2] === 1, code: u14At(p, 3) }
        : null;
    case MSG.store:
      return p.length === 1 ? { kind: "store", status: p[0]! } : null;
    case MSG.savedEnd:
      return p.length === 2 ? { kind: "savedEnd", count: u14At(p, 0) } : null;
    default:
      return null;
  }
}

/** The pedal's CPU load and dropouts from a status LOG line; load in per mille of the block period. */
export interface PedalLoad {
  avg: number;
  max: number;
  underruns: number;
}

const STATUS = /^load avg (\d+) max (\d+) permille underruns (\d+) /;

/** The load fields of a status LOG line, or null for any other log line. */
export function parseLoad(text: string): PedalLoad | null {
  const m = STATUS.exec(text);
  return m ? { avg: +m[1]!, max: +m[2]!, underruns: +m[3]! } : null;
}

/** True when the dropout total grew since `prev`; prev null = the first line seen. */
export const droppedSince = (prev: PedalLoad | null, next: PedalLoad): boolean =>
  prev !== null && next.underruns > prev.underruns;

/** The boot line (also the HELLO answer): the pedal restarted, so its WATCH is off. */
export const isBootLine = (text: string): boolean => /^bad_tape /.test(text);

const STORE_TEXT: Record<number, string> = {
  [STORE.ok]: "OK",
  [STORE.failed]: "FLASH WRITE FAILED",
  [STORE.badRequest]: "BAD REQUEST",
};
export const storeText = (status: number): string => STORE_TEXT[status] ?? `ERROR ${status}`;
