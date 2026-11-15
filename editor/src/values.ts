// The pedal's parameters: a port of daisy/src/params.h (lookup, tapers, codes) and the readouts the editor shows.
import { CODE_MAX, type ParamInfo } from "./protocol";

/** The pedal's parameter table by id; an id stays undefined until the DUMP delivers it. */
export type ParamTable = readonly (ParamInfo | undefined)[];

/** The parameter with this name (params.h), or undefined. */
export const paramNamed = (params: ParamTable, name: string): ParamInfo | undefined =>
  params.find((p) => p?.name === name);

/** v along p's taper as 0..1 (clamped). */
function toNorm(p: ParamInfo, v: number): number {
  const c = Math.min(Math.max(v, p.min), p.max);
  if (p.taper === 1) return Math.log(c / p.min) / Math.log(p.max / p.min);
  if (p.taper === 3) return Math.log2(1 + (31 * (c - p.min)) / (p.max - p.min)) / 5;
  return (c - p.min) / (p.max - p.min);
}

/** 0..1 (clamped) along p's taper; exactly min at 0 and max at 1. */
function fromNorm(p: ParamInfo, n: number): number {
  if (!(n > 0)) return p.min;
  if (n >= 1) return p.max;
  if (p.taper === 1) return p.min * (p.max / p.min) ** n;
  if (p.taper === 3) return p.min + ((p.max - p.min) * (2 ** (5 * n) - 1)) / 31;
  if (p.taper === 2) return p.min + Math.round(n * (p.max - p.min));
  return p.min + n * (p.max - p.min);
}

/** A code as a position 0..1 (sliders). */
export const codeToNorm = (code: number): number => Math.min(code, CODE_MAX) / CODE_MAX;
/** A position 0..1 as the nearest code. */
export const normToCode = (n: number): number => Math.round(n * CODE_MAX);
export const fromCode = (p: ParamInfo, code: number): number => fromNorm(p, codeToNorm(code));
export const toCode = (p: ParamInfo, v: number): number => normToCode(toNorm(p, v));
/** An enum code as its option index (value − min). */
export const enumIndex = (p: ParamInfo, code: number): number => Math.round(fromCode(p, code) - p.min);
/** An enum option index as its code. */
export const enumCode = (p: ParamInfo, index: number): number => toCode(p, p.min + index);

/** A parameter not on a switch or footswitch: the editor can edit and save it. */
export const isSaveable = (p: ParamInfo): boolean => p.control === 0;

/** An enum's value names in value order (index = value − min), parsed from its unit "0 reverse, 1 off, …"; else null. */
export function enumLabels(p: ParamInfo): string[] | null {
  if (p.taper !== 2) return null;
  return p.unit.split(",").map((s) => s.trim().replace(/^-?\d+\s+/, ""));
}

/** v to 2, 1 or 0 decimals by magnitude. */
function fmtNumber(v: number): string {
  const a = Math.abs(v);
  return v.toFixed(a >= 100 ? 0 : a >= 10 ? 1 : 2);
}

/** v as the editor shows it: "350 ms", "2.00 s", "5.50 kHz", "+3.0 dB", "off", an enum's label. */
export function formatValue(p: ParamInfo, v: number): string {
  const labels = enumLabels(p);
  if (labels) return labels[Math.round(v - p.min)] ?? String(v);
  if (p.unit.includes("= off") && v <= p.min) return "off";
  // "dB (-60 = off)" and "dBFS RMS" print only their first word.
  if (p.unit.startsWith("dB")) {
    const tenths = Math.round(v * 10) / 10; // a code's float noise around 0 prints as 0.0, never +0.0 or -0.0
    return `${tenths > 0 ? "+" : ""}${(tenths === 0 ? 0 : tenths).toFixed(1)} ${p.unit.split(" ")[0]}`;
  }
  if (p.unit === "Hz" && v >= 1000) return `${fmtNumber(v / 1000)} kHz`;
  const text = p.unit === "%" || p.unit === "ms" ? v.toFixed(Math.abs(v) >= 10 ? 0 : 1) : fmtNumber(v);
  return p.unit ? `${text} ${p.unit}` : text;
}

/** A code as the editor shows it. */
export const formatCode = (p: ParamInfo, code: number): string => formatValue(p, fromCode(p, code));
