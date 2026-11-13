// The knob slots (daisy/src/params.h knob_N_primary / knob_N_secondary): each holds the id of the parameter knob N
// drives without FS2 (primary) or with FS2 held (secondary). A port of params.h IsAssignable and ResolveSlot.
import { ALGO_TABS, TABS } from "./layout";
import { GROUP, type ParamInfo } from "./protocol";
import { fromCode, type ParamTable, paramNamed } from "./values";

export const NO_PARAM = 255;
export const KNOBS = 6;
/** Slot s: 0..KNOBS-1 the primary of knob s+1, KNOBS..2*KNOBS-1 the FS2 parameter of knob s-KNOBS+1. */
export const SLOTS = 2 * KNOBS;
export const slotName = (s: number): string =>
  s < KNOBS ? `knob_${s + 1}_primary` : `knob_${s - KNOBS + 1}_secondary`;
/** The Space macros; index = reverb_algo. */
export const SPACE: readonly string[] = ALGO_TABS.map((t) => `${t}_space`);
type CodeOf = (id: number) => number | undefined;

const isAssignable = (p: ParamInfo): boolean => p.control === 0 && p.group !== GROUP.knobs;
/** One of the three `*_space` macros: on a slot it means the active algorithm's Space. */
export const isSpace = (name: string): boolean => SPACE.includes(name);

/** The assignment's parameter id (a select value): NO_PARAM for none or anything not assignable; any *_space id → hall_space's id. */
export function assignValue(params: ParamTable, info: ParamInfo, code: number): number {
  const v = Math.round(fromCode(info, code));
  const p = params[v];
  if (!p || !isAssignable(p)) return NO_PARAM;
  return isSpace(p.name) ? (paramNamed(params, "hall_space")?.id ?? v) : v;
}

/** Each slot's driven parameter (index = slot); undefined = none or not yet dumped. */
export type Slots = readonly (ParamInfo | undefined)[];

/** Each slot's driven parameter, as params.h ResolveSlot (Space → SPACE[reverb_algo]). */
export function slotParams(params: ParamTable, codeOf: CodeOf): Slots {
  const algoInfo = paramNamed(params, "reverb_algo");
  const algoCode = algoInfo && codeOf(algoInfo.id);
  const algo = algoInfo && algoCode !== undefined ? Math.round(fromCode(algoInfo, algoCode)) : 1;
  const space = paramNamed(params, SPACE[algo] ?? "hall_space");
  return Array.from({ length: SLOTS }, (_, s) => {
    const info = paramNamed(params, slotName(s));
    const code = info && codeOf(info.id);
    if (!info || code === undefined) return undefined;
    const p = params[assignValue(params, info, code)];
    return p && isSpace(p.name) ? space : p;
  });
}

/** In slotParams' result: the knob (1-6) whose primary drives id, the lowest knob if several; undefined if none. */
export function primaryIn(slots: Slots, id: number): number | undefined {
  const k = slots.slice(0, KNOBS).findIndex((p) => p?.id === id);
  return k < 0 ? undefined : k + 1;
}

/** The knob (1-6) whose primary drives id, the lowest knob if several; undefined if none. */
export const primaryKnob = (params: ParamTable, codeOf: CodeOf, id: number): number | undefined =>
  primaryIn(slotParams(params, codeOf), id);

/** Each laid-out row's place: name → "<section> · <row>" on the Delay tab, "<tab> <section> · <row>" elsewhere. */
const PLACE = new Map(
  TABS.flatMap((t) =>
    t.sections.flatMap((sec) =>
      sec.rows.map((r) => {
        const place = `${sec.title} · ${r.label}`;
        return [r.name, t.id === "delay" ? place : `${t.label} ${place}`] as const;
      }),
    ),
  ),
);

/** "None", "Space (active reverb)", else "<section> · <row>" on the Delay tab and "<tab> <section> · <row>" elsewhere (layout.ts TABS). */
export function assignLabel(name: string): string {
  if (name === "") return "None";
  if (isSpace(name)) return "Space (active reverb)";
  return PLACE.get(name) ?? name;
}

export interface AssignGroup {
  label: string;
  options: { id: number; label: string }[];
}

/** First group "" = [None, Space (active reverb)]; then one group per layout tab with assignable rows, in layout order, of those rows except *_space, labelled by assignLabel. */
export function assignGroups(params: ParamTable): AssignGroup[] {
  const hallSpace = paramNamed(params, "hall_space");
  const first: AssignGroup = {
    label: "",
    options: [
      { id: NO_PARAM, label: assignLabel("") },
      ...(hallSpace ? [{ id: hallSpace.id, label: assignLabel(hallSpace.name) }] : []),
    ],
  };
  const tabs = TABS.map((t) => ({
    label: t.label,
    options: t.sections.flatMap((sec) =>
      sec.rows.flatMap((r) => {
        const p = paramNamed(params, r.name);
        return p && isAssignable(p) && !isSpace(p.name) ? [{ id: p.id, label: assignLabel(p.name) }] : [];
      }),
    ),
  }));
  return [first, ...tabs.filter((g) => g.options.length > 0)];
}
