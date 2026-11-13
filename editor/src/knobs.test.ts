import { expect, test } from "bun:test";
import { assignGroups, NO_PARAM, SPACE, slotName, slotParams } from "./knobs";
import { TABS } from "./layout";
import { GROUP, type ParamInfo } from "./protocol";
import { toCode } from "./values";

const TAB_GROUP: Record<string, number> = {
  delay: GROUP.delay,
  hall: GROUP.hall,
  plate: GROUP.plate,
  canyon: GROUP.canyon,
  eq: GROUP.eq,
  knobs: GROUP.knobs,
};
const base = { taper: 0, control: 0, defCode: 0, unit: "", min: 0, max: 1 } as const;
const enumOf = (id: number, name: string, group: number, control: number, max: number): ParamInfo => ({
  ...base,
  id,
  name,
  group,
  control,
  taper: 2,
  max,
});

/** Every laid-out row as a parameter (control 0, group from its tab), then reverb_algo on SW3. */
const params: ParamInfo[] = TABS.flatMap((t) =>
  t.sections.flatMap((sec) => sec.rows.map((r) => ({ name: r.name, group: TAB_GROUP[t.id] ?? 0 }))),
).map(({ name, group }, id) =>
  group === GROUP.knobs ? enumOf(id, name, group, 0, 255) : { ...base, id, name, group },
);
params.push(enumOf(params.length, "reverb_algo", GROUP.global, 3, 2));
const byName = (name: string): ParamInfo => {
  const p = params.find((q) => q.name === name);
  if (!p) throw new Error(name);
  return p;
};

test("every assignment option label is unique", () => {
  const labels = assignGroups(params).flatMap((g) => g.options.map((o) => o.label));
  expect(labels.length).toBeGreaterThan(100);
  expect(new Set(labels).size).toBe(labels.length);
});

test("slotParams resolves Space to the active algorithm's and drops none and switch parameters", () => {
  const algo = byName("reverb_algo");
  const assigned: Record<string, number> = {
    [slotName(0)]: byName("plate_space").id,
    [slotName(1)]: NO_PARAM,
    [slotName(2)]: algo.id,
  };
  for (const a of [0, 1, 2]) {
    const codeOf = (id: number): number | undefined => {
      const p = params[id];
      if (!p) return undefined;
      if (p.id === algo.id) return toCode(p, a);
      return toCode(p, assigned[p.name] ?? NO_PARAM);
    };
    const slots = slotParams(params, codeOf);
    expect(slots[0]?.name).toBe(SPACE[a]);
    expect(slots[1]).toBeUndefined();
    expect(slots[2]).toBeUndefined();
  }
});
