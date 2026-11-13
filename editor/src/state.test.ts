import { describe, expect, test } from "bun:test";
import { NO_LINK } from "./link";
import type { ParamInfo } from "./protocol";
import { initial, isOverridden, reducer, type State } from "./state";
import { toCode } from "./values";

const knob: ParamInfo = {
  id: 4,
  taper: 0,
  control: 0,
  group: 1,
  defCode: 8192,
  name: "delay_mix",
  unit: "%",
  min: 0,
  max: 100,
};
const plain: ParamInfo = { ...knob, id: 30, control: 0, name: "delay_tape_wow" };
const assignInfo: ParamInfo = {
  id: 108,
  taper: 2,
  control: 0,
  group: 6,
  defCode: 0,
  name: "knob_1_primary",
  unit: "param id (255 = none)",
  min: 0,
  max: 255,
};
// knob_1_primary drives delay_mix (id 4), its factory default.
const assign: ParamInfo = { ...assignInfo, defCode: toCode(assignInfo, 4) };

const linked = { ...NO_LINK, status: "LINKED", linked: true };
const start = (): State =>
  [knob, plain, assign].reduce(
    (s, info) => reducer(s, { type: "msg", msg: { kind: "param", info, code: info.defCode }, now: 0 }),
    reducer(initial, { type: "link", snap: linked }),
  );
const value = (s: State, id: number, code: number, now: number) =>
  reducer(s, { type: "msg", msg: { kind: "value", id, code }, now });
const edited = (s: State, id: number, code: number) => reducer(s, { type: "local", id, code, now: 0 });

describe("editor overrides of pedal controls", () => {
  test("a local edit of a knob parameter marks it", () => {
    expect(isOverridden(edited(start(), knob.id, 12000), knob.id)).toBe(true);
  });

  test("a pedal value while the edit is young keeps the mark (echo of an older throttled SET)", () => {
    const s = value(edited(start(), knob.id, 12000), knob.id, 9000, 100);
    expect(isOverridden(s, knob.id)).toBe(true);
  });

  test("a later pedal value within the slack keeps the mark", () => {
    const s = value(edited(start(), knob.id, 12000), knob.id, 12016, 1000);
    expect(isOverridden(s, knob.id)).toBe(true);
  });

  test("a later pedal value beyond the slack clears the mark (the knob took over)", () => {
    const s = value(edited(start(), knob.id, 12000), knob.id, 11500, 1000);
    expect(isOverridden(s, knob.id)).toBe(false);
  });

  test("a parameter on no switch and no primary knob slot is never marked", () => {
    expect(isOverridden(edited(start(), plain.id, 12000), plain.id)).toBe(false);
  });

  test("losing the link clears every mark", () => {
    const s = reducer(edited(start(), knob.id, 12000), { type: "link", snap: { ...linked, linked: false } });
    expect(isOverridden(s, knob.id)).toBe(false);
  });

  test("a remapped knob 1 moves the mark to its new parameter", () => {
    const s = value(start(), assign.id, toCode(assign, plain.id), 1000);
    expect(isOverridden(edited(s, knob.id, 12000), knob.id)).toBe(false);
    expect(isOverridden(edited(s, plain.id, 12000), plain.id)).toBe(true);
  });
});
