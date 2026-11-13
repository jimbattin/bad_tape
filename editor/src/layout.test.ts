import { expect, test } from "bun:test";
import { readFileSync } from "node:fs";
import { ROWS } from "./layout";

const doc = readFileSync(new URL("../../docs/MIDI.md", import.meta.url), "utf8");

/** The parameter table of docs/MIDI.md: name → control column. */
function table(): Map<string, string> {
  const rows = new Map<string, string>();
  for (const line of doc.split("\n")) {
    const c = line.split("|");
    // |Id|`name`|Unit|Min|Max|Default|Taper|Control|CC|Smoothing ms|Group|
    if (c.length === 13 && /^\d+$/.test(c[1]!)) rows.set(c[2]!.replaceAll("`", ""), c[8]!);
  }
  return rows;
}

test("every saveable parameter is laid out exactly once", () => {
  const rows = table();
  expect(rows.size).toBeGreaterThan(100);
  const saveable = [...rows].filter(([, control]) => control === "").map(([name]) => name);
  expect(ROWS.map((r) => r.name).sort()).toEqual(saveable.sort());
});

test("link marks are the Arbitration section's source/target pairs", () => {
  const pairs = [...doc.matchAll(/`(\w+)` writes `(\w+)` =/g)].map((m) => `${m[1]}>${m[2]}`);
  expect(pairs.length).toBe(11);
  const marked = ROWS.filter((r) => r.link).map((r) => `${r.link}>${r.name}`);
  expect(marked.sort()).toEqual(pairs.sort());
});
