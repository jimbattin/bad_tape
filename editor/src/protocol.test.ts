import { expect, test } from "bun:test";
import {
  clear,
  droppedSince,
  dump,
  hello,
  isBootLine,
  type ParamInfo,
  parseLoad,
  parseMessage,
  save,
  savedQuery,
  set,
  storeText,
  watch,
} from "./protocol";

const HEAD = [0xf0, 0x7d, 0x62, 0x02];
const frame = (cmd: number, ...payload: number[]) => Uint8Array.from([...HEAD, cmd, ...payload, 0xf7]);
const text = (s: string) => Array.from(s, (c) => c.charCodeAt(0));

test("encoders", () => {
  expect(hello()).toEqual([...HEAD, 0x01, 0xf7]);
  expect(set(18, 8192)).toEqual([...HEAD, 0x03, 0x00, 0x12, 0x40, 0x00, 0xf7]);
  expect(set(18, 99999)).toEqual([...HEAD, 0x03, 0x00, 0x12, 0x7f, 0x7f, 0xf7]); // clamped to 16383
  expect(dump()).toEqual([...HEAD, 0x04, 0xf7]);
  expect(watch(true)).toEqual([...HEAD, 0x05, 0x01, 0xf7]);
  expect(watch(false)).toEqual([...HEAD, 0x05, 0x00, 0xf7]);
  expect(save([9, 74])).toEqual([0xf0, 0x7d, 0x62, 0x02, 0x07, 0x00, 0x09, 0x00, 0x4a, 0xf7]);
  expect(clear([9])).toEqual([...HEAD, 0x08, 0x00, 0x09, 0xf7]);
  expect(savedQuery()).toEqual([...HEAD, 0x09, 0xf7]);
});

// Shared with tests/link_test.cpp (Saved.*).
test("saved, store and saved-end vectors", () => {
  expect(parseMessage(Uint8Array.from([0xf0, 0x7d, 0x62, 0x02, 0x16, 0x00, 0x09, 0x01, 0x24, 0x34, 0xf7]))).toEqual({
    kind: "saved",
    id: 9,
    has: true,
    code: 0x1234,
  });
  expect(parseMessage(Uint8Array.from([0xf0, 0x7d, 0x62, 0x02, 0x17, 0x01, 0xf7]))).toEqual({
    kind: "store",
    status: 1,
  });
  expect(parseMessage(Uint8Array.from([0xf0, 0x7d, 0x62, 0x02, 0x18, 0x00, 0x60, 0xf7]))).toEqual({
    kind: "savedEnd",
    count: 96,
  });
});

test("PARAM carries the table entry and the current code", () => {
  const info: ParamInfo = {
    id: 6,
    taper: 3,
    control: 0,
    group: 1,
    defCode: 0x1234,
    name: "delay_time",
    unit: "ms",
    min: 10,
    max: 2000,
  };
  const m = parseMessage(frame(0x14, 0, 6, 3, 0, 17, 1, 0x24, 0x34, 0x40, 0x00, ...text("delay_time|ms|10|2000")));
  expect(m).toEqual({ kind: "param", info: info, code: 8192 });
});

test("PARAM unit may contain a bar", () => {
  const m = parseMessage(frame(0x14, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, ...text("x|a|b|0|1")));
  expect(m).toMatchObject({ kind: "param", info: { name: "x", unit: "a|b", min: 0, max: 1 } });
});

test("DUMP END count past 127", () => {
  expect(parseMessage(frame(0x15, 1, 0))).toEqual({ kind: "dumpEnd", count: 128 });
});

test("other frames and wrong lengths are null", () => {
  expect(parseMessage(Uint8Array.from([0xf0, 0x7e, 0x7f, 0x06, 0x02, 0x7d, 0xf7]))).toBeNull();
  expect(parseMessage(Uint8Array.from([0xf0, 0x7d, 0x44, 0x01, 0x12, 0x41, 0xf7]))).toBeNull();
  expect(parseMessage(frame(0x13, 1, 2))).toBeNull();
  expect(parseMessage(frame(0x16, 1, 2, 0, 0))).toBeNull();
  expect(parseMessage(frame(0x15))).toBeNull();
});

test("load line", () => {
  const line = "load avg 523 max 610 permille underruns 2 bypass off delay fwd route mix algo hall in -40 dBFS";
  expect(parseLoad(line)).toEqual({ avg: 523, max: 610, underruns: 2 });
  expect(parseLoad("ctl k 1 2 3 4 5 6")).toBeNull();
  expect(droppedSince(null, { avg: 1, max: 1, underruns: 9 })).toBe(false);
  expect(droppedSince({ avg: 1, max: 1, underruns: 2 }, { avg: 1, max: 1, underruns: 3 })).toBe(true);
  expect(droppedSince({ avg: 1, max: 1, underruns: 2 }, { avg: 1, max: 1, underruns: 2 })).toBe(false);
});

test("boot line and store text", () => {
  expect(isBootLine("bad_tape hothouse sysclk 480 MHz abc1234")).toBe(true);
  expect(isBootLine("load avg 1 max 1 permille underruns 0 ")).toBe(false);
  expect([0, 1, 2, 9].map(storeText)).toEqual(["OK", "FLASH WRITE FAILED", "BAD REQUEST", "ERROR 9"]);
});
