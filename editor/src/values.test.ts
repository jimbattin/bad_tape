import { expect, test } from "bun:test";
import type { ParamInfo, Taper } from "./protocol";
import { enumLabels, formatValue, fromCode, toCode } from "./values";

const param = (name: string, unit: string, min: number, max: number, taper: Taper = 0): ParamInfo => ({
  id: 0,
  taper,
  control: 0,
  group: 0,
  defCode: 0,
  name,
  unit,
  min,
  max,
});

const delayTime = param("delay_time", "ms", 10, 2000, 3);
const hallRtMid = param("hall_rt_mid", "s", 0.2, 20, 1);
const routing = param("routing", "0 wet, 1 mix, 2 dry", 0, 2, 2);

test("log taper codes", () => {
  expect(fromCode(hallRtMid, 0)).toBe(0.2);
  expect(fromCode(hallRtMid, 16383)).toBe(20);
  for (const c of [1, 5000, 16382]) expect(toCode(hallRtMid, fromCode(hallRtMid, c))).toBe(c);
});

test("audio taper codes", () => {
  expect(fromCode(delayTime, 0)).toBe(10);
  expect(fromCode(delayTime, 16383)).toBe(2000);
  expect(fromCode(delayTime, 8192)).toBeCloseTo(309, 0);
  for (const c of [1, 5000, 16382]) expect(toCode(delayTime, fromCode(delayTime, c))).toBe(c);
});

test("enum canonical code", () => {
  expect(toCode(routing, 1)).toBe(8192);
  expect(fromCode(routing, 8192)).toBe(1);
  expect(enumLabels(routing)).toEqual(["wet", "mix", "dry"]);
  expect(enumLabels(delayTime)).toBeNull();
});

test("formatValue", () => {
  expect(formatValue(delayTime, 350)).toBe("350 ms");
  expect(formatValue(hallRtMid, 2)).toBe("2.00 s");
  expect(formatValue(param("eq_high_freq", "Hz", 2000, 12000, 1), 5500)).toBe("5.50 kHz");
  expect(formatValue(param("hall_crossover", "Hz", 50, 4000, 1), 600)).toBe("600 Hz");
  expect(formatValue(param("eq_low_gain", "dB", -15, 15), 3)).toBe("+3.0 dB");
  expect(formatValue(param("eq_low_gain", "dB", -15, 15), 0)).toBe("0.0 dB");
  expect(formatValue(param("eq_low_gain", "dB", -15, 15), 0.0009)).toBe("0.0 dB"); // code 8192 of -15..15
  expect(formatValue(param("eq_low_gain", "dB", -15, 15), -0.0009)).toBe("0.0 dB");
  expect(formatValue(param("hall_echo_level_1", "dB (-60 = off)", -60, 0), -60)).toBe("off");
  expect(formatValue(param("hall_echo_level_1", "dB (-60 = off)", -60, 0), -12)).toBe("-12.0 dB");
  expect(formatValue(param("delay_tape_hiss", "dBFS RMS", -90, -40), -60)).toBe("-60.0 dBFS");
  expect(formatValue(param("delay_mode", "0 reverse, 1 off, 2 forward", 0, 2, 2), 2)).toBe("forward");
  expect(formatValue(param("canyon_rate_1", "", 0, 100), 6)).toBe("6.00");
});
