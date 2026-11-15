import { afterEach, expect, test, vi } from "bun:test";
import { PedalLink } from "./link";

afterEach(() => {
  vi.useRealTimers();
});

test("a paced recall cancels the parameter's pending throttled send", () => {
  vi.useFakeTimers();
  const link = new PedalLink();
  const sent: number[][] = [];
  link.send = (b) => {
    sent.push(b);
  };
  link.sendThrottled("p5", [1]); // goes out at once
  link.sendThrottled("p5", [2]); // waits for the throttle
  link.sendPaced([{ key: "p5", bytes: [3] }]);
  vi.advanceTimersByTime(100);
  link.stop();
  expect(sent).toEqual([[1], [3]]);
});
