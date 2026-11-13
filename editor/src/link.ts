// The pedal's USB MIDI link: Web MIDI ports, SysEx framing, the HELLO keep-alive, the link status, and throttled and
// paced sends.
import { type DeviceMessage, hello, parseMessage } from "./protocol";

export interface PortInfo {
  id: string;
  name: string;
}
export interface LinkSnapshot {
  status: string;
  linked: boolean;
  inputs: PortInfo[];
  outputs: PortInfo[];
  inputId: string;
  outputId: string;
}

export const NO_RESPONSE = "NO RESPONSE";
/** The snapshot before start(): no MIDI access yet. */
export const NO_LINK: LinkSnapshot = {
  status: "CONNECTING…",
  linked: false,
  inputs: [],
  outputs: [],
  inputId: "",
  outputId: "",
};

// A flash write (SAVE, CLEAR) blocks the pedal's main loop for up to a few hundred ms.
const RX_TIMEOUT_MS = 4000;
const HELLO_PERIOD_MS = 1000;
const THROTTLE_MS = 20;
const PACE_COUNT = 16;
const PACE_MS = 20;
const MAX_FRAME = 512;

export class PedalLink {
  private access: MIDIAccess | null = null;
  private input: MIDIInput | null = null;
  private output: MIDIOutput | null = null;
  private status = NO_LINK.status;
  private lastRx = 0;
  private lastHello = 0;
  private timer: ReturnType<typeof setInterval> | undefined;
  private stopped = false;
  private msgListeners = new Set<(m: DeviceMessage) => void>();
  private snapListeners = new Set<(s: LinkSnapshot) => void>();
  private pending = new Map<string, { bytes: number[]; timer: ReturnType<typeof setTimeout> }>();
  private lastSent = new Map<string, number>();
  private paced: number[][] = [];
  private pacedTimer: ReturnType<typeof setTimeout> | undefined;
  private buf: number[] = [];
  private inputPref = "";
  private outputPref = "";

  onMessage(f: (m: DeviceMessage) => void): () => void {
    this.msgListeners.add(f);
    return () => this.msgListeners.delete(f);
  }
  onChange(f: (s: LinkSnapshot) => void): () => void {
    this.snapListeners.add(f);
    return () => this.snapListeners.delete(f);
  }

  get linked(): boolean {
    return !!this.input && !!this.output && Date.now() - this.lastRx < RX_TIMEOUT_MS;
  }

  private snapshot(): LinkSnapshot {
    return {
      status: this.status,
      linked: this.linked,
      inputs: this.access ? [...this.access.inputs.values()].map(portInfo) : [],
      outputs: this.access ? [...this.access.outputs.values()].map(portInfo) : [],
      inputId: this.input?.id ?? "",
      outputId: this.output?.id ?? "",
    };
  }

  async start(): Promise<void> {
    if (!navigator.requestMIDIAccess) {
      this.setStatus(
        window.isSecureContext ? "WEB MIDI UNAVAILABLE - USE CHROME" : "WEB MIDI NEEDS HTTPS OR LOCALHOST",
      );
      return;
    }
    let access: MIDIAccess;
    try {
      access = await navigator.requestMIDIAccess({ sysex: true });
    } catch {
      if (!this.stopped) this.setStatus("NO MIDI ACCESS - CONNECT THE PEDAL, ALLOW MIDI, RELOAD");
      return;
    }
    if (this.stopped) return;
    this.access = access;
    access.onstatechange = () => this.choosePorts();
    this.choosePorts();
    this.timer = setInterval(() => this.tick(), 250);
  }

  /** Ends the link: no more HELLOs, throttled sends or MIDI handlers. */
  stop(): void {
    this.stopped = true;
    clearInterval(this.timer);
    this.timer = undefined;
    for (const p of this.pending.values()) clearTimeout(p.timer);
    this.pending.clear();
    clearTimeout(this.pacedTimer);
    this.pacedTimer = undefined;
    this.paced = [];
    if (this.access) this.access.onstatechange = null;
    if (this.input) this.input.onmidimessage = null;
  }

  selectInput(id: string): void {
    this.inputPref = id;
    this.choosePorts();
  }
  selectOutput(id: string): void {
    this.outputPref = id;
    this.choosePorts();
  }

  send(bytes: number[]): void {
    if (this.output) {
      try {
        this.output.send(bytes);
      } catch {
        /* port vanished; statechange re-chooses */
      }
    }
  }

  /**
   * Per-key throttle, leading and trailing: a send goes out at once when the key was last sent 20 ms or more ago;
   * otherwise the latest bytes go out when that time is up. A change reaches the wire after 20 ms at most.
   */
  sendThrottled(key: string, bytes: number[]): void {
    const p = this.pending.get(key);
    if (p) {
      p.bytes = bytes;
      return;
    }
    const wait = THROTTLE_MS - (performance.now() - (this.lastSent.get(key) ?? Number.NEGATIVE_INFINITY));
    if (wait <= 0) {
      this.lastSent.set(key, performance.now());
      this.send(bytes);
      return;
    }
    const timer = setTimeout(() => {
      const q = this.pending.get(key);
      this.pending.delete(key);
      if (!q) return;
      this.lastSent.set(key, performance.now());
      this.send(q.bytes);
    }, wait);
    this.pending.set(key, { bytes, timer });
  }

  /** Sends at once, replacing any throttled send still waiting for the key (a recall must not be overwritten). */
  sendNow(key: string, bytes: number[]): void {
    const p = this.pending.get(key);
    if (p) clearTimeout(p.timer);
    this.pending.delete(key);
    this.lastSent.set(key, performance.now());
    this.send(bytes);
  }

  /**
   * Queues messages for a burst (a recall of many parameters): PACE_COUNT per PACE_MS, as the pedal's USB receive
   * ring (512 bytes) holds about 51 SETs.
   */
  sendPaced(msgs: number[][]): void {
    this.paced.push(...msgs);
    if (this.pacedTimer === undefined) this.drainPaced();
  }

  private drainPaced(): void {
    for (const m of this.paced.splice(0, PACE_COUNT)) this.send(m);
    this.pacedTimer = this.paced.length > 0 ? setTimeout(() => this.drainPaced(), PACE_MS) : undefined;
  }

  private choosePorts(): void {
    if (!this.access) return;
    const pick = <T extends MIDIPort>(ports: Iterable<T>, pref: string): T | null => {
      const list = [...ports].filter((p) => p.state !== "disconnected");
      return list.find((p) => p.id === pref) ?? list.find((p) => /daisy/i.test(p.name ?? "")) ?? list[0] ?? null;
    };
    const input = pick(this.access.inputs.values(), this.inputPref);
    const output = pick(this.access.outputs.values(), this.outputPref);
    const inChanged = input !== this.input;
    if (inChanged) {
      if (this.input) this.input.onmidimessage = null;
      this.input = input;
      this.buf = [];
      if (input) input.onmidimessage = (e) => e.data && this.rx(e.data);
    }
    const outChanged = output !== this.output;
    this.output = output;
    if (inChanged || outChanged) this.lastRx = 0;
    this.hello();
    // The ports may have changed under an unchanged status: publish either way.
    this.status = this.linkStatus();
    this.publish();
  }

  private rx(data: Uint8Array): void {
    for (const b of data) {
      if (b >= 0xf8) continue;
      if (b === 0xf0) this.buf = [b];
      else if (b === 0xf7) {
        if (this.buf.length) {
          this.buf.push(b);
          const msg = parseMessage(Uint8Array.from(this.buf));
          if (msg) this.deliver(msg);
        }
        this.buf = [];
      } else if (b & 0x80) this.buf = [];
      else if (this.buf.length) {
        this.buf.push(b);
        if (this.buf.length > MAX_FRAME) this.buf = [];
      }
    }
  }

  private deliver(msg: DeviceMessage): void {
    this.lastRx = Date.now();
    this.updateStatus();
    for (const f of this.msgListeners) f(msg);
  }

  private hello(): void {
    this.lastHello = Date.now();
    this.send(hello());
  }

  private tick(): void {
    const now = Date.now();
    if (now - this.lastRx >= RX_TIMEOUT_MS && now - this.lastHello >= HELLO_PERIOD_MS) this.hello();
    this.updateStatus();
  }

  private linkStatus(): string {
    if (!this.input || !this.output) return "NO MIDI DEVICE";
    return this.linked ? "LINKED" : NO_RESPONSE;
  }

  private updateStatus(): void {
    this.setStatus(this.linkStatus());
  }

  /** Publishes a new status; "LINKED" is the only status with `linked` set, so an unchanged status is an unchanged link. */
  private setStatus(s: string): void {
    if (s === this.status) return;
    this.status = s;
    this.publish();
  }

  private publish(): void {
    const snap = this.snapshot();
    for (const f of this.snapListeners) f(snap);
  }
}

const portInfo = (p: MIDIPort): PortInfo => ({ id: p.id, name: p.name ?? p.id });
