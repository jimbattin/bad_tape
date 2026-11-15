import { assignLabel, isSpace, KNOBS, type Slots } from "../knobs";
import type { ParamInfo } from "../protocol";
import { isOverridden, type State, shownCode, shownParam, shownValue } from "../state";
import { codeToNorm, enumCode, enumIndex, formatCode, normToCode } from "../values";
import { CpuMeter } from "./CpuMeter";
import { GroupBox } from "./GroupBox";
import { Lamp } from "./Lamp";
import { Segmented } from "./Segmented";
import { Slider } from "./Slider";

interface Props {
  s: State;
  /** Date.now() of this render: local edits younger than the hold win over the pedal's value. */
  now: number;
  /** What each knob slot drives (knobs.ts slotParams). */
  slots: Slots;
  /** Editing is possible (linked and synced). */
  enabled: boolean;
  onEdit: (id: number, code: number) => void;
}

const ALGOS = ["Plate", "Hall", "Canyon"] as const;
const COLOR = ["BBD degrade", "Tape age", "Bitcrush"] as const;

const KNOB_TITLE = "Set from the editor: the pedal holds this value until the knob moves more than 3 %";
const SWITCH_TITLE = "Set from the editor: the pedal holds this value until the switch changes position";

function EditorBadge({ title }: { title: string }) {
  return (
    <span class="badge" title={title}>
      Editor
    </span>
  );
}

/**
 * The pedal's own controls (what you see is what you get): the three switches, FS1 and FS2 (sound on sound), the six
 * knobs with the parameters their slots drive (primary, and FS2 below), the CPU load. Editing one is a MIDI write: the
 * pedal holds it until the knob moves more than 3 % or the switch changes position.
 */
export function FrontPanel({ s, now, slots, enabled, onEdit }: Props) {
  const algo = Math.round(shownValue(s, "reverb_algo", now) ?? 1);
  const delayType = Math.round(shownValue(s, "delay_type", now) ?? 1);
  const nameOf = (p: ParamInfo): string =>
    isSpace(p.name)
      ? `${ALGOS[algo] ?? "Hall"} space`
      : p.name === "delay_color"
        ? (COLOR[delayType] ?? "Tape age")
        : assignLabel(p.name);

  const switchRow = (sw: string, label: string, name: string, options: readonly string[]) => {
    const p = shownParam(s, name, now);
    return (
      <div class="pedal-row" key={sw}>
        <span>
          <span class="pedal-id">{sw}</span> {label}
        </span>
        <Segmented
          label={`${sw} ${label}`}
          options={options}
          value={p ? enumIndex(p.info, p.code) : -1}
          disabled={!enabled || !p}
          onSelect={(i) => p && onEdit(p.info.id, enumCode(p.info, i))}
        />
        <span>{p && isOverridden(s, p.info.id) && <EditorBadge title={SWITCH_TITLE} />}</span>
      </div>
    );
  };

  const knob = (k: number) => {
    const p = slots[k - 1];
    const code = p && shownCode(s, p.id, now);
    const label = p ? nameOf(p) : "None";
    const text = p && code !== undefined ? formatCode(p, code) : "—";
    const fs2 = slots[k - 1 + KNOBS];
    const fs2Code = fs2 && shownCode(s, fs2.id, now);
    return (
      <div key={k}>
        <div class="pedal-knob-head">
          <span class="pedal-id">K{k}</span>
          <span class="pedal-knob-name">{label}</span>
          {p && isOverridden(s, p.id) && <EditorBadge title={KNOB_TITLE} />}
          <output>{text}</output>
        </div>
        <Slider
          label={`Knob ${k} ${label}`}
          norm={code !== undefined ? codeToNorm(code) : 0}
          text={text}
          ticks
          disabled={!enabled || code === undefined}
          onNorm={(n) => p && onEdit(p.id, normToCode(n))}
          onReset={() => p && onEdit(p.id, p.defCode)}
        />
        <div class="pedal-knob-fs2">
          {fs2 ? `FS2 ${nameOf(fs2)} ${fs2Code !== undefined ? formatCode(fs2, fs2Code) : "—"}` : "FS2 None"}
        </div>
      </div>
    );
  };

  const fs1 = shownParam(s, "bypass", now);
  const effectOn = fs1 !== undefined && enumIndex(fs1.info, fs1.code) === 0;
  const fs2 = shownParam(s, "delay_sos", now);
  const sosMode = shownParam(s, "delay_sos_mode", now);
  const sosArmed = sosMode !== undefined && enumIndex(sosMode.info, sosMode.code) !== 0;
  const sosOn = fs2 !== undefined && enumIndex(fs2.info, fs2.code) === 1;

  return (
    <aside class="pedal" aria-label="Pedal controls">
      <GroupBox title="Switches">
        <div class="group-body">
          {switchRow("SW1", "Delay", "delay_mode", ["Reverse", "Off", "Forward"])}
          {switchRow("SW2", "Type", "delay_type", ["BBD", "Tape", "Digital"])}
          {switchRow("SW3", "Reverb", "reverb_algo", ALGOS)}
          <div class="pedal-row">
            <span>
              <span class="pedal-id">FS1</span> Effect
            </span>
            <button
              type="button"
              class="btn"
              aria-pressed={effectOn}
              disabled={!enabled || !fs1}
              onClick={() => fs1 && onEdit(fs1.info.id, enumCode(fs1.info, effectOn ? 1 : 0))}
            >
              <Lamp color={effectOn ? "green" : null} />
              {effectOn ? "Effect on" : "Bypassed"}
            </button>
          </div>
          <div class="pedal-row">
            <span>
              <span class="pedal-id">FS2</span> Sound on sound
            </span>
            <button
              type="button"
              class="btn"
              aria-pressed={sosOn}
              disabled={!enabled || !fs2 || !sosArmed}
              onClick={() => fs2 && onEdit(fs2.info.id, enumCode(fs2.info, sosOn ? 0 : 1))}
            >
              <Lamp color={sosOn && sosArmed ? "amber" : null} />
              {sosOn ? "SOS on" : "SOS off"}
            </button>
          </div>
        </div>
      </GroupBox>
      <GroupBox title="Knobs">
        <div class="group-body">{Array.from({ length: KNOBS }, (_, i) => knob(i + 1))}</div>
      </GroupBox>
      <GroupBox title="CPU load">
        <div class="group-body">
          <CpuMeter load={s.load} xrun={s.xrun} />
        </div>
      </GroupBox>
    </aside>
  );
}
