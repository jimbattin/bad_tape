import type { ComponentChildren } from "preact";
import type { ParamInfo } from "../protocol";
import type { SavedEntry } from "../state";
import { codeToNorm, enumCode, enumIndex, enumLabels, formatCode, normToCode } from "../values";
import { BitMask, maskText } from "./BitMask";
import { Lamp } from "./Lamp";
import { Segmented } from "./Segmented";
import { Slider } from "./Slider";

interface Props {
  info: ParamInfo;
  label: string;
  /** The Space macro that writes this parameter, if any. */
  link?: string;
  /** Replaces the slider or choice buttons. */
  control?: ComponentChildren;
  /** Replaces formatCode for the current, factory and saved values. */
  format?: (code: number) => string;
  /** An 8-bit mask: one toggle per bit (BitMask), values shown as bits. */
  mask?: boolean;
  /** The knob (1-6) whose primary drives this parameter, if any. */
  knob?: number;
  /** The current code; undefined until the pedal has reported it. */
  code: number | undefined;
  /** The code the pedal powers up with: the saved value, else the factory default. */
  powerUp: number;
  saved: SavedEntry | undefined;
  enabled: boolean;
  onEdit: (code: number) => void;
  onSave: () => void;
  onClear: () => void;
  onRecallSaved: () => void;
  onRecallFactory: () => void;
}

const LINK_TITLE =
  "The reverb's Space macro writes this when Space changes; a value set here or saved holds until Space moves more than 3 %";
const knobTitle = (k: number): string =>
  `On knob ${k} (what you see is what you get): a value set here holds until the knob moves more than 3 %; at power-up the knob sets it`;

const capitalise = (s: string): string => s.charAt(0).toUpperCase() + s.slice(1);

/** One saveable parameter on one line: lamp, name, control, current, factory and saved values, memory buttons. */
export function ParamRow(p: Props) {
  const { info, code, saved, enabled, knob } = p;
  const onKnob = knob !== undefined;
  const edited = code !== undefined && code !== p.powerUp;
  const text = p.format ?? (p.mask ? (c: number) => maskText(enumIndex(info, c)) : (c: number) => formatCode(info, c));
  const labels = enumLabels(info);
  const readout = code === undefined ? "—" : text(code);
  const factory = text(info.defCode);
  const savedText = saved?.has ? text(saved.code) : "—";
  const off = !enabled || code === undefined;
  return (
    <div class="row">
      <Lamp color={onKnob ? null : edited ? "amber" : "green"} />
      <span class="row-label" title={info.name}>
        <span class="row-name">{p.label}</span>
        {onKnob ? (
          <span class="badge" title={knobTitle(knob)}>
            K{knob}
          </span>
        ) : (
          p.link && (
            <span class="badge" title={LINK_TITLE}>
              Space
            </span>
          )
        )}
      </span>
      <span class="row-control">
        {p.control ??
          (p.mask ? (
            <BitMask
              label={p.label}
              value={code === undefined ? -1 : enumIndex(info, code)}
              disabled={off}
              onChange={(m) => p.onEdit(enumCode(info, m))}
            />
          ) : labels ? (
            <Segmented
              label={p.label}
              options={labels.map(capitalise)}
              value={code === undefined ? -1 : enumIndex(info, code)}
              disabled={off}
              onSelect={(i) => p.onEdit(enumCode(info, i))}
            />
          ) : (
            <Slider
              label={p.label}
              norm={codeToNorm(code ?? 0)}
              text={readout}
              disabled={off}
              onNorm={(n) => p.onEdit(normToCode(n))}
              onReset={() => p.onEdit(info.defCode)}
            />
          ))}
      </span>
      <output class="row-value" title={`Factory ${factory} · Saved ${savedText}`}>
        {readout}
      </output>
      <span class="row-factory" title="Factory default">
        {factory}
      </span>
      <span class="row-saved" title="Saved power-up value">
        {savedText}
      </span>
      <span class="row-buttons">
        <button
          type="button"
          class="btn small"
          title="Save the current value as the power-up value"
          disabled={!enabled || !edited || onKnob}
          onClick={p.onSave}
        >
          Save
        </button>
        <button
          type="button"
          class="btn small"
          title="Back to the power-up value"
          disabled={!enabled || !edited || onKnob}
          onClick={p.onRecallSaved}
        >
          ↺ Saved
        </button>
        <button
          type="button"
          class="btn small"
          title="Back to the factory default"
          disabled={!enabled || code === info.defCode}
          onClick={p.onRecallFactory}
        >
          ↺ Factory
        </button>
        <button
          type="button"
          class="btn small"
          title="Forget the saved value: power up with the factory default"
          disabled={!enabled || !saved?.has}
          onClick={p.onClear}
        >
          Clear
        </button>
      </span>
    </div>
  );
}

/** The column headings above a tab's rows (the same grid as ParamRow). */
export function ParamRowsHead() {
  return (
    <div class="rows-head" aria-hidden="true">
      <span />
      <span>Parameter</span>
      <span />
      <span class="row-value">Value</span>
      <span class="row-factory">Factory</span>
      <span class="row-saved">Saved</span>
      <span />
    </div>
  );
}
