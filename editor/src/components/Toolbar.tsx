import { type LinkSnapshot, NO_RESPONSE, type PortInfo } from "../link";
import { Lamp } from "./Lamp";

interface Props {
  link: LinkSnapshot;
  onInput: (id: string) => void;
  onOutput: (id: string) => void;
  /** Parameters whose current value differs from the power-up value. */
  edited: number;
  /** Parameters with a saved value. */
  saved: number;
  enabled: boolean;
  onSaveAll: () => void;
  onRecallSaved: () => void;
  onRecallFactory: () => void;
  onClearSaved: () => void;
}

/** The strip under the title bar: the MIDI link and its ports, then the actions on every saved value at once. */
export function Toolbar(p: Props) {
  const { link, edited, saved, enabled } = p;
  return (
    <div class="toolbar" role="toolbar" aria-label="Link and saved values">
      <Lamp color={link.linked ? "green" : link.status === NO_RESPONSE ? "amber" : null} />
      <span class="link-status" role="status">
        {link.status}
      </span>
      <PortSelect label="In" ports={link.inputs} value={link.inputId} onSelect={p.onInput} />
      <PortSelect label="Out" ports={link.outputs} value={link.outputId} onSelect={p.onOutput} />
      <span class="toolbar-sep" aria-hidden="true" />
      <button type="button" class="btn" disabled={!enabled || edited === 0} onClick={p.onSaveAll}>
        Save all edits ({edited})
      </button>
      <button type="button" class="btn" disabled={!enabled || edited === 0} onClick={p.onRecallSaved}>
        Recall saved
      </button>
      <button type="button" class="btn" disabled={!enabled} onClick={p.onRecallFactory}>
        Recall factory
      </button>
      <button type="button" class="btn" disabled={!enabled || saved === 0} onClick={p.onClearSaved}>
        Clear saved ({saved})
      </button>
    </div>
  );
}

interface PortSelectProps {
  label: string;
  ports: PortInfo[];
  value: string;
  onSelect: (id: string) => void;
}

function PortSelect({ label, ports, value, onSelect }: PortSelectProps) {
  return (
    <label class="port">
      {label}
      <select value={value} onChange={(e) => onSelect(e.currentTarget.value)}>
        {ports.map((p) => (
          <option key={p.id} value={p.id}>
            {p.name}
          </option>
        ))}
      </select>
    </label>
  );
}
