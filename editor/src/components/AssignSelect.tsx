import type { AssignGroup } from "../knobs";

interface Props {
  label: string;
  groups: AssignGroup[];
  /** The selected parameter id (knobs.ts NO_PARAM = none). */
  value: number;
  disabled: boolean;
  onSelect: (id: number) => void;
}

/** A knob slot's parameter: a menu of the assignable parameters, by tab; the unlabelled group has no heading. */
export function AssignSelect({ label, groups, value, disabled, onSelect }: Props) {
  const options = (g: AssignGroup) =>
    g.options.map((o) => (
      <option key={o.id} value={String(o.id)}>
        {o.label}
      </option>
    ));
  return (
    <select
      aria-label={label}
      value={String(value)}
      disabled={disabled}
      onChange={(e) => onSelect(Number(e.currentTarget.value))}
    >
      {groups.map((g) =>
        g.label === "" ? (
          options(g)
        ) : (
          <optgroup key={g.label} label={g.label}>
            {options(g)}
          </optgroup>
        ),
      )}
    </select>
  );
}
