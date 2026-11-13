interface Props {
  label: string;
  /** The mask 0..255; -1 = unknown. */
  value: number;
  disabled?: boolean;
  onChange: (mask: number) => void;
}

/** Bit 7 (the MSB) first, numbered 8..1. */
const BITS = [7, 6, 5, 4, 3, 2, 1, 0] as const;

/** An 8-bit mask as `0000 1100`. */
export const maskText = (mask: number): string => {
  const b = mask.toString(2).padStart(8, "0");
  return `${b.slice(0, 4)} ${b.slice(4)}`;
};

/** An 8-bit mask as a bevel-button group, one toggle per bit. */
export function BitMask({ label, value, disabled, onChange }: Props) {
  return (
    // biome-ignore lint/a11y/useSemanticElements: a bevel-button group laid out like Segmented, not a form fieldset
    <div class="segmented" role="group" aria-label={label}>
      {BITS.map((b) => {
        const on = value >= 0 && (value & (1 << b)) !== 0;
        return (
          <button
            key={b}
            type="button"
            aria-pressed={on}
            data-on={on}
            disabled={disabled}
            title={`Bit ${b + 1}${b === 7 ? " (MSB)" : ""}`}
            onClick={() => onChange(value ^ (1 << b))}
          >
            {b + 1}
          </button>
        );
      })}
    </div>
  );
}
