import type { ComponentChildren } from "preact";
import { GroupBox } from "./GroupBox";
import { Lamp } from "./Lamp";

interface Props {
  title: string;
  /** The section's condition (e.g. the delay type it belongs to) holds on the pedal. Undefined: always in use. */
  active?: boolean;
  children: ComponentChildren;
}

/** An etched group box of parameter rows; an inactive section says so but stays editable. */
export function ParamSection({ title, active, children }: Props) {
  return (
    <GroupBox
      title={
        <>
          {active !== undefined && <Lamp color={active ? "green" : null} />}
          {title}
          {active === false && (
            <>
              <span class="not-selected" aria-hidden="true">
                Not selected on pedal · still editable
              </span>
              <span class="visually-hidden"> (not selected on the pedal)</span>
            </>
          )}
        </>
      }
    >
      {children}
    </GroupBox>
  );
}
