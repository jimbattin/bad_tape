// Where each saveable parameter sits in the editor: tabs of sections of rows, after the pedal's parameter table
// (daisy/src/params.h; layout.test.ts checks every saveable parameter of docs/MIDI.md appears exactly once). A
// saveable parameter missing here still shows, in the "Other" section of its group's tab (unplacedRows).
import { GROUP, type ParamInfo } from "./protocol";

export type TabId = "delay" | "plate" | "hall" | "canyon" | "eq" | "knobs" | "log";

export interface RowDef {
  /** The parameter's name (params.h). */
  name: string;
  label: string;
  /** The reverb's Space macro (a `*_space` parameter) that writes this one when Space changes. */
  link?: string;
  /** An 8-bit mask: one toggle button per bit (BitMask). */
  mask?: boolean;
}

export interface SectionDef {
  title: string;
  rows: RowDef[];
  /** The section is in use while this enum parameter has this value; otherwise it is dimmed. */
  activeWhen?: { name: string; value: number };
}

interface TabDef {
  id: TabId;
  label: string;
  sections: SectionDef[];
}

const row = (name: string, label: string, link?: string): RowDef => ({ name, label, link });
/** `prefix` 1..n as rows labelled "<label> i". */
const numbered = (prefix: string, label: string, n: number): RowDef[] =>
  Array.from({ length: n }, (_, i) => row(`${prefix}_${i + 1}`, `${label} ${i + 1}`));

const EQ_ON = { name: "eq_enable", value: 1 };

/** The reverb tabs in `reverb_algo` order (params.h: 0 plate, 1 hall, 2 canyon). */
export const ALGO_TABS = ["plate", "hall", "canyon"] as const satisfies readonly TabId[];

export const TABS: TabDef[] = [
  {
    id: "delay",
    label: "Delay",
    sections: [
      {
        title: "Delay",
        rows: [
          row("delay_mix", "Mix"),
          row("delay_time", "Time"),
          row("delay_feedback", "Feedback"),
          row("delay_color", "Color"),
        ],
      },
      { title: "Reverb", rows: [row("reverb_mix", "Mix"), row("routing", "Input")] },
      {
        title: "Sound on sound",
        rows: [
          row("delay_sos_mode", "Mode"),
          row("delay_sos_length", "Length"),
          row("delay_sos_ratio", "501 head"),
          row("delay_sos_feedback", "Overdub keep"),
          row("delay_sos_level", "Level"),
        ],
      },
      {
        title: "Bucket brigade",
        activeWhen: { name: "delay_type", value: 0 },
        rows: [
          row("delay_bbd_scope", "Colors"),
          row("delay_bbd_stages", "Stages"),
          row("delay_bbd_tone", "Tone"),
          row("delay_bbd_drive", "Drive"),
          row("delay_bbd_noise", "Noise"),
          row("delay_bbd_clock", "Clock"),
          row("delay_bbd_mod_rate", "Mod rate"),
          row("delay_bbd_mod_depth", "Mod depth"),
        ],
      },
      {
        title: "Tape",
        activeWhen: { name: "delay_type", value: 1 },
        rows: [
          row("delay_tape_scope", "Colors"),
          row("delay_tape_wow", "Wow"),
          row("delay_tape_flutter", "Flutter"),
          row("delay_tape_drive", "Drive"),
          row("delay_tape_tone", "Tone"),
          row("delay_tape_bump", "Head bump"),
          row("delay_tape_hiss", "Hiss"),
          row("delay_tape_dropouts", "Dropouts"),
        ],
      },
      {
        title: "Digital",
        activeWhen: { name: "delay_type", value: 2 },
        rows: [
          row("delay_crush_scope", "Colors"),
          row("delay_crush_drive", "Drive"),
          row("delay_crush_rate", "Hold rate"),
          row("delay_crush_aa", "Anti-alias"),
          row("delay_crush_tone", "Tone"),
          row("delay_crush_open", "Open filters"),
          row("delay_crush_clock", "Clock follows time"),
          row("delay_crush_wrap", "Wrap"),
          row("delay_crush_bits", "Bit invert"),
          { name: "delay_crush_mask", label: "Invert mask", mask: true },
        ],
      },
    ],
  },
  {
    id: "plate",
    label: "Plate",
    sections: [
      {
        title: "Decay",
        rows: [
          row("plate_mid_decay", "Mid decay", "plate_space"),
          row("plate_lf_decay", "LF decay", "plate_space"),
          row("plate_crossover", "Crossover"),
          row("plate_treble_decay", "Treble decay"),
          row("plate_hf_bandwidth", "HF bandwidth"),
        ],
      },
      {
        title: "Space",
        rows: [
          row("plate_space", "Macro"),
          row("plate_size", "Size", "plate_space"),
          row("plate_depth", "Depth", "plate_space"),
          row("plate_predelay", "Predelay"),
          row("plate_diffusion", "Diffusion"),
          row("plate_definition", "Definition"),
          row("plate_chorus", "Chorus"),
          row("plate_decay_opt", "Decay opt"),
        ],
      },
      {
        title: "Stop",
        rows: [
          row("plate_stop_decay", "Stop decay"),
          row("plate_lf_stop_decay", "LF stop decay"),
          row("plate_mid_stop_decay", "Mid stop decay"),
          row("plate_stop_delay", "Stop delay"),
        ],
      },
    ],
  },
  {
    id: "hall",
    label: "Hall",
    sections: [
      {
        title: "Decay",
        rows: [
          row("hall_rt_mid", "RT mid", "hall_space"),
          row("hall_bass_mult", "Bass mult"),
          row("hall_crossover", "Crossover"),
          row("hall_rt_hf_cut", "RT HF cut"),
          row("hall_hf_cutoff", "HF cutoff"),
        ],
      },
      {
        title: "Space",
        rows: [
          row("hall_space", "Macro"),
          row("hall_size", "Size", "hall_space"),
          row("hall_shape", "Shape", "hall_space"),
          row("hall_spread", "Spread", "hall_space"),
          row("hall_diffusion", "Diffusion"),
          row("hall_predelay", "Predelay"),
        ],
      },
      {
        title: "Motion & output",
        rows: [row("hall_spin", "Spin"), row("hall_wander", "Wander"), row("hall_output_shelf", "Output shelf")],
      },
      {
        title: "Echoes",
        rows: [...numbered("hall_echo_level", "Level", 4), ...numbered("hall_echo_delay", "Delay", 4)],
      },
    ],
  },
  {
    id: "canyon",
    label: "Canyon",
    sections: [
      {
        title: "Master",
        rows: [
          row("canyon_space", "Space macro"),
          row("canyon_feedback", "Feedback", "canyon_space"),
          row("canyon_feedback_invert", "Invert"),
          row("canyon_master_delay", "Delay", "canyon_space"),
          row("canyon_master_rate", "Rate"),
          row("canyon_master_depth", "Depth", "canyon_space"),
          row("canyon_glide_rate", "Glide rate"),
          row("canyon_delay_gliding", "Gliding"),
        ],
      },
      {
        title: "Levels",
        rows: [row("canyon_mix", "Mix"), row("canyon_in_trim", "In trim"), row("canyon_out_trim", "Out trim")],
      },
      {
        title: "Lines",
        rows: [
          ...numbered("canyon_delay", "Delay", 6),
          ...numbered("canyon_rate", "Rate", 6),
          ...numbered("canyon_depth", "Depth", 6),
        ],
      },
    ],
  },
  {
    id: "eq",
    label: "EQ",
    sections: [
      { title: "Stage", rows: [row("eq_enable", "EQ"), row("eq_position", "Position")] },
      { title: "Low shelf", activeWhen: EQ_ON, rows: [row("eq_low_gain", "Gain"), row("eq_low_freq", "Freq")] },
      ...[1, 2, 3].map((n) => ({
        title: `Mid ${n}`,
        activeWhen: EQ_ON,
        rows: [row(`eq_mid_gain_${n}`, "Gain"), row(`eq_mid_freq_${n}`, "Freq"), row(`eq_mid_q_${n}`, "Q")],
      })),
      { title: "High shelf", activeWhen: EQ_ON, rows: [row("eq_high_gain", "Gain"), row("eq_high_freq", "Freq")] },
    ],
  },
  {
    id: "knobs",
    label: "Knobs",
    sections: [1, 2, 3, 4, 5, 6].map((k) => ({
      title: `Knob ${k}`,
      rows: [row(`knob_${k}_primary`, "Primary"), row(`knob_${k}_secondary`, "FS2")],
    })),
  },
  { id: "log", label: "Log", sections: [] },
];

/** Every laid-out row, in tab order. */
export const ROWS: readonly RowDef[] = TABS.flatMap((t) => t.sections.flatMap((s) => s.rows));
const PLACED = new Set(ROWS.map((r) => r.name));

/** The tab of a group's parameters that the layout does not place. */
const TAB_OF_GROUP: Record<number, TabId> = {
  [GROUP.global]: "delay",
  [GROUP.delay]: "delay",
  [GROUP.hall]: "hall",
  [GROUP.plate]: "plate",
  [GROUP.canyon]: "canyon",
  [GROUP.eq]: "eq",
  [GROUP.knobs]: "knobs",
};

/** Saveable parameters the layout does not place, by their group's tab, labelled with their name. */
export function unplacedRows(saveable: readonly ParamInfo[]): Map<TabId, RowDef[]> {
  const byTab = new Map<TabId, RowDef[]>();
  for (const p of saveable) {
    if (PLACED.has(p.name)) continue;
    const tab = TAB_OF_GROUP[p.group] ?? "delay";
    const rows = byTab.get(tab) ?? [];
    rows.push({ name: p.name, label: p.name });
    byTab.set(tab, rows);
  }
  return byTab;
}
