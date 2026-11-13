import { useMemo, useRef, useState } from "preact/hooks";
import { AssignSelect } from "./components/AssignSelect";
import { EqGraph } from "./components/EqGraph";
import { FrontPanel } from "./components/FrontPanel";
import { LogView } from "./components/LogView";
import { ParamRow, ParamRowsHead } from "./components/ParamRow";
import { ParamSection } from "./components/ParamSection";
import { NO_STATUS, type Status, StatusLine } from "./components/StatusLine";
import { type NavTab, TabNav } from "./components/TabNav";
import { Toolbar } from "./components/Toolbar";
import { readEqSettings } from "./eqModel";
import { assignGroups, assignLabel, assignValue, primaryIn, SLOTS, slotName, slotParams } from "./knobs";
import { ALGO_TABS, type RowDef, type SectionDef, TABS, type TabId, unplacedRows } from "./layout";
import { clear, GROUP, type ParamInfo, save, set } from "./protocol";
import { effectiveSaved, isEdited, shownCode, shownValue } from "./state";
import { usePedal } from "./usePedal";
import { isSaveable, toCode } from "./values";

const errorText = (e: unknown): string => (e instanceof Error ? e.message : String(e));
const valueCount = (n: number): string => `${n} value${n === 1 ? "" : "s"}`;

export function App() {
  const { s, link, dispatch, store } = usePedal();
  const [tab, setTab] = useState<TabId>("delay");
  const [busy, setBusy] = useState<string | null>(null);
  const busyRef = useRef(false);
  const [status, setStatus] = useState<Status>(NO_STATUS);

  const now = Date.now();
  const linked = s.link.linked;
  const ready = linked && s.dumped && s.savedDone && busy === null;
  const saveable = useMemo(() => s.params.filter((p): p is ParamInfo => p !== undefined && isSaveable(p)), [s.params]);
  const slots = slotParams(s.params, (id) => shownCode(s, id, now));
  const groups = useMemo(() => assignGroups(s.params), [s.params]);
  // A parameter on a primary slot is the knob's: the knob sets it at power-up, so its value is not stored.
  const stored = saveable.filter((p) => primaryIn(slots, p.id) === undefined);

  // One store operation at a time.
  const act = async (label: string, fn: () => Promise<string>): Promise<void> => {
    if (busyRef.current) return;
    busyRef.current = true;
    setStatus(NO_STATUS);
    setBusy(label);
    try {
      setStatus({ text: await fn(), error: false });
    } catch (e) {
      setStatus({ text: errorText(e), error: true });
    } finally {
      busyRef.current = false;
      setBusy(null);
    }
  };
  const saveIds = (ids: number[], what: string) =>
    act("SAVING…", async () => {
      await store(save(ids));
      return `Saved ${what}`;
    });
  const clearIds = (ids: number[], what: string) =>
    act("CLEARING…", async () => {
      await store(clear(ids));
      return `Cleared ${what}`;
    });

  const edited = stored.filter((p) => isEdited(s, p.id));
  const withSaved = saveable.filter((p) => s.saved.get(p.id)?.has);

  const onEdit = (id: number, c: number) => {
    dispatch({ type: "local", id, code: c, now: Date.now() });
    link.sendThrottled(`p${id}`, set(id, c));
  };
  const recall = (id: number, c: number) => {
    dispatch({ type: "local", id, code: c, now: Date.now() });
    link.sendNow(`p${id}`, set(id, c));
  };
  const recallMany = (targets: { id: number; code: number }[]) => {
    const t = Date.now();
    for (const x of targets) dispatch({ type: "local", id: x.id, code: x.code, now: t });
    link.sendPaced(targets.map((x) => set(x.id, x.code)));
  };

  const onSaveAll = () =>
    void saveIds(
      edited.map((p) => p.id),
      valueCount(edited.length),
    );
  const onRecallSaved = () => recallMany(edited.map((p) => ({ id: p.id, code: effectiveSaved(s, p.id) })));
  const onRecallFactory = () => {
    if (!window.confirm("Set every parameter to its factory default? Saved values are kept.")) return;
    recallMany(stored.filter((p) => s.current.get(p.id) !== p.defCode).map((p) => ({ id: p.id, code: p.defCode })));
  };
  const onClearSaved = () => {
    if (!window.confirm("Forget every saved value? The pedal powers up with its factory defaults.")) return;
    void clearIds(
      withSaved.map((p) => p.id),
      valueCount(withSaved.length),
    );
  };

  const others = useMemo(() => unplacedRows(saveable), [saveable]);

  const infoByName = useMemo(() => new Map(saveable.map((p) => [p.name, p])), [saveable]);
  const assignments = Array.from({ length: SLOTS }, (_, i) => infoByName.get(slotName(i))).filter(
    (p): p is ParamInfo => p !== undefined,
  );
  const layoutChanged = assignments.some((p) => shownCode(s, p.id, now) !== p.defCode);
  const onFactoryLayout = () => recallMany(assignments.map((p) => ({ id: p.id, code: p.defCode })));
  const renderRow = (c: RowDef, sectionTitle: string) => {
    const info = infoByName.get(c.name);
    if (!info) return null;
    const powerUp = effectiveSaved(s, info.id);
    const code = shownCode(s, info.id, now);
    const slotProps =
      info.group === GROUP.knobs
        ? {
            control: (
              <AssignSelect
                label={`${sectionTitle} ${c.label}`}
                groups={groups}
                value={code === undefined ? -1 : assignValue(s.params, info, code)}
                disabled={!ready || code === undefined}
                onSelect={(id) => onEdit(info.id, toCode(info, id))}
              />
            ),
            format: (v: number) => assignLabel(s.params[assignValue(s.params, info, v)]?.name ?? ""),
          }
        : { knob: primaryIn(slots, info.id), link: slots.some((p) => p?.id === info.id) ? undefined : c.link };
    return (
      <ParamRow
        key={c.name}
        info={info}
        label={c.label}
        {...slotProps}
        mask={c.mask}
        code={code}
        powerUp={powerUp}
        saved={s.saved.get(info.id)}
        enabled={ready}
        onEdit={(v) => onEdit(info.id, v)}
        onSave={() => void saveIds([info.id], info.name)}
        onClear={() => void clearIds([info.id], info.name)}
        onRecallSaved={() => recall(info.id, powerUp)}
        onRecallFactory={() => recall(info.id, info.defCode)}
      />
    );
  };
  const renderSection = (sec: SectionDef) => {
    const w = sec.activeWhen;
    const active = w ? shownValue(s, w.name, now) === w.value : undefined;
    return (
      <ParamSection key={sec.title} title={sec.title} active={s.dumped ? active : undefined}>
        {sec.rows.map((r) => renderRow(r, sec.title))}
      </ParamSection>
    );
  };
  const eqSettings = readEqSettings((n) => shownValue(s, n, now));
  const onEqSet = (name: string, v: number) => {
    const info = infoByName.get(name);
    if (info) onEdit(info.id, toCode(info, v));
  };

  const algo = Math.round(shownValue(s, "reverb_algo", now) ?? -1);
  /** A tab's lamp: its section is in use on the pedal; undefined: no lamp. */
  const tabLit = (id: TabId): boolean | undefined => {
    if (!s.dumped) return undefined;
    switch (id) {
      case "delay":
        return shownValue(s, "delay_mode", now) !== 1;
      case "eq":
        return shownValue(s, "eq_enable", now) === 1;
      case "knobs":
      case "log":
        return undefined;
      case "plate":
      case "hall":
      case "canyon":
        return ALGO_TABS[algo] === id;
    }
  };
  const tabs: NavTab<TabId>[] = TABS.map((t) => ({ id: t.id, label: t.label, lit: tabLit(t.id) }));
  const current = TABS.find((t) => t.id === tab);
  const extra = others.get(tab);

  return (
    <main class="window">
      <header class="titlebar">
        <h1>Bad Tape</h1>
      </header>
      <Toolbar
        link={s.link}
        onInput={(id) => link.selectInput(id)}
        onOutput={(id) => link.selectOutput(id)}
        edited={edited.length}
        saved={withSaved.length}
        enabled={ready}
        onSaveAll={onSaveAll}
        onRecallSaved={onRecallSaved}
        onRecallFactory={onRecallFactory}
        onClearSaved={onClearSaved}
      />
      <div class="window-body">
        <FrontPanel s={s} now={now} slots={slots} enabled={ready} onEdit={onEdit} />
        <div class="workspace">
          <TabNav tabs={tabs} active={tab} onSelect={setTab} />
          <div class="tabpanel" role="tabpanel" id="tabpanel" aria-labelledby={`tab-${tab}`}>
            {tab === "log" ? (
              <LogView lines={s.log} />
            ) : !s.dumped && saveable.length === 0 ? (
              <p class="tab-empty">{linked ? "Reading the pedal…" : "Waiting for the pedal."}</p>
            ) : (
              <>
                {tab === "eq" && <EqGraph settings={eqSettings} enabled={ready} onSet={onEqSet} />}
                {tab === "knobs" && (
                  <div class="tab-note">
                    <p>
                      Hold FS2 and turn a knob to set its FS2 parameter. That knob's primary keeps its value until the
                      knob moves more than 3 % again without FS2. Switches and footswitches are fixed. Save stores the
                      layout as the power-up layout.
                    </p>
                    <button type="button" class="btn" disabled={!ready || !layoutChanged} onClick={onFactoryLayout}>
                      Factory layout
                    </button>
                  </div>
                )}
                <ParamRowsHead />
                {current?.sections.map(renderSection)}
                {extra && <ParamSection title="Other">{extra.map((r) => renderRow(r, "Other"))}</ParamSection>}
              </>
            )}
          </div>
        </div>
      </div>
      <StatusLine text={busy ?? status.text} error={busy === null && status.error} />
    </main>
  );
}
