// The pedal link's lifecycle: the reducer state it feeds, the sync requests on (re)link and their retries, and the
// SAVE/CLEAR round trip.
import { useEffect, useReducer, useRef, useState } from "preact/hooks";
import { PedalLink } from "./link";
import { dump, isBootLine, STORE, savedQuery, storeText, watch } from "./protocol";
import { type Action, initial, reducer, type State } from "./state";

/** A DUMP or SAVED? that has not completed this long after its request is sent again. */
const RETRY_MS = 3000;
/** A boot line this soon after a sync request does not start another (the first line of a new link is one). */
const SYNC_GAP_MS = 500;
/** SAVE/CLEAR take one flash write: up to a few hundred ms. */
const STORE_TIMEOUT_MS = 5000;

interface SyncTimes {
  dump: number;
  saved: number;
}

/** Asks a (re)linked pedal for its table, its saved values and its VALUE stream. */
function requestSync(link: PedalLink, last: SyncTimes): void {
  last.dump = last.saved = Date.now();
  link.send(dump());
  link.send(savedQuery());
  link.send(watch(true));
}

export interface Pedal {
  s: State;
  link: PedalLink;
  dispatch: (a: Action) => void;
  /** Sends a SAVE or CLEAR; resolves on its STORE, rejects on a non-zero status or after STORE_TIMEOUT_MS. */
  store: (bytes: number[]) => Promise<void>;
}

export function usePedal(): Pedal {
  const [link] = useState(() => new PedalLink());
  const [s, dispatch] = useReducer(reducer, initial);
  const sRef = useRef(s);
  sRef.current = s;
  const lastSync = useRef<SyncTimes>({ dump: 0, saved: 0 });
  const storeWaiter = useRef<((status: number) => void) | null>(null);

  useEffect(() => {
    const offChange = link.onChange((snap) => dispatch({ type: "link", snap }));
    const offMsg = link.onMessage((msg) => {
      dispatch({ type: "msg", msg, now: Date.now() });
      if (msg.kind === "store") storeWaiter.current?.(msg.status);
      // The pedal restarted (or just linked): its WATCH is off and its tables are new.
      if (msg.kind === "log" && isBootLine(msg.text) && Date.now() - lastSync.current.dump >= SYNC_GAP_MS)
        requestSync(link, lastSync.current);
    });
    void link.start();
    const timer = setInterval(() => {
      const now = Date.now();
      dispatch({ type: "tick", now });
      const cur = sRef.current;
      if (!cur.link.linked) return;
      if (!cur.dumped && now - lastSync.current.dump >= RETRY_MS) {
        lastSync.current.dump = now;
        link.send(dump());
      }
      if (!cur.savedDone && now - lastSync.current.saved >= RETRY_MS) {
        lastSync.current.saved = now;
        link.send(savedQuery());
      }
    }, 250);
    return () => {
      offChange();
      offMsg();
      clearInterval(timer);
      link.stop();
    };
  }, [link]);

  const linked = s.link.linked;
  useEffect(() => {
    if (linked) requestSync(link, lastSync.current);
  }, [link, linked]);

  const store = async (bytes: number[]): Promise<void> => {
    const { promise, resolve, reject } = Promise.withResolvers<number>();
    const timer = setTimeout(() => {
      storeWaiter.current = null;
      reject(new Error("NO RESPONSE FROM PEDAL"));
    }, STORE_TIMEOUT_MS);
    storeWaiter.current = (st) => {
      clearTimeout(timer);
      storeWaiter.current = null;
      resolve(st);
    };
    link.send(bytes);
    const st = await promise;
    if (st !== STORE.ok) throw new Error(storeText(st));
  };

  return { s, link, dispatch, store };
}
