#!/usr/bin/env -S uv run
"""Read the Bad Tape pedal's USB log directly over libusb, and read or set its parameters.

The Seed's USB port (VID:PID 0483:5740) is a USB MIDI device
(`daisy/src/bad_tape.cpp`, the bench `daisy/src/bench.cpp`, or the latency
image `daisy/src/latency.cpp`): the log
travels as LOG SysEx messages (`F0 7D 62 02 12 <ASCII> F7`,
`daisy/src/link.h`). Other SysEx is skipped. The
kernel's MIDI driver is detached while reading and reattached afterwards, so
close any other program holding the MIDI port first.

This script talks to the device through `pyusb` (the `usb` package, with a
libusb backend such as `libusb-1.0`), so no MIDI driver binding is needed. It
polls `usb.core.find()` until the device appears, and again whenever it
disappears, so it keeps reading across resets and reflashes (the port
re-enumerates each time) for as long as it is asked to run.

Usage:
    uv run daisy/tools/usb_log.py [SECONDS] [--port BUS-PORT] [--dfu] [--identity]
        [--dump] [--get NAME]... [--set NAME=VALUE]... [--check-params]

--port picks the Seed by its USB path (BUS-PORT, e.g. 3-3, as in
/sys/bus/usb/devices and dfu-util's -p); it is required when more than one
Seed (0483:5740) is attached, and the script exits 2 without it. The path is
the physical port, so it survives resets and reflashes.

SECONDS is how long to capture (default 30). Each output line is prefixed
with the seconds since the device was last (re)connected. --dfu sends the
link's DFU SysEx in both versions (`F0 7D 62 01 06 F7`, `F0 7D 62 02 06 F7`)
once the pedal is connected: it logs `dfu reset` and restarts into the Daisy
bootloader's DFU mode, ready for
`daisy/tools/flash_when_ready.sh` (which sends it itself). --identity sends
the MIDI Identity Request once the pedal is connected and prints its reply as
`-- identity device 62 board 1 build <build id>`.

Parameters (docs/MIDI.md; the table comes from the pedal's SysEx DUMP; only
the enum codes, CONTROL_NAMES and the tapers, mirror daisy/src/params.h). These run once the pedal is connected (after --identity and
--dfu); the script then exits, with SECONDS as the overall deadline:
    --dump               print `id name value unit [min..max] control cc`
    --get NAME           print `-- value NAME = V UNIT (code C)`, after any --set
    --set NAME=VALUE     set in engineering units (a MIDI write: held until
                         its knob moves more than 3 %), print the stored value
    --check-params       for every parameter SET a set of codes (0, 8191,
                         16383 and the default; every value of an enum), each
                         checked in the VALUE reply and by GET, then restore the
                         value it had; prints `-- check-params P/N ok` (exit 0)
                         or `-- check-params FAIL F/N` (exit 1)
Each reply has a 1 s timeout (`-- timeout waiting for ...`, exit 1). An
unknown NAME or a VALUE outside the parameter's range exits 2.
Requires `pyusb` (in the repo's uv environment: `uv sync` at the repo root)
and a working libusb backend.
"""
import argparse
import math
import sys
import time
from collections import deque
from dataclasses import dataclass

import usb.core
import usb.util

# Bytes per USB-MIDI event packet by code index number (libDaisy usb_midi.cpp).
CIN_SIZE = (3, 3, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1)
PEDAL_HEADER = bytes([0xF0, 0x7D, 0x62, 0x02])
LOG, VALUE, PARAM, DUMP_END = 0x12, 0x13, 0x14, 0x15  # pedal -> host
GET, SET, DUMP, DFU = 0x02, 0x03, 0x04, 0x06  # host -> pedal
IDENTITY_REQUEST = bytes([0xF0, 0x7E, 0x7F, 0x06, 0x01, 0xF7])
IDENTITY_REPLY_HEADER = bytes([0xF0, 0x7E, 0x7F, 0x06, 0x02, 0x7D])
REPLY_TIMEOUT = 1.0
CODE_MAX = 16383
TAPER_LOG, TAPER_ENUM, TAPER_AUDIO = 1, 2, 3
CONTROL_NAMES = ('-', 'SW1', 'SW2', 'SW3', 'FS1', 'FS2')


def sysex_packets(msg: bytes) -> bytes:
    """A complete F0 ... F7 message as USB-MIDI packets (cable 0), as TxBuffer::AppendRaw packs them: CIN 4 per 3
    bytes, the last 1/2/3 bytes as CIN 5/6/7."""
    out = bytearray()
    i = 0
    while len(msg) - i > 3:
        out += bytes([0x04]) + msg[i:i + 3]
        i += 3
    rest = msg[i:]
    out += bytes([0x04 + len(rest)]) + rest + bytes(3 - len(rest))
    return bytes(out)


def pedal_msg(cmd: int, payload: bytes = b'') -> bytes:
    return PEDAL_HEADER + bytes([cmd]) + payload + bytes([0xF7])


LEGACY_DFU = bytes([0xF0, 0x7D, 0x62, 0x01, DFU, 0xF7])  # link version 01 firmware (one-byte ids)
# Each firmware ignores the other version's frame, so --dfu restarts a pedal running either.
DFU_PACKETS = sysex_packets(LEGACY_DFU) + sysex_packets(pedal_msg(DFU))
IDENTITY_PACKETS = sysex_packets(IDENTITY_REQUEST)


def c_round(x: float) -> float:
    """std::round / lroundf: halves away from zero."""
    return math.copysign(math.floor(abs(x) + 0.5), x)


@dataclass
class Param:
    """One table entry from the pedal's PARAM message; value()/code() mirror daisy/src/params.h."""
    id: int
    taper: int
    control: int
    cc: int
    group: int
    def_code: int
    cur_code: int
    name: str
    unit: str
    min: float
    max: float

    @classmethod
    def parse(cls, msg: bytes) -> 'Param':
        p = msg[5:-1]
        parts = p[10:].decode('ascii', errors='replace').split('|')
        return cls(p[0] << 7 | p[1], p[2], p[3], p[4], p[5], p[6] << 7 | p[7], p[8] << 7 | p[9], parts[0],
                   '|'.join(parts[1:-2]), float(parts[-2]), float(parts[-1]))

    def value(self, code: int) -> float:
        n = min(code, CODE_MAX) / CODE_MAX
        if n <= 0:
            return self.min
        if n >= 1:
            return self.max
        if self.taper == TAPER_LOG:
            return self.min * (self.max / self.min) ** n
        if self.taper == TAPER_AUDIO:
            return self.min + (self.max - self.min) * (2 ** (5 * n) - 1) / 31
        if self.taper == TAPER_ENUM:
            return self.min + c_round(n * (self.max - self.min))
        return self.min + n * (self.max - self.min)

    def code(self, value: float) -> int:
        v = min(max(value, self.min), self.max)
        if self.taper == TAPER_ENUM:
            v = c_round(v)
        if self.taper == TAPER_LOG:
            n = math.log(v / self.min) / math.log(self.max / self.min)
        elif self.taper == TAPER_AUDIO:
            n = math.log2(1 + 31 * (v - self.min) / (self.max - self.min)) / 5
        else:
            n = (v - self.min) / (self.max - self.min)
        return int(c_round(n * CODE_MAX))

    def test_codes(self) -> list:
        if self.taper == TAPER_ENUM:
            return [self.code(v) for v in range(int(self.min), int(self.max) + 1)]
        return sorted({0, 8191, CODE_MAX, self.def_code})


class Timeout(Exception):
    pass


class Session:
    """One claimed connection: send SysEx, and receive the pedal's messages. LOG lines and identity replies are
    printed as they arrive (LOG only while show_log); other pedal SysEx is handed to the caller."""

    def __init__(self, ms_if, t0: float, end: float):
        self.ep_in = in_endpoint(ms_if)
        self.ep_out = out_endpoint(ms_if)
        self.t0 = t0
        self.show_log = True
        self.end = end  # overall deadline: no reply is waited for past it
        self.msg = None  # SysEx being assembled
        self.pending = deque()  # complete pedal messages not yet handed out

    def send(self, msg: bytes) -> None:
        self.ep_out.write(sysex_packets(msg), timeout=1000)

    def messages(self, deadline: float):
        """Yields the pedal's non-LOG SysEx messages until the deadline."""
        while True:
            while self.pending:
                yield self.pending.popleft()
            if time.time() >= deadline:
                return
            self._pump(min(500, max(1, int((deadline - time.time()) * 1000))))

    def wait(self, cmd: int, what: str, ident: int = None) -> bytes:
        """The next pedal message with this command (and id), within REPLY_TIMEOUT; others are dropped."""
        for m in self.messages(self.reply_deadline()):
            if m[4] == cmd and (ident is None or (len(m) > 7 and (m[5] << 7 | m[6]) == ident)):
                return m
        raise Timeout(f'-- timeout waiting for {what}')

    def reply_deadline(self) -> float:
        return min(time.time() + REPLY_TIMEOUT, self.end)

    def _pump(self, timeout_ms: int) -> None:
        try:
            data = bytes(self.ep_in.read(512, timeout=timeout_ms))
        except usb.core.USBTimeoutError:
            return
        for i in range(0, len(data) - 3, 4):
            cin = data[i] & 0x0F
            if cin < 2:  # reserved codes, skipped like libDaisy does
                continue
            for b in data[i + 1:i + 1 + CIN_SIZE[cin]]:
                if b >= 0xF8:  # realtime
                    continue
                if b == 0xF0:
                    self.msg = bytearray([b])
                elif self.msg is not None and (b < 0x80 or b == 0xF7):
                    self.msg.append(b)
                    if b == 0xF7:
                        self._complete(bytes(self.msg))
                        self.msg = None
                else:  # other status byte: not SysEx
                    self.msg = None

    def _complete(self, msg: bytes) -> None:
        if msg.startswith(PEDAL_HEADER) and len(msg) >= 6:
            if msg[4] == LOG:
                if self.show_log:
                    emit(self.t0, msg[5:-1].decode('ascii', errors='replace'))
            else:
                self.pending.append(msg)
        elif msg.startswith(IDENTITY_REPLY_HEADER) and len(msg) >= 16:
            build = msg[14:-1].decode('ascii', errors='replace')
            emit(self.t0, f'-- identity device {msg[6]:02X} board {msg[8]} build {build}')


def usb_path(d) -> str:
    """The device's USB path, BUS-PORT[.PORT...]: /sys/bus/usb/devices and dfu-util -p use the same string."""
    return f"{d.bus}-{'.'.join(str(p) for p in d.port_numbers)}"


def in_endpoint(intf):
    return next(e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)


def out_endpoint(intf):
    return next(e for e in intf if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)


def emit(t0: float, text: str) -> None:
    print(f'{time.time() - t0:7.2f}  {text}', flush=True)


def detach_drivers(d, cfg) -> list:
    """Detach the kernel driver from every interface that has one; returns their numbers."""
    detached = []
    for n in sorted({i.bInterfaceNumber for i in cfg}):
        try:
            if d.is_kernel_driver_active(n):
                d.detach_kernel_driver(n)
                detached.append(n)
        except NotImplementedError:  # backend without driver control (not Linux)
            pass
    return detached


def fmt(p: Param, code: int) -> str:
    return f'{p.value(code):g} {p.unit}'.rstrip()


def dump(s: Session) -> list:
    s.send(pedal_msg(DUMP))
    table = []
    while True:
        for m in s.messages(s.reply_deadline()):
            if m[4] in (PARAM, DUMP_END):
                break
        else:
            raise Timeout('-- timeout waiting for DUMP')
        if m[4] == DUMP_END:
            return table
        p = Param.parse(m)
        if p.id == 0:  # a restarted dump
            table = []
        table.append(p)


def get_code(s: Session, p: Param) -> int:
    s.send(pedal_msg(GET, bytes([p.id >> 7, p.id & 0x7F])))
    m = s.wait(VALUE, f'VALUE {p.name}', p.id)
    return m[7] << 7 | m[8]


def set_code(s: Session, p: Param, code: int) -> int:
    s.send(pedal_msg(SET, bytes([p.id >> 7, p.id & 0x7F, code >> 7, code & 0x7F])))
    m = s.wait(VALUE, f'VALUE {p.name}', p.id)
    return m[7] << 7 | m[8]


def check_params(s: Session, table: list) -> int:
    ok = 0
    for p in table:
        failures = 0
        for code in p.test_codes():
            got = set_code(s, p, code)
            if got != code:
                print(f'-- {p.name}: SET code {code} answered {got}', flush=True)
                failures += 1
            got = get_code(s, p)
            if got != code:
                print(f'-- {p.name}: GET after SET code {code} answered {got}', flush=True)
                failures += 1
        set_code(s, p, p.cur_code)
        ok += failures == 0
    if ok == len(table):
        print(f'-- check-params {ok}/{len(table)} ok', flush=True)
        return 0
    print(f'-- check-params FAIL {len(table) - ok}/{len(table)}', flush=True)
    return 1


def run_params(s: Session, args) -> int:
    """The parameter flags; returns the exit code."""
    s.show_log = not args.check_params
    table = dump(s)
    by_name = {p.name: p for p in table}
    names = list(args.get) + [kv.split('=', 1)[0] for kv in args.set]
    for name in names:
        if name not in by_name:
            print(f'-- unknown parameter {name!r} (see docs/MIDI.md)', flush=True)
            return 2
    sets = []
    for kv in args.set:
        name, _, text = kv.partition('=')
        p = by_name[name]
        try:
            v = float(text)
        except ValueError:
            v = math.nan
        if not p.min <= v <= p.max:
            print(f'-- {name}={text} outside {p.min:g}..{p.max:g}', flush=True)
            return 2
        sets.append((p, v))
    if args.dump:
        for p in table:
            cc = p.cc if p.cc else '-'
            print(f'{p.id:3d} {p.name:24s} {p.value(p.cur_code):g} {p.unit} [{p.min:g}..{p.max:g}] '
                  f'{CONTROL_NAMES[p.control] if p.control < len(CONTROL_NAMES) else p.control} {cc}', flush=True)
    for p, v in sets:
        code = set_code(s, p, p.code(v))
        print(f'-- value {p.name} = {fmt(p, code)} (code {code})', flush=True)
    for name in args.get:
        p = by_name[name]
        code = get_code(s, p)
        print(f'-- value {name} = {fmt(p, code)} (code {code})', flush=True)
    return check_params(s, table) if args.check_params else 0


def read_midi(d, cfg, ms_if, end: float, pending: list, args):
    """`pending` holds (USB-MIDI packets, description) still to send (sent once, then removed). With parameter flags
    in args, runs them and returns the exit code; otherwise reads the log until `end` and returns None."""
    # The audio driver owns both the control and the MIDI interface; detaching
    # one shuts its card down, so both are detached and reattached together.
    detached = detach_drivers(d, cfg)
    try:
        usb.util.claim_interface(d, ms_if.bInterfaceNumber)
        s = Session(ms_if, time.time(), end)
        while pending:
            packets, text = pending.pop(0)
            s.ep_out.write(packets, timeout=1000)
            emit(s.t0, f'-- sent {text}')
        if args.dump or args.get or args.set or args.check_params:
            try:
                return run_params(s, args)
            except Timeout as e:
                print(e, flush=True)
                return 1
        for _ in s.messages(end):
            pass
        return None
    finally:
        try:
            usb.util.release_interface(d, ms_if.bInterfaceNumber)
        except usb.core.USBError:
            pass
        for n in detached:
            try:
                d.attach_kernel_driver(n)
            except usb.core.USBError:  # gone, or already claimed by the reattached control interface
                pass


def main() -> None:
    ap = argparse.ArgumentParser(description='Read the Bad Tape pedal USB log; read and set its parameters.')
    ap.add_argument('seconds', nargs='?', type=float, default=30.0, help='capture time / deadline (default 30)')
    ap.add_argument('--dfu', action='store_true', help='send the DFU SysEx: the pedal restarts into DFU')
    ap.add_argument('--port', metavar='BUS-PORT',
                    help='the USB path of the pedal (e.g. 3-3, as in /sys/bus/usb/devices); '
                         'required when more than one Seed is attached')
    ap.add_argument('--identity', action='store_true', help='send the MIDI Identity Request and print the reply')
    ap.add_argument('--dump', action='store_true', help='print the parameter table with current values')
    ap.add_argument('--get', action='append', default=[], metavar='NAME', help='print a parameter (repeatable)')
    ap.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='set a parameter in engineering units (repeatable)')
    ap.add_argument('--check-params', action='store_true', help='SET/GET round trip of every parameter')
    args = ap.parse_args()
    for kv in args.set:
        if '=' not in kv:
            ap.error(f'--set {kv}: expected NAME=VALUE')
    pending = [(IDENTITY_PACKETS, 'identity')] if args.identity else []
    if args.dfu:
        pending.append((DFU_PACKETS, 'dfu'))
    end = time.time() + args.seconds
    while time.time() < end:
        seeds = list(usb.core.find(find_all=True, idVendor=0x0483, idProduct=0x5740))
        if args.port:
            d = next((s for s in seeds if usb_path(s) == args.port), None)
        elif len(seeds) > 1:
            print('-- usb: ' + str(len(seeds)) + ' Seeds attached (' + ', '.join(usb_path(s) for s in seeds)
                  + '); pass --port', flush=True)
            sys.exit(2)
        else:
            d = seeds[0] if seeds else None
        if d is None:
            time.sleep(0.2)
            continue
        try:
            cfg = d.get_active_configuration()
            ms_if = next((i for i in cfg if i.bInterfaceClass == 0x01 and i.bInterfaceSubClass == 0x03), None)
            if ms_if is None:
                print('-- usb: no MIDI streaming interface', flush=True)
                time.sleep(0.5)
                continue
            rc = read_midi(d, cfg, ms_if, end, pending, args)
            if rc is not None:
                sys.exit(rc)
        except usb.core.USBError as e:
            print(f'-- usb: {e}', flush=True)
            time.sleep(0.5)
    if args.dump or args.get or args.set or args.check_params:
        print('-- no pedal answered before the deadline', flush=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
