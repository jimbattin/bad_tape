# Bad Tape

A guitar pedal on the Electrosmith Daisy Seed in a Cleveland Music Co. HotHouse enclosure: EQ → delay → reverb (the EQ can also sit after the reverb), mono in, the output copied to both jacks. Every parameter is under USB MIDI control, and a browser editor edits, saves and recalls them.

## Repository layout

|Path|Contents|
|---|---|
|`daisy/`|Firmware. `src/bad_tape.cpp` (pedal: audio callback and main loop), `src/host_session.cpp` (the pedal's host protocol and saved values), `src/bench.cpp` (DSP benchmark image), `src/latency.cpp` (audio round-trip image), `src/firmware.h` (the startup they share: FZ, MPU), `src/dsp/` (the DSP, host-portable), `src/params.h` (the parameter table), `Makefile`, linker script, `tools/` (`flash_when_ready.sh`, `usb_log.py`), `libDaisy/` (submodule, v9.0.0). `daisy/README.md` is the firmware reference: file map, signal flow, controls, USB link, log lines, performance, memory map.|
|`editor/`|Browser editor (Bun, TypeScript, Preact, Web MIDI SysEx). See `editor/README.md`.|
|`tests/`|Host tests (GoogleTest) of everything in `daisy/src` that has no libDaisy dependency.|
|`host/`|Host programs: `render.cpp` (the DSP chain offline on a WAV), `gen_midi_doc.cpp` (writes `docs/MIDI.md`).|
|`tools/`|Python reverb measurements: `stimuli.py`, `features.py` (run with `uv`).|
|`docs/`|`MIDI.md` (generated: never edit by hand), `EQ.md` (the EQ's specification), `SOS.md` (sound on sound, a player's guide).|

## Build, test, flash

```sh
make test                          # host: build/host/tests + check docs/MIDI.md is current (needs pkg-config gtest_main)
make midi-doc                      # regenerate docs/MIDI.md after any change to daisy/src/params.h
git submodule update --init --recursive && make -C daisy/libDaisy   # once
make -C daisy -j                   # firmware (Arm GNU Toolchain 13.3.rel1 on PATH): daisy/build/hothouse/bad_tape.bin
BT_USB_PORT=3-3 daisy/tools/flash_when_ready.sh    # DFU flash; sends the running pedal the DFU SysEx first
uv run daisy/tools/usb_log.py 30 --port 3-3        # the pedal's log for 30 s
make -C daisy APP=bench -j         # benchmark image (APP=latency: latency image)
make -C editor test check          # editor tests, tsc, Biome
```

The firmware is `APP_TYPE = BOOT_SRAM`: it runs from SRAM, is flashed to QSPI at 0x90040000 and needs the Daisy bootloader (`make -C daisy program-boot`, once per Seed). Pedal hosts are build targets (`BOARD=hothouse`, the only one; others are added in `daisy/Makefile` and `daisy/src/board.h`).

Two Seeds can be attached to this machine, both `0483:5740` "Daisy Seed Built In". Select by USB path: `--port BUS-PORT` for `usb_log.py`, `BT_USB_PORT` for `flash_when_ready.sh` (both refuse to guess with more than one attached). Bad Tape answers `uv run daisy/tools/usb_log.py 3 --identity --port <path>` with `device 62 board 1`; confirm before every flash. In the editor, use the port that reads `LINKED`.

## Hardware: HotHouse

Six knobs (two rows of three), three on/off/on toggles SW1–SW3, footswitches FS1/FS2, LED1/LED2, stereo I/O used MIMO: left in, left out copied to right. Pins are in `daisy/src/board.h`. The board layer is our own; Cleveland's GPLv3 `hothouse.cpp` is not vendored.

## Pedal controls (what you see is what you get)

The knobs are remappable (factory layout below; editor Knobs tab, `knob_N_primary`/`knob_N_secondary`); the switches and footswitches are fixed.

|Control|Function|
|---|---|
|Knob 1|Delay mix|
|Knob 2|Reverb mix|
|Knob 3|Reverb Space, a macro per algorithm (small and short → large and long). Hall: size, decay, spread, shape. Plate: size, mid and LF decay, depth. Canyon: master delay, feedback, master depth.|
|Knob 4|Delay time, 10–2000 ms, audio taper (`32^n`, 15 % of the range at mid-travel: 309 ms at noon). FS2 held: `delay_sos_length` (10–120 s, log)|
|Knob 5|Delay feedback, 0–110 %|
|Knob 6|Delay color for the SW2 type, none fully CCW → full fully CW. BBD: sample degradation (still a BBD at 0). Tape: tape age. Digital: bitcrush, with four on/off options (`delay_crush_open`, `_clock`, `_wrap`, `_bits` + `delay_crush_mask`). Each type's scope (`delay_bbd_scope`, `delay_tape_scope`, `delay_crush_scope`; all, the default: the whole signal; repeats: the feedback loop only).|
|SW1|Delay: up forward, middle off (no new input; repeats ring out), down reverse|
|SW2|Delay type: up digital, middle tape, down BBD|
|SW3|Reverb: up Canyon, middle Hall, down Plate|
|FS1|Bypass (LED1 lit = effect on). Dry at unity; delay and reverb tails spill over.|
|FS2|Tap (released with no knob moved more than 3 % and no toggle changed): sound on sound on/off (`delay_sos`, when `delay_sos_mode` is not off). Held: knob N drives its FS2 secondary `knob_N_secondary` (factory: knob 4 drives `delay_sos_length`, the others none). Debounced 5 ms press / 30 ms release. No tap tempo or MIDI clock.|
|LED2|Input meter on the raw left input: off ≤ −30 dBFS, full ≥ −10 dBFS, linear in dB, instant attack, 150 ms release. Blinks at 2 Hz while sound on sound is on.|

A MIDI write holds a parameter on a knob slot until that knob moves more than 3 %. A knob turned under FS2 keeps its primary until it moves more than 3 % again with FS2 up. A toggle that changes position always wins. Details: `docs/MIDI.md` "Arbitration".

## Signal flow

The formulas are in `daisy/README.md` "Signal flow" and the code is `daisy/src/dsp/engine.h`. The reverb's input is the `routing` parameter (MIDI only): delay wet, delay mix (default) or dry. Knobs 1 and 2 are equal-power mixes (knob 2's equal point is at 75 %). In bypass the dry input is never equalised or crushed.

## Delay

`daisy/src/dsp/delay.h`, a 2^19-float line in SDRAM (5.46 s at 96 kHz). The types are `bbd.h` (compander, clocked sample-and-hold, filters, clock LFO), `tape.h` (wow/flutter, saturation, HF loss, LF bump, gated hiss, dropouts) and `crush.h` (bitcrush with open filters, a hold rate following the delay time, wrap, and bit inversion of an 8-bit word). Each type's scope (`delay_bbd_scope`, `delay_tape_scope`, `delay_crush_scope`) puts its color in one of two places. Repeats: inside the feedback loop, except the bit inversion and the tape's bump and hiss, which are on the wet output. All (the default): on the chain's input after a pre EQ (`Delay::ColorInput`, with its own tape, crush and BBD), so the dry, the repeats (once; the loop stays clean) and every reverb input are colored while bypass stays raw. Tape wow and flutter stay on the repeats' read head either way and also wobble the input in all (a short wow line); the BBD's clock LFO stays on the read head only. On a time change, tape and BBD glide the pitch and digital crossfades two heads. Reverse plays every repeat backwards (two crossfading grains), and the repeats stay reversed.

Sound on sound (`delay_sos_mode`: off, 301 (the default), 501, overdub; `delay_sos` on FS2 and CC 26) is a second line in SDRAM (`kSosSamples`, 44 MiB, 120 s) that records what the loop writes, every sample, and one more head into the wet: 301 `delay_sos_length` back, once; 501 at `delay_sos_ratio` × the delay time, fed back into the loop; overdub `delay_sos_length` back, the line keeping `delay_sos_feedback` of it. Idle, `Delay::Process` is the delay's own plus one store; live, `ProcessSos` runs out of line with its ramps, gains and 501 glide on the control tick (`SosTick`). `docs/SOS.md` is the player's guide.

## Reverb

There are three native C++ algorithms. Each aims at the character of its source, and none emulates it: no ROM code, microcode, or ROM-derived tables or data. Every algorithm parameter is a firmware parameter, even though only Space is on a knob.
- Hall (`hall.h`): after the Lexicon 480L Random Hall.
- Plate (`plate.h`): after the Lexicon 224XL Rich Plate.
- Canyon (`canyon.h`): after the Eventide H3000 Breathing Canyon (our own six-line modulated FDN).

The reverbs run at 48 kHz (`kReverbRate`; their line lengths are 48 kHz sample counts) inside a 2:1 half-band decimator and interpolator (`halfband.h` `HalfRateReverb`), the rest of the chain at 96 kHz. The three share one pool: `reverb_pool.h` sizes it from the largest algorithm's footprint per region, `reverb.h` holds the interface. The fast region is in D2 SRAM and the slow region in SDRAM. An SW3 change fades out, clears the pool (the old tail is lost) and fades in. Each header documents its topology, laws and deviations.

## EQ

A five-band parametric. `docs/EQ.md` is the normative spec and `daisy/src/dsp/eq.h` implements it: five RBJ cookbook sections (bilinear, prewarped) realised as Simper SVFs, designed in double and filtering in float (it meets the spec at 96 kHz). Low shelf 40–600 Hz, bell 1 40–800 Hz, bell 2 200 Hz–5 kHz, bell 3 800 Hz–12 kHz (bells Q 0.4–8), high shelf 1.5–16 kHz (shelves fixed at Q 0.7071), every gain ±12 dB; frequencies and Qs glide in log. Ids 74–85 (reused from the earlier Neve EQ) and 128–130. It is off by default. `eq_position` puts it pre (before the delay) or post (after the mix). It has no switch and no factory knob: MIDI, the editor, or a knob slot assigned to it. The editor's EQ tab draws the response from `editor/src/eqModel.ts`.

## MIDI and the USB link

- `daisy/src/params.h` is the single source of truth: ids, names, units, ranges, tapers, smoothing, switches, CCs, the knob-slot assignments and the Space links. Ids are fixed: append new parameters, never renumber. Ids are a `uint8_t` up to 254 (255 is `kNoParam`, a knob slot's none). Run `make midi-doc` after every change; `make test` fails while `docs/MIDI.md` is stale.
- The USB MIDI device takes CC (the factory knob parameters, the switches, FS1 and FS2), 14-bit NRPN (MSB = id >> 7, LSB = id & 0x7F) and SysEx for every parameter. SysEx frames are `F0 7D 62 02 <cmd> … F7` (manufacturer 0x7D non-commercial, device 0x62, link version 02: ids, counts and codes as two 7-bit bytes), specified in `daisy/src/link.h`. DFU is `F0 7D 62 02 06 F7`; `usb_log.py --dfu` also sends the version-01 frame `F0 7D 62 01 06 F7`, so it restarts older firmware. The Identity Reply carries board 1 (HotHouse) and the build id (short commit, `+dirty`). Frames for other devices and version-01 frames are ignored.
- The pedal sends only SysEx: a CC or NRPN write is applied and logged, never echoed (the Morningstar MC6 Pro takes incoming CCs on its channel as commands and rebooted on the echo). The editor learns of MIDI changes through WATCH.
- Saved power-up values (editor SAVE) exist only for parameters not on a switch or footswitch (the knob layout included; a knob's primary replaces its saved value at power-up). They live in two QSPI slots (`saved_store.h`; a 256-id payload, the older 128-id payload still read). There are no presets.
- The status log line reports the callback's CPU load (avg/max ‰) once a second. The bench image times every stage.

## Development rules

- C++17 (`gnu++17`), plain Make, `-O3`, no `-ffast-math`, FPSCR.FZ on. No heap anywhere in the firmware: check with `nm -u` on `daisy/build/hothouse/*.o`.
- `daisy/src/dsp/`, `params.h`, `param_store.h`, `controls.h`, `midi.h`, `link.h` and `saved_store.h` must stay host-portable (no libDaisy), so `tests/` and `host/render` build them.
- DSP output must not depend on block size. 96 kHz, 24-bit codec, 32-sample blocks (`btdsp::kSampleRate`, `btdsp::kBlockSize` in `dsp/blocks.h`); control ticks at 3 kHz (`kControlInterval` 32, the reverbs' `kReverbControlInterval` 16 at 48 kHz). Parameter changes must not click; `tests/dsp_invariants_test.cpp` checks every stage.
- Memory placement (`daisy/src/memory.h`): `BT_ITCM` on every per-sample and per-tick function and `AudioCallback`, and every caller of a `BT_ITCM` function is itself `BT_ITCM` or cold. GCC drops `BT_ITCM` from template instantiations: hot templates are `[[gnu::always_inline]]`. Hot state goes in DTCM (`BT_DTCM`), the delay and SOS lines and the reverb pool's slow region in SDRAM (`BT_SDRAM`), and the pool's fast region in D2 (`BT_D2`). NOLOAD regions are not zeroed at startup: their owner clears them.
- Load budget: worst case (`kWorst` in `src/bench.cpp`, every EQ section live) under 850 ‰ of the block period on the pedal.
- Keep these in step with the code: `daisy/README.md` (file map, controls, log lines, which are documented only there, and measured numbers), `docs/MIDI.md` (generated), `docs/EQ.md`, `editor/README.md`.
