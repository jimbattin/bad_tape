# Bad Tape on the Daisy Seed (Cleveland Music Co. HotHouse)

The pedal firmware: EQ → delay → reverb on the left input (or delay → reverb → EQ), copied to both outputs (MIMO). The DSP chain (`src/dsp/engine.h`) is the forward or reverse delay of the type SW2 selects (BBD, tape or digital) with knob 6 coloring it (sample degradation, tape age or bitcrush; by default each colors the whole signal: `delay_bbd_scope`, `delay_tape_scope`, `delay_crush_scope`), feeding the reverb SW3 selects: down the Plate, middle the Hall, up the Canyon, with an SW3 change fading the reverb out, clearing the shared pool and fading the new algorithm in. The reverb's input is the delay mix unless the `routing` parameter (MIDI only) selects the delay wet or the dry input, and knobs 1 and 2 are equal-power mixes, dry and wet equal at 50 % on knob 1 and at 75 % on knob 2. The EQ (`src/dsp/eq.h`, a five-band parametric: low shelf, three bells, high shelf; `../docs/EQ.md`) has no pedal control and is off by default; `eq_position` puts it before the delay or after the mix, and FS1 bypass always passes the dry input unequalised and uncolored. Every parameter is under MIDI control (CC, NRPN, SysEx); the controls, LEDs, USB MIDI link, logging and DFU work. Two more images share the sources: the bench (`APP=bench`) times every DSP stage on the Seed, and the latency image (`APP=latency`) measures the audio round trip.

- `src/bad_tape.cpp`: the pedal: the audio callback, the main loop and the state they share (its header comment documents the threading and the memory placement).
- `src/host_session.h`, `src/host_session.cpp`: the pedal's host session on the main loop: SysEx host messages, MIDI CC/NRPN writes, the saved power-up values in QSPI (load at boot, SAVE/CLEAR), the replies, the dump and the WATCH stream.
- `src/bench.cpp`: the DSP benchmark image (Performance below). `src/latency.cpp`: the audio round-trip image (Performance, "Latency").
- `src/firmware.h`: the startup every image shares (`btfw::Start`: FPSCR.FZ, the board, the MPU map with the first 4 MB of QSPI readable, so the memory-mapped reads of the saved values cannot trigger the 0x907FFFFF AXI stall, ES0392 2.7.5) and the build id.
- `src/usb_link.h`, `src/usb_link.cpp`: the USB side of the link for every image: the USB MIDI device, the receive ring, the double-buffered non-blocking send and log, DFU, and the bench's and latency image's HELLO/DFU session.
- `src/board.h`: the HotHouse (pins, knobs, toggles, footswitches, LEDs) on libDaisy's `DaisySeed` directly.
- `src/params.h`: the parameter table (ids, names, units, ranges, tapers, smoothing, switches, CCs), the single source of truth, the twelve knob-slot assignments (`knob_N_primary`, `knob_N_secondary`, `ResolveSlot`) and the links (the `hall_space`, `plate_space` and `canyon_space` macros, each law documented in its `kLinks` entry); host-portable. [`../docs/MIDI.md`](../docs/MIDI.md) is generated from it.
- `src/param_store.h`: the parameter values, the knob slots (primaries, FS2 secondaries, remapping), the knob/MIDI arbitration and the links (a source change writes its targets; a MIDI-written target is held until its source moves 3 %); host-portable.
- `src/midi.h`: CC extraction from the raw MIDI stream and NRPN decoding; host-portable.
- `src/dsp/engine.h`: the DSP chain and the SW3 switch (fade, pool clear, swap); no libDaisy, the same code runs on the Seed, in the bench, in `build/host/render` and under the host tests. `src/dsp/halfband.h`: `HalfRateReverb`, the 47-tap half-band decimator and interpolator that run the reverb at half the chain's rate. `src/dsp/blocks.h`: the building blocks the delay, the reverbs and the EQ share (the audio rate, the reverbs' rate, the block size and the control ticks: `kSampleRate` 96 kHz, `kReverbRate` 48 kHz, `kBlockSize` 32, `kControlInterval` 32 and `kReverbControlInterval` 16, both 3 kHz; constants and glide laws, coefficient helpers, flush to zero, dB to gain, soft clipper, state-variable filter and 4-pole Butterworth, one-pole, smoothers and ramps, smoothstep crossfade, noise gate, xorshift, quadrature oscillator, and the fractional line, allpass and glide). `src/dsp/delay.h`: the delay line (Hermite reads, two-head crossfade on time changes for the digital type, a read-speed glide for tape and BBD, `ReverseGrains`: two crossfading reverse grains, DC blocker and soft limiter in the loop; `DelayTargets` and the type → color policy; each color's scope: in the loop or on the chain's input, `ColorInput`, with its own tape and wow line, crush and BBD); `src/dsp/bbd.h`: the BBD type (compander, clocked sample-and-hold, filters, overload, noise); `src/dsp/tape.h`: the tape type's color (`colorShare` splits wow and flutter from the rest); `src/dsp/crush.h`: the digital type's color (bitcrush and its options: open filters, clock, wrap, bit inversion); `src/dsp/reverb.h`: the `Reverb` interface, the `ReverbPool` and the mono fold `kReverbFold`, and `AlgoParams` (an algorithm's parameter values and smoothers); `src/dsp/reverb_pool.h`: the pool's sizes, the largest algorithm's footprint per region, and `ClearPool`; `src/dsp/hall.h`: the Hall (480L Random Hall character); `src/dsp/plate.h`: the Plate (224XL Rich Plate character, a figure-8 plate tank); `src/dsp/canyon.h`: the Canyon (H3000 Breathing Canyon character, our own six-line modulated FDN). Each reverb's header documents its topology, laws and deviations.
- `src/controls.h`: toggle meanings (delay mode, delay type, reverb algorithm, and the routing parameter's values), toggle hysteresis and the FS2 debounce (`ShiftFilter`); host-portable.
- `src/link.h`: the USB MIDI link (SysEx `F0 7D 62 02 <cmd> … F7`, ids and counts as two 7-bit bytes), host-portable.
- `src/memory.h`, `src/memory.cpp`: `BT_ITCM`/`BT_DTCM`/`BT_SDRAM`/`BT_D2` placement (empty on the host) and the boot-time TCM copy.
- `src/spsc_ring.h`: the lock-free ring from the USB IRQ to the main loop (also the main loop's reply queue).
- `src/saved_store.h`: the saved power-up values (editor SAVE): two 4 KB QSPI slots at 0x903FE000/0x903FF000 past the BOOT_SRAM image, CRC-32 and sequence number per slot, the newest valid slot wins, a 1 056-byte payload for 256 ids (the 528-byte 128-id payload of link-version-01 firmware is still read, its knob assignments of 126 and up becoming none); host-portable.
- `../editor/`: the browser editor (Bun, TypeScript, Preact; see its README): every parameter not on a switch or FS1 can be edited over the SysEx link, saved as a power-up value, recalled; the knob layout (primaries and FS2 secondaries) is edited on its Knobs tab; the sidebar sets the switches, FS1 and the knobs' parameters as MIDI writes.
- `STM32H750IB_sram_badtape.lds`: libDaisy's BOOT_SRAM script with `.data`/`.bss` in AXI SRAM, plus `.dtcm_data` and `.itcm_text` (loaded from SRAM, copied at boot), `.d2_bss` (NOLOAD, the reverb pool's fast region), `.sdram_bss` (NOLOAD, the delay line and the pool's slow region) and a 16 KB stack reserve in DTCM (`.reserved_for_stack`: DTCM overflow is a link error).
- `libDaisy/`: submodule, v9.0.0.
- `Makefile`: the firmware build (`APP=bad_tape|bench|latency`, `BOARD=hothouse`, a generated `build_id.c` per build directory; a linker script change relinks).
- `tools/flash_when_ready.sh`: DFU flashing (sends the running image's DFU SysEx first when no DFU device is present).
- `tools/usb_log.py`: the log reader over pyusb, with `--dfu`, `--identity` and the parameter commands `--dump`, `--get`, `--set`, `--check-params`.

Host side (repo root): `Makefile` (`make host`/`make test`/`make midi-doc`/`make check-midi-doc`), `tests/` (GoogleTest), `host/` (`render.cpp`, `gen_midi_doc.cpp`, `wav.h`), `tools/` (`features.py`, `stimuli.py`: reverb measurements), `pyproject.toml`/`uv.lock` (the Python tools' environment).

## Signal flow

```
x    = Color(eqPre ? EQ(in) : in)             // eq_position pre (the EQ is off by default: identity); Color: the SW2
                                              // type's color with its scope all (Delay::ColorInput), else identity
w    = Delay(x · inputOn)                     // delay wet; inputOn = 0 (5 ms ramp) with SW1 off or in bypass: tails spill
m    = MixD(x, w, k1)                         // Knob 1, Delay Mix
r_in = { wet: w,  mix: m,  dry: x }           // routing (MIDI; default mix)
wet  = g · Fold(Reverb(r_in · (1 − b) · g))   // g: the SW3 switch gain (1; ramps to 0 and back across a switch)
y    = MixR(m, wet, k2)                       // Knob 2, Reverb Mix
out  = eqPost ? EQ((1 − b) · y + b · tails) + b · in      // b: bypass (5 ms ramp); tails = √t1 · w + √t2 · wet
     :          (1 − b) · y + b · (in + tails);   outR = outL
```
- `MixD` and `MixR` are equal-power crossfades (`√(1−t)` input, `√t` wet), with `t = k1` for `MixD` and `t = k2^2.409` for `MixR` (equal at Knob 2 75 %). 0 % gives the input only and 100 % gives the wet only. At 100 %/100 % with the default routing (mix), the chain is the pure series chain (EQ → delay → reverb). A routing change crossfades the reverb input over 20 ms. `Fold` is `kReverbFold · (L + R)`. The chain runs at 96 kHz; `Reverb` runs at 48 kHz between a 47-tap half-band decimator and interpolator (`src/dsp/halfband.h` `HalfRateReverb`: flat to 18 kHz, −80 dB from 30 kHz), which delay the wet by 49 samples (0.51 ms).
- In bypass, the dry input goes to the output unchanged (never equalised or colored, in either EQ placement), and the reverb's input is muted. The delay and reverb tails keep decaying and are summed onto the dry signal until they fall silent; post, they pass through the EQ; pre, they were equalised (and, with the color's scope all, colored) on the way in.
- An `eq_position` change fades the EQ out over 20 ms, moves it on a chunk boundary and fades it back in.

## Controls and LEDs

|Control|Seed pin|Use|
|---|---|---|
|Knobs 1-6|D16-D21|remappable (`knob_N_primary`, editor Knobs page); factory: 1 delay mix, 2 reverb mix, 3 reverb space (each algorithm's macro: size and decay together, small and short → large and long), 4 delay time (10-2000 ms, audio taper: 15 % of the range at mid-travel, 309 ms at noon), 5 delay feedback, 6 delay color (none fully counter-clockwise → full fully clockwise, for the SW2 type)|
|SW1 up/down|D9/D10|delay: up forward, middle off (no new input, the repeats ring out in the last direction), down reverse (every repeat backwards, windows as long as the delay time)|
|SW2 up/down|D7/D8|delay type: up digital (knob 6 bitcrush), middle tape (knob 6 tape age), down BBD (knob 6 sample degradation)|
|SW3 up/down|D5/D6|reverb: up Canyon, middle Hall, down Plate|
|FS1 / FS2|D25 / D26|bypass (dry at unity, the delay's repeats and the reverb's tail spill over) / shift: held, each knob drives its FS2 parameter (`knob_N_secondary`; factory none)|
|LED1 / LED2|D22 / D23|lit while the effect is active / input meter|

- Toggles read 2 up, 1 middle, 0 down (an unwired toggle reads middle). A new position is accepted after 90 consecutive 1/3 ms blocks (30 ms), so a lever thrown up ↔ down never reports the middle it passes.
- Knob 6 colors the delay type SW2 selects, from none fully counter-clockwise to full fully clockwise. Each type's scope (`delay_bbd_scope`, `delay_tape_scope`, `delay_crush_scope`; default all; a change moves the color from one place to the other over 20 ms) sets where its color sits: all colors the chain's input after a pre EQ, so the dry, the repeats (each colored once: the loop stays clean) and every reverb input whatever `routing` carry it, and bypass still passes the raw input; repeats puts it in the feedback loop, so each repeat is colored again and the dry stays clean (the tape's bump and hiss and the digital type's bit inversion on the wet output only). The tape's wow and flutter stay on the repeats' read head either way and with all also wobble the input, so the dry lags by the wobble's span (about 2.1 ms at full age with the default depths, up to 5.5 ms at their maxima; moving knob 6 or the depths glides it). The BBD's clock LFO stays on the repeats only; the input BBD is clocked from the delay time like the loop's.
- BBD (SW2 down; `src/dsp/bbd.h`): a 2:1/1:2 compander (NE570 style, unity at −20 dBFS) around the chip: 4-pole anti-alias and reconstruction lowpasses at `delay_bbd_tone` (4 kHz), a sample-and-hold clocked at `delay_bbd_stages` (16384) / (2 × the delay time), a soft overload and noise inside the compander, so it breathes with the signal and stops with the repeats. Fully counter-clockwise it is still a BBD: the filters darken every repeat, long delays clock below the filters and alias (8.2 kHz at 1 s, 4.1 kHz at 2 s), with 3 dB of overload and the noise at a tenth of `delay_bbd_noise`. Turning knob 6 clockwise multiplies the clock by down to `delay_bbd_clock` (0.25, two octaves: 13.7 → 3.4 kHz at 600 ms, where a 2.5 kHz note folds down to 913 Hz), raises the overload by up to `delay_bbd_drive` (12 dB) and the noise to `delay_bbd_noise` (−60 dBFS RMS). A triangle LFO swings the clock (Deluxe Memory Man style, MIDI only): `delay_bbd_mod_rate` (0.2 Hz, 0.1–10 Hz) and `delay_bbd_mod_depth` (0.5 % of the clock, up to 25 %), so the repeats bend sharp and flat by about ±4 × rate × depth × delay (s), ±7 cents at 1 s by default; it ignores knob 6 and fades out over 150 ms when SW2 leaves the BBD.
- Tape (SW2 middle; age scales every part in from nothing; the `delay_tape_*` values are at full age): wow 0.5-2 Hz and flutter 6-12 Hz on the read position with random drift (`delay_tape_wow` 2 ms, `delay_tape_flutter` 0.1 ms); a saturation that turns asymmetric with age (`delay_tape_drive` 9 dB, never expands) and a lowpass falling to `delay_tape_tone` (2.5 kHz); a +`delay_tape_bump` (3 dB) peak at 100 Hz and hiss at `delay_tape_hiss` (−60 dBFS RMS) gated by the signal it rides on, so it stops with it; above 60 % age, random 20-80 ms dropouts (`delay_tape_dropouts`, 6 per minute at full age). With scope repeats the saturation and lowpass sit in the loop, the bump and hiss on the wet output and the dropouts dip the wet; with all they all sit on the input.
- Digital (SW2 up; bitcrush; `delay_crush_scope` as above): the hold rate falls toward `delay_crush_rate` (default 1 kHz, 1–16 kHz); the bit depth steps 12 → 8 at 45 % → 4 at 78 %, with ±1 % of hysteresis at each step; the signal is driven into a soft clipper ahead of the quantiser up to `delay_crush_drive` (default 12 dB, max 24 dB) and brought back after it; a 4-pole lowpass after the hold (`delay_crush_tone`) removes the staircase fizz, and the anti-alias filter ahead of it (`delay_crush_aa`, default 0.4 × the hold rate) keeps most aliasing out. Four on/off options (each fades over 20 ms) set what else knob 6 does: `delay_crush_open` (on) opens both filters with the knob, to a bare sample-and-hold fully clockwise (everything above half the hold rate folds down, and the staircase stays); `delay_crush_clock` (off) makes `delay_crush_rate` the hold rate at a 1 s delay, scaled by 1 s / the delay time (500 Hz at 2 s with the default 1 kHz), so knob 4 sweeps the aliasing; `delay_crush_wrap` (on) wraps an overload to the opposite rail instead of clipping it; `delay_crush_bits` (off) inverts bits after the crush, never fed back (on the wet's output with scope repeats, on the crushed input with scope all): the bits of `delay_crush_mask` (default 0000 1100, bit 7 the MSB) are inverted in an 8-bit offset-binary word and mixed in with the knob, fully from −20 dBFS and with the square of the level below, so it falls away with the signal; a signal below half an 8-bit step passes untouched, so the output is silent once the input and the repeats are. Fully counter-clockwise the digital type is clean.
- A knob 4 change glides the read speed (about 150 ms, 0.5-1.5× speed) with tape and BBD, at any color, so the pitch bends; the digital type crossfades two read heads at pitch instead. An SW2 change smooths the colors over 20 ms and crossfades the BBD stage in or out over 20 ms.
- LED1 blinks three times at power-on, then is lit while the effect is active (off in bypass).
- LED2 meters the left input: off at or below −30 dBFS, full at −10 dBFS and above, its duty linear in dB between them (`cbrt` undoes libDaisy's cube law), instant attack, 150 ms release. Its software PWM is updated every 8th sample (12 kHz).
- From 200 ms after the audio starts every knob and toggle sets its parameter (what you see is what you get). Each knob drives the parameter its primary slot `knob_N_primary` names (any parameter not on a switch or FS1; 255 none) and, while FS2 is held, the one its secondary `knob_N_secondary` names; a Space slot drives the active algorithm's space macro (`plate_space`, `hall_space`, `canyon_space`). FS2 is debounced in `btctl::ShiftFilter`: a press counts after 5 ms of steady contact, a release after 30 ms. While FS2 is held a knob drives its secondary only once it has moved more than 3 % from where it was at the press; a knob moved more than 3 % under FS2 leaves its primary at its value until it moves more than 3 % from where it was at the release. A newly assigned parameter keeps its value until its knob moves more than 3 %; a parameter on two slots follows the knob that moved last; a link target on a slot is the knob's, and its macro no longer writes it. `hall_space` sets `hall_size` (3–39 m), `hall_rt_mid` (0.4–10 s), `hall_spread` and `hall_shape` together; `plate_space` sets `plate_size` (0–100 %), `plate_mid_decay` and `plate_lf_decay` (0.36–9 s) and `plate_depth`; `canyon_space` sets `canyon_master_delay` (30–100 %), `canyon_feedback` (60–100 %) and `canyon_master_depth`. A MIDI write to a target holds it until its Space knob moves more than 3 % ([`../docs/MIDI.md`](../docs/MIDI.md) Arbitration).

## USB MIDI link

The Seed's USB port is a USB MIDI device ("Daisy Seed Built In", `0483:5740`). `src/link.h` documents the SysEx messages and [`../docs/MIDI.md`](../docs/MIDI.md) the whole MIDI implementation; the pedal's own SysEx is `F0 7D 62 02 <cmd> … F7` (link version 02: ids, counts and codes travel as two 7-bit bytes, `hi = v >> 7`, `lo = v & 0x7F`), besides the standard Identity Request/Reply, CC and NRPN. Frames with other device bytes, and version-01 frames (the previous one-byte-id format), are ignored.

|Direction|Message|Effect|
|---|---|---|
|host → pedal|`01` HELLO|the pedal logs the boot line again|
|host → pedal|`06` DFU|logs `dfu reset`, flushes for ~100 ms, restarts into the Daisy bootloader's DFU mode (no timeout)|
|host → pedal|Identity Request `F0 7E <id> 06 01 F7`|Identity Reply|
|host → pedal|`02` GET `<id hi> <id lo>`, `03` SET `<id hi> <id lo> <hi> <lo>`, `04` DUMP|VALUE; SET writes the parameter first (as a MIDI write); DUMP answers one PARAM per id, then DUMP END|
|host → pedal|CC (any channel) on a parameter's CC; NRPN 99 = id >> 7, 98 = id & 0x7F, 6/38 = 14-bit code|sets the parameter; nothing is sent back (the log shows the stored value)|
|host → pedal|`05` WATCH `<on>`, `07` SAVE `<id hi> <id lo>…`, `08` CLEAR `<id hi> <id lo>…`, `09` SAVED?|WATCH streams VALUE for every parameter that changes (any cause, 20 ms checks); SAVE/CLEAR persist or forget power-up values in one flash write; SAVED? lists them|
|pedal → host|`12` LOG `<ASCII ≤ 120>`|one log line; dropped, never blocking, when the send buffers are full|
|pedal → host|`13` VALUE `<id hi> <id lo> <hi> <lo>`, `14` PARAM, `15` DUMP END `<count hi> <count lo>`|replies to GET/SET/DUMP|
|pedal → host|`16` SAVED `<id hi> <id lo> <has> <hi> <lo>`, `17` STORE `<status>`, `18` SAVED END `<count hi> <count lo>`|replies to SAVE/CLEAR/SAVED?|
|pedal → host|Identity Reply `F0 7E 7F 06 02 7D 62 00 01 00 00 00 00 00 <build id> F7`|board 1 = HotHouse; build id = short commit, `+dirty` for uncommitted tracked changes, or `unknown`|

A MIDI or SysEx write holds a parameter on a knob slot until that knob moves more than 3 % (the log shows `midi NAME = V UNIT` for the write, then `knob N takes NAME`, or `knob N fs2 takes NAME` for an FS2 secondary); a toggle that changes position always wins. Any other command with the pedal's header logs `link: unknown command CC`; a GET/SET/DUMP with a bad id or length logs `link: bad param message CC`. Logging goes straight to `UsbHandle::TransmitInternal` from two 1 KB buffers, because libDaisy's `MidiUsbHandler::SendMessage` retries a busy endpoint with a delay. Floats are formatted without newlib-nano's float printf (`btlink::FormatNumber`).

Log lines (this is their only description; the bench's and the latency image's are under Performance):
- At power-on and on HELLO: `bad_tape hothouse sysclk N MHz audio R Hz B bit block K <build id>` (the codec's rate, sample word and block size as libDaisy reports them: `audio 96000 Hz 24 bit block 32`).
- On every MIDI or SysEx write: `midi NAME = V UNIT` (the stored value); when a knob takes its parameter back: `knob N takes NAME` (primary) or `knob N fs2 takes NAME` (FS2 secondary); on DFU: `dfu reset`; `link: unknown command CC` and `link: bad param message CC` as above.
- Once a second the raw controls: `ctl k K1 … K6 sw S1 S2 S3 fs F1 F2 time T ms type bbd|tape|digital color C`. `k` is each knob in percent, `sw` the filtered toggle positions, `fs` whether FS1 (libDaisy debounce) and FS2 (the shift debounce) are held, `time` the effective `delay_time`, `type` and `color` the effective `delay_type` and `delay_color` (switch, knob or MIDI).
- When a SAVE or CLEAR writes flash: `saved slot A|B seq N T ms` (or `saved write failed`); at power-on `saved empty` or `saved slot A|B seq N: K params`; `link: bad save message CC` and `link: bad watch message` as above.
Once a second, after the `ctl` line, a status line: `load avg A max M permille underruns U bypass on|off delay fwd|off|rev route wet|mix|dry algo plate|hall|canyon in P dBFS`.
- `load`: the audio callback's `CpuLoadMeter` average and maximum since the previous line, in permille of the block period.
- `underruns`: blocks since boot whose callback ran longer than one block period (32 samples at 96 kHz = 1/3 ms, timed with `System::GetTick()`).
- `bypass`, `delay`, `route`, `algo`: the effective parameter values (controls or MIDI; `route` is MIDI only).
- `in`: the left input's peak since the previous line in dBFS (−120 for silence).

`uv run daisy/tools/usb_log.py 6 --identity` sends the Identity Request and prints `-- identity device 62 board 1 build <build id>`; `--dfu` sends DFU in both link versions (01 and 02), so it restarts a pedal running either.

Parameters over SysEx (`usb_log.py` reads the table from the pedal's DUMP, so it never hard-codes it):

```sh
uv run daisy/tools/usb_log.py 10 --dump                     # id name value unit [min..max] control cc
uv run daisy/tools/usb_log.py 10 --set delay_time=500 --get delay_time
uv run daisy/tools/usb_log.py 60 --check-params             # SET/GET round trip of every parameter
```

## Host tests

`make host && build/host/tests` (or `make test`, which also fails when `docs/MIDI.md` is not what `make midi-doc` writes) from the repo root builds the host-portable headers in `daisy/src/` against GoogleTest (`pkg-config gtest_main`) and runs `tests/*.cpp`. `tests/test_util.h` holds the shared helpers: signals and levels, a zero-crossing frequency measure, an RT60 measure, the engine rigs (`EngineRig` with `Render`, `DelayRig`), the color scopes' loop probe (`SecondRepeatError`), the single-algorithm `ReverbRig` and the checks every reverb shares (bounded decay at every loop-parameter corner, L/R decorrelation). The suites:
- `tests/blocks_test.cpp`: the parameter smoother's time constant; `HalfRateReverb` passing 5 kHz at level (±1.2 %) and leaving a 40 kHz tone's 8 kHz alias 70 dB down.
- `tests/controls_test.cpp`: the toggle position mapping and hysteresis, the FS2 debounce (press and release counts, bounce on either edge).
- `tests/params_test.cpp`: the table's ranges, tapers and codes, every link law rising with its source and giving the target's default at the source's.
- `tests/param_store_test.cpp`: knob pickup, toggle precedence (SW2 sets the delay type and leaves the MIDI-only routing alone), Space following SW3 on any knob, each space macro writing its targets and a MIDI-written target held until knob 3 moves 3 %, the FS2 secondaries (pickup at the press, the primary frozen until the knob moves again), remapping (a new assignment held, duplicates following the last-moved knob, none driving nothing, a link target on a slot left alone by its macro).
- `tests/midi_test.cpp`, `tests/link_test.cpp`: CC/NRPN parsing (ids past 127 with NRPN MSB 1) and the SysEx link (two-byte ids and counts, version-01 frames ignored, `DecodeIds`).
- `tests/dsp_invariants_test.cpp`: the invariants every stage meets (block-size independence, no NaN/Inf or denormal stalls, bounded impulse response, no click on a parameter jump), run on the engine at the defaults (tape, color 0), at full color on each delay type (each with its scope all and repeats), on the BBD at color 0 and in reverse, each with the Hall at the default `reverb_mix`, and on the Plate and the Canyon; the parameter-jump check includes the `delay_type` (SW2) and `reverb_algo` (SW3) switches.
- `tests/delay_test.cpp`: timing, feedback decay, bounded self-oscillation, click-free time sweeps, SW1 off and bypass spillover, reverse (time-reversed repeats, click-free direction switches, SW1 off keeping the direction); `reverb_mix` 0.
- `tests/crush_test.cpp`: the digital type's bitcrush (the staircase smoothed with open off and left with it on) and its bit-depth hysteresis, open letting a 9 kHz note fold to 1 kHz, the clock halving the hold rate at 2 s, bit inversion without DC and exactly 0 once the repeats are gone, wrap folding an overload to the other rail, `delay_crush_scope` all crushing the dry and the reverb's input on the mix and dry taps (repeats leaving the dry exactly the input) and keeping the loop clean (the second repeat half the first).
- `tests/tape_test.cpp`: age 0 bit-identical to the clean digital delay and full age colored, wow and flutter within bounds, the time glide at any age (digital at pitch), a 100 → 2000 ms change landing on the new time, a bounded loop at full age, hiss level and gate, dropouts only at high age (these with `delay_tape_scope` repeats); scope all taking a 5 kHz note on the dry down through the HF loss (repeats leaving the dry exactly the input), wow wobbling the dry, and the loop clean (the second repeat half the first).
- `tests/bbd_test.cpp`: the BBD at color 0 keeping a 300 Hz note within 1.5 dB and 8 kHz 26 dB down, full color folding a 2.5 kHz note to 913 Hz at 600 ms (no alias at color 0), the noise rising with color and exactly 0 once the repeats are gone, a bounded loop at full color, the time glide; the clock LFO bending a 220 Hz note by ±4 × rate × depth × delay along its triangle (1 and 2 Hz at 2 %, 500 ms), holding the read speed to 0.75–1.25 (and reaching it) at 25 %, 10 Hz and 2 s, and leaving tape and digital bit-identical (these with `delay_bbd_scope` repeats); scope all keeping a 300 Hz note on the dry within 1.5 dB and 8 kHz 26 dB down (repeats leaving the dry exactly the input), and the loop clean.
- `tests/hall_test.cpp`: RT60 at 1 kHz tracking `hall_rt_mid` within 15 % at 0.5, 2 and 6 s, bounded decay at all 256 loop-parameter corners, L/R decorrelation, no fixed comb (a static delay path) in the folded tail, SHAPE/SPREAD build-up, SIZE glide keeping the tail, pre-echo sides.
- `tests/plate_test.cpp`: RT60 tracking MID DECAY within 15 %, LF DECAY lengthening the bass, bounded decay at all 256 corners, L/R decorrelation, SIZE glide keeping the tail, STOP DECAY cutting the free tail by 30 dB, `plate_space` lengthening the decay along its law.
- `tests/canyon_test.cpp`: RT60 rising with FEEDBACK, bounded decay at all 256 corners, L/R decorrelation, MIX 0 passing the trimmed input, `canyon_space` lengthening the decay, DELAY GLIDING turned on mid-cross-fade continuing from the new base.
- `tests/algo_switch_test.cpp`: the SW3 switch: silent during the clear, the new algorithm answering exactly as one selected at power-on, a return during the fade-out keeping the tail, block-size independence across switches.
- `tests/engine_test.cpp`: the chain through `btdsp::Engine`: `routing` wet reverberating only the delay wet, mix the delay mix (silent before the first repeat), dry leaving the repeats dry; bypass in every routing passing new input at unity while the tails die out; the equal-power crossovers (knob 1 at 50 %, knob 2 at 75 %); each algorithm's wet level matching the input.
- `tests/eq_test.cpp` (with the model `tests/eq_model.h`): the model's laws (bell gain at f0, shelf half gain at f0, exact cut/boost inverse) and `docs/EQ.md`'s setting A, the digital EQ within 0.05 dB of the model (each band at its frequency range's ends and default, ±12 dB, Q 0.4 and 8), cut mirroring boost within 0.1 dB, bit-exact identity, a section starting from 0 dB without a click, robustness under random changes, and its placement in the engine (pre, post, bypass).
- `tests/saved_store_test.cpp`: the saved values' payload round trip, decoding dropping switch parameters and non-finite values, a flipped payload byte failing the CRC, the newest of two slots (with its sequence number), and the 528-byte legacy payload read with a knob assignment of 127 moved to none.

`make host` also builds `build/host/render IN.wav OUT.wav [--set NAME=VALUE]... [--block N] [--tail SECONDS]`: the DSP chain offline (96 kHz input, channel 0; mono float32 output). Each `--set` is applied in order as a MIDI write is on the pedal, so `--set hall_space=N` (or `plate_space`, `canyon_space`) also sets its linked parameters. `make midi-doc` regenerates `docs/MIDI.md` from the table.

Reverb measurements (host only, outputs in the git-ignored `renders/`): `uv run tools/stimuli.py [--guitar WAV]` writes the stimuli (impulse, 50 ms noise burst, 1 kHz sine, and `guitar` from a 96 kHz WAV you supply; all at 96 kHz) to `renders/stimuli/`; `uv run tools/features.py report WAV --stimulus S` and `compare WAV_A WAV_B --stimulus S` measure RT60 per octave, third-octave balance, pre-delay, echo density and modulation depth.

## Build, flash, log (from the repo root)

Needs the Arm GNU Toolchain 13.3.rel1 (`arm-none-eabi-gcc`) on `PATH`, `dfu-util`, and `uv` for `usb_log.py` (pyusb). Output: `daisy/build/hothouse/bad_tape.bin` (`BOARD=hothouse`, the default and only board; any other value is an error).

```sh
git submodule update --init --recursive && make -C daisy/libDaisy   # once, and after a submodule update
make -C daisy -j
daisy/tools/flash_when_ready.sh          # daisy/build/hothouse/bad_tape.bin
uv run daisy/tools/usb_log.py 30         # the pedal's log
uv run daisy/tools/usb_log.py 5 --dfu    # restart the running pedal into DFU
```

The firmware runs from SRAM (`APP_TYPE = BOOT_SRAM`) and is flashed to QSPI at 0x90040000, so the Seed needs the Daisy bootloader. Once per Seed: hold BOOT, tap RESET, release BOOT (ST's system DFU), then `make -C daisy program-boot` (libDaisy's `dsy_bootloader_v6_4-intdfu-2000ms.bin`).

`flash_when_ready.sh` needs no button presses while the pedal runs: with no DFU device present it sends the pedal the link's DFU SysEx in both link versions (`usb_log.py --dfu`, `F0 7D 62 01 06 F7` and `F0 7D 62 02 06 F7`; each firmware ignores the other version's frame). The pedal logs `dfu reset` and restarts into the Daisy bootloader's DFU mode with no timeout. Close any other program holding the MIDI port first. For other firmware, enter DFU by hand: hold BOOT, tap RESET, release BOOT.

The bench and the latency image are further images from the same sources (`APP=bench`, `APP=latency`, each with its own build directory); `make -C daisy APP=foo` stops with `APP must be bad_tape, bench or latency`. They speak the same link (HELLO, DFU), so `flash_when_ready.sh` moves between the images with no button presses:

```sh
make -C daisy APP=bench -j
daisy/tools/flash_when_ready.sh daisy/build/hothouse-bench/bad_tape_bench.bin
uv run daisy/tools/usb_log.py 40         # one bench pass every ~31 s
make -C daisy APP=latency -j             # needs a cable from the left output to the left input
daisy/tools/flash_when_ready.sh daisy/build/hothouse-latency/bad_tape_latency.bin
uv run daisy/tools/usb_log.py 20         # one latency pass every ~17 s
make -C daisy -j && daisy/tools/flash_when_ready.sh   # back to the pedal
```

## Performance

The budget is one block period: a 1/3 ms block (32 samples at 96 kHz) = 160 000 cycles at 480 MHz; ‰ below is permille of it, as in the status line's `load`. With every EQ section live the pedal's worst case must stay under 850 ‰.

The worst-case settings (`kWorst` in `src/bench.cpp`; the pedal runs set the same values with `usb_log.py --set`): `delay_time` 2000, `delay_feedback` 110, `delay_type` 0 and `delay_color` 100 (the BBD, the heaviest type, at full color), `delay_mix` 100, `reverb_mix` 100; `plate_size` 100, `plate_chorus` 100, `plate_mid_decay` 20; `hall_size` 39, `hall_rt_mid` 20, `hall_wander` 100, `hall_spin` 100, `hall_spread` 100, `hall_echo_level_1..4` 0; `canyon_master_delay` 100, `canyon_master_depth` 100, `canyon_feedback` 100; the EQ on, post, with every section live: `eq_enable` 1, `eq_position` 1, `eq_low_gain`, `eq_mid_gain_1..3` and `eq_high_gain` 12 (frequencies and Qs at their defaults: they do not change a section's cost).

### Bench (per stage)

`src/bench.cpp` (`APP=bench`) runs no audio. It brings up the board, USB and SDRAM with the pedal's startup (`btfw::Start`: FPSCR.FZ, the MPU) and the same memory placement (DSP objects in DTCM, the delay line and the pool's slow region in SDRAM, the fast region in D2, the timed block functions in ITCM like `AudioCallback`), then times each stage with the DWT cycle counter on `kBlockSize` blocks of uniform noise (±0.5, `XorShift32`, 64 blocks cycled): 200 ms of warm-up blocks, then 2 s of measured blocks, interrupts masked inside each measured block. Its log lines:
- at power-on and on HELLO: `bad_tape_bench hothouse sysclk N MHz <build id>`;
- per pass: `bench pass N budget B cycles`, one `bench NAME cycles avg A max M permille avg P.p max Q.q` per stage, then `bench pass N done`; passes repeat every ~31 s (2 s apart).

The stages:
- `controls`: `hw.Process()`, the three toggle filters and the six knob reads (the callback's control work).
- `delay fwd|rev bbd|tape|digital`: `btdsp::Delay` alone at 2000 ms and 110 % feedback, forward or reverse, each type at full color (digital with all four crush options on), its color in the loop (scope repeats); `delay fwd bbd|tape|digital all`: forward with every scope all, `Delay::ColorInput` timed too.
- `plate|hall|canyon default|worst`: each reverb alone on a zeroed pool at `kReverbRate` (48 kHz, half a block's samples per block), at its table defaults or its `kWorst` settings; `worst sdram`: the same with the fast region in SDRAM (the placement comparison below). `resample`: `HalfRateReverb` around a reverb that costs nothing (the decimator and interpolator alone).
- `eq worst`: `btdsp::Eq` alone at its `kWorst` settings (all five sections live); `eq sweep`: the same with every gain flipped between ±12 dB and every frequency between its range's ends every 20 ms, so every section redesigns its coefficients on every control tick.
- `engine fwd|off|rev plate|hall|canyon`: `btdsp::Engine` (the whole chain) at `kWorst` for each SW1 × SW3 position.
- Control paths, all at `kWorst`: `engine sw3 switch` (reverse, Hall → Plate at measured block 0: the fade-out, the ~15 ms pool clear and the fade-in all inside the run); `engine time sweep bbd|tape|digital` (forward, Hall, `delay_time` 10 ↔ 2000 ms every 20 ms, each type at full color: BBD and tape glide the head, digital crossfades two heads); `engine eq sweep rev plate` (reverse, Plate: the pedal's worst row, the EQ's five gains flipped between ±12 dB every 20 ms). Maximum feedback (110 %) is in every engine stage.

Measured 2026-11-12 at 96 kHz, 32-sample blocks, with every color's scope at its default, all (3 passes; avg the median pass, max the highest), ‰ of the 160 000-cycle block:

|Stage|avg ‰|max ‰|
|---|---|---|
|`controls`|4.5|8.3|
|`delay fwd bbd`|184.6|187.6|
|`delay rev bbd`|235.5|269.0|
|`delay fwd bbd all`|233.3|237.1|
|`plate worst`|245.6|257.2|
|`hall worst`|220.0|248.2|
|`canyon worst`|108.8|115.2|
|`resample`|22.5|22.7|
|`eq worst`|62.9|63.2|
|`eq sweep`|110.6|120.4|
|`engine fwd plate`|629.1|640.3|
|`engine fwd hall`|604.8|632.3|
|`engine fwd canyon`|489.7|495.3|
|`engine rev plate`|674.9|704.6|
|`engine rev hall`|650.4|690.9|
|`engine rev canyon`|535.1|564.3|
|`engine sw3 switch`|674.1|704.6|
|`engine time sweep bbd`|625.7|654.8|
|`engine eq sweep rev plate`|724.0|759.2|

At 96 kHz the delay, the input colors, the EQ and the mixing cost twice their 48 kHz share; the reverbs run at 48 kHz (`HalfRateReverb`), so their rows are unchanged, and the resampler adds 22.5 ‰. A color stage that is off (the loop's and the input's crush and tape with the BBD selected) runs no filters and starts them warm (`delay fwd bbd all` 321.6 → 215.3, the `engine` rows −110 ‰). Another, output-identical, cut the Plate: its tank and tap reads are range-proved at compile time (`PlateReadsInRange`) and read unclamped, its 30 output taps are unrolled with constant lines and delays, and its input diffusers read at their integer lengths without interpolation (`plate worst` 327.7 → 245.2, the `engine * plate` rows −83 ‰; the bench image calls `Line::Read` out of line from `Plate::Process`, the pedal image inlines it). Then the Hall, bit-identical: its tank and output-tap reads are range-proved (`HallReadsInRange`, with the SIZE glide held to the table range) and read unclamped, its two tank halves are unrolled with constant lengths, its input diffusers read at their integer lengths, and a control tick whose inputs (the smoothed values, SIZE, SPREAD, the echo targets) are unchanged skips the coefficient recomputation (about 20 `expf` and 2 `powf`); and, by rounding only, its SPREAD step multiplies by 1/3 and its SPREAD weights age by one `expf` and a running product (`hall worst` 257.7 → 211.7, `engine fwd hall` 668.0 → 622.2, `engine rev hall` 719.6 → 671.8). A reciprocal in `Crossfade` (shared by the Hall, the Canyon, the digital heads and the crush mask) was measured and not kept: −8 ‰ on `engine fwd hall` but +6 to +12 ‰ on the standalone delay stages and +2 to +4 ‰ on `engine rev canyon` and `engine eq sweep rev plate`, a code-layout effect. The blocks run from ITCM, and the bench runs the pedal's MPU setup.

### Reverb pool placement

The fast region (then 61 440 floats, now 58 076: the largest algorithm's diffusers and tank lines, the Canyon's) in D2 SRAM against the same region in SDRAM, each reverb alone at `kWorst`, cycles per 48-sample block (measured with the chain at 48 kHz; the reverbs still run at 48 kHz):

|Algorithm|fast in D2 avg|max|fast in SDRAM avg|max|SDRAM vs D2|
|---|---|---|---|---|---|
|plate|155 854|158 923|206 843|229 190|+33 %|
|hall|108 741|116 227|131 903|143 471|+21 %|
|canyon|50 400|52 118|65 866|74 779|+31 %|

D2 wins for all three (SDRAM costs 21–33 % more), so the fast region stays in D2 (`.d2_bss`); only the long input/predelay lines (the slow region) stay in SDRAM. A DTCM tier for the Hall's short lines (its four diffusers and four loop allpasses, 11 759 floats = 47 KB of DTCM, the two loop lines left in D2) was built and measured on the 2026-11-12 Hall (with its bit-identical cuts from "Bench"): `hall worst` 210.3 ‰ against 214.9 ‰ with the same lines in D2, a 4.6 ‰ gain, under the 5 ‰ it had to clear for its 47 KB of DTCM, so it was removed. No algorithm's whole fast set fits DTCM's ~100 KB free (Hall 32 248 floats = 126 KB; Plate 35 218; Canyon 58 076).

### Pedal (SW1 × SW3)

`kWorst` (the EQ on, post, all five sections live), `delay_mode` and `reverb_algo` set over SysEx, 12 s of log per row (`usb_log.py`), the first two status lines of each row dropped; measured 2026-11-12 at 96 kHz with no input (the cost does not depend on the signal) and every color's scope at its default, all (the BBD on the input). avg is the range of the remaining lines, max the highest. `underruns 0` on every row.

|SW1|SW3|avg ‰|max ‰|
|---|---|---|---|
|fwd|plate|602-616|634|
|fwd|hall|611-613|648|
|fwd|canyon|501-501|509|
|off|plate|602-603|615|
|off|hall|612-613|648|
|off|canyon|501-502|509|
|rev|plate|657-658|708|
|rev|hall|666-668|729|
|rev|canyon|555-556|601|

EQ gains moving on the worst row (reverse, Plate): the five gains flipped together between −12 and +12 dB by SysEx 12 times, each followed by 2 s of log: highest `max` 751 ‰ (700–717 ‰ on most lines while the gains glide), `load avg` up to 691 ‰, no underrun. On the pedal the callback costs more than the bench's `engine` stage: the controls, the LEDs' `log10f`/`cbrtf`, the load meter and interrupt entry run once per block. Hot code in ITCM saves 1–3 ‰ of average and no maximum: the load is the data side, the reverb lines and the delay line read through the D-cache from D2 and SDRAM.

### Block size

32-sample blocks at 96 kHz (`btdsp::kBlockSize`) keep the 1/3 ms block period of the earlier 16 samples at 48 kHz, so the callback's per-block work, the knob filters' rate (`AudioCallbackRate`, 3 kHz), `kToggleStableBlocks` (90 blocks: the 30 ms toggle hysteresis), the FS2 debounce counts, the underrun threshold and the bench's budget are unchanged, and the latency falls to 1.031 ms ("Latency"). At 48 kHz, 16-sample blocks against 48-sample ones left the DSP's per-sample cost unchanged (the bench's engine rows moved −8 to +3 ‰) and raised the pedal's average load 21–45 ‰, because the callback's per-block work ran 3× as often. The DSP is block-size independent (`DspInvariants`, `AlgoSwitch`, `HalfRateReverb`); the control ticks are 3 kHz in both domains (`kControlInterval` 32 at 96 kHz, `kReverbControlInterval` 16 at 48 kHz).

### Latency

`src/latency.cpp` (`APP=latency`) measures the audio round trip through a cable from the left output to the left input. The pedal adds no delay of its own (no lookahead, no buffering past libDaisy's), so the round trip is its input-to-output latency: a click leaves through the output DMA and the DAC and returns through the ADC and the input DMA, the chain a guitar sample takes. For each sample rate and block size it restarts the audio, waits 500 ms, then sends 5 one-sample clicks (0.25 full scale), each 200 ms apart. It reports the delay to the input's largest excursion (the peak, parabola-refined: the codec filters' group delay) and to the first sample above a tenth of it (the onset). Log lines:
- at power-on and on HELLO: `bad_tape_latency hothouse seed <version> <build id>`;
- per trial: `lat SR B t T peak P.PP smp U us onset O smp amp A noise N` (A and N in 1/10 000 of full scale; A signed);
- per configuration: `lat SR B peak min P.PP max Q.QQ smp U..V us`; per pass: `lat pass N done seed <version>`; passes repeat every ~17 s.

Measured 2026-11-11 on the HotHouse (Seed 1.2, PCM3060), 3 passes; 96 kHz with 32-sample blocks measured 2026-11-12, 4 passes, and the other rows repeated to 0.01 sample. Every trial of a configuration gave the same peak to 0.01 sample:

|Rate|Block|Peak (samples)|Peak (ms)|Onset (samples)|2 blocks (ms)|Rest (ms)|
|---|---|---|---|---|---|---|
|48 kHz|48|130.72|2.723|128|2.000|0.723|
|48 kHz|24|82.72|1.723|80|1.000|0.723|
|48 kHz|16 (the pedal until 2026-11-12)|66.72|1.390|64|0.667|0.723|
|48 kHz|8|50.72|1.057|48|0.333|0.723|
|96 kHz|48|130.96|1.364|130|1.000|0.364|
|96 kHz|32 (the pedal)|98.96|1.031|98|0.667|0.364|
|96 kHz|24|82.96|0.864|82|0.500|0.364|

The round trip is exactly two blocks plus a fixed ~35 samples (34.72 at 48 kHz, 34.96 at 96 kHz): libDaisy's double-buffered DMA holds one block on each side, and the rest is the PCM3060's converter filters and the analog path, fixed in samples, so its time halves at 96 kHz. The click returns inverted at 0.16 of full scale (the codec filters spread its energy over several samples, so this is not the path's gain). 96 kHz with 48-sample blocks (1.36 ms) is faster than 48 kHz with 24 (1.72 ms) because the converters' delay halves too. The pedal now runs 96 kHz with 32-sample blocks (the reverbs at 48 kHz inside `HalfRateReverb`, so their lines still fit D2): 1.031 ms, 0.36 ms less than its earlier 48 kHz with 16-sample blocks.

## Memory map

The pedal image (2026-11-13, 96 kHz, 32-sample blocks; `make -C daisy -j` `--print-memory-usage`, `arm-none-eabi-size -A` and `arm-none-eabi-nm -C -S --size-sort daisy/build/hothouse/bad_tape.elf`):

|Region|Used|Contents|
|---|---|---|
|SRAM (AXI)|263 012 B of 480 KB|`.text` 163 224 B: libDaisy, the main loop and the host session, the cold DSP code (`Init`, `Reset`, `Snap`, `SetParam`, `Engine::Activate`, `Eq::Start`), `libm`/`libc` (with the double-precision functions the EQ's coefficient designs call); `.data` 1 584 B; `.bss` 35 252 B (including the saved values and their slot buffer, 1 728 B, and the WATCH and SAVED reply state); the ITCM and DTCM load images|
|DTCM|15 288 B + the 16 KB stack reserve of 128 KB|`.dtcm_data`: the engine (`gEngine`, 15 120 B: its parameter snapshot, the delay's state with the reverse grains, the tape and BBD stages and their input copies (the input tape's 2 048-float wow line), the BBD clock LFO and the two crushes (loop and input), the three reverbs' state and the half-rate resampler (`HalfRateReverb`, 270 floats of history and FIFO), the EQ's state and coefficients (five float sections with their coefficient ramps, double glides), the mix and routing gain smoothers and the SW3 switch), the callback's LEDs (88 B) and load meter (28 B), the toggle filters (24 B), the LED2 envelope, the peak accumulator, `gBlockTicks` and `gParamSeen`; `.reserved_for_stack` (16 384 B): the stack grows down from 0x20020000, and the placed state plus the reserve over 128 KB is a link error|
|ITCM|46 912 B of 64 KB|`.itcm_text`, after the 256-byte guard at 0x0: `btdsp::Engine::Process` 10 012 B (the input colors, `Delay::ColorInput`, `HalfRateReverb::Process` and the EQ's per-sample sections inlined), `btdsp::Delay::Process` 7 764 B (the BBD clock LFO, the reverse grains, the loop's BBD chip and the loop crush with its bit inversion inlined; the input's chip is inlined into `Engine::Process`), `btdsp::Tape::Tick` 712 B (wow, flutter and dropouts, once per control tick), `btdsp::Crush::Set` 1 072 B, `btdsp::Tape::Set` 784 B, `btdsp::Bbd::Set` 600 B, `btdsp::Hall::Process` 7 804 B (its two tank halves unrolled), `btdsp::Hall::Update` 2 236 B, `btdsp::Plate::Process` 8 228 B (the 30 output taps unrolled), `btdsp::Canyon::Process` 3 458 B, the EQ's control tick (`btdsp::Eq::Control` 1 086 B, `noinline` with its coefficient designs so `Engine::Process` does not save d8-d14 per sample), `AudioCallback` 2 684 B, and 25 veneers (8 B each) to `libm`/`libc`, libDaisy and the cold code in SRAM. Every other `BT_ITCM` function (`src/dsp/*.h`: the blocks, the crush's per-sample stages, the tape stage, the BBD clock LFO, the reverse grains, each reverb's `Tick`/`Update`/`Read`/`TapRead` helpers, the pool clear, the EQ's smoothers, sections and designs) is inlined into these.|
|RAM_D2|232 304 B of 256 KB|`.d2_bss`: the reverb pool's fast region (`gReverbFast`, 58 076 floats: the largest algorithm's, the Canyon's; the Hall uses 32 248, the Plate 35 218), cleared by `btdsp::Engine::Init` and in slices on every SW3 switch. The kept placement: Performance, "Reverb pool placement".|
|RAM_D2_DMA|17 692 B of 32 KB|`.sram1_bss`: libDaisy's audio and ADC DMA buffers|
|SDRAM|2 231 568 B of 64 MB|`.sdram_bss`: the delay line (`gDelayMem`, 2^19 floats = 2 MiB, 5.46 s at 96 kHz), cleared by `btdsp::Delay::Init`; the reverb pool's slow region (`gReverbSlow`, 33 604 floats: the Hall's 700 ms input line, the largest; the Plate's predelay line uses 9 604, the Canyon's sixth line 11 524)|
|QSPI|8 KB of 8 MB, past the image|the saved power-up values (`src/saved_store.h`): two 4 KB slots at 0x903FE000 and 0x903FF000, 16-byte header + 1 056-byte payload each (the 528-byte payload of link-version-01 firmware is still read), read memory-mapped (MPU region 4, `src/firmware.h`) and written by `QSPIHandle` from the main loop (75–77 ms per SAVE/CLEAR, audio running). The image itself starts at 0x90040000.|

`BT_ITCM` (`src/memory.h`) gives each function its own input section (`.itcm_text.N`, all collected into `.itcm_text`): GCC rejects one named section holding both inline (COMDAT) header functions and an ordinary function such as `AudioCallback` in one translation unit. An inlined copy runs from its caller's section, so every caller of a `BT_ITCM` function is itself `BT_ITCM` or cold. GCC 13 drops the section attribute from template instantiations, so the hot templates (`SvfT`, `DbToGain`, `StepSmoothers`, `AlgoParams::Tick`, `ReverseGrains::Process`) are `always_inline`: an out-of-line copy would run from SRAM.

The bench image (`daisy/build/hothouse-bench/bad_tape_bench.elf`, 96 kHz, 32-sample blocks): DTCM 38 184 B + the 16 KB stack reserve (the engine, a separate delay, the three reverbs and the EQ, the resample stage's `HalfRateReverb`, the toggle filters, 64 noise blocks and the output buffers), ITCM 40 376 B (the timed block functions included), D2 227 KB (`gReverbFast`), SDRAM 2 406 KB (the delay line, the slow region and `gFastInSdram`, the 227 KB comparison region).

## Reverb character

Knob 3 space macros, Plate and Canyon (host renders of the impulse, `--set reverb_algo=0|2 --set plate_space=N|canyon_space=N --set delay_mix=0 --set reverb_mix=100 --tail 10`, `features.py report`):

|Space|0 %|25 %|50 %|75 %|100 %|
|---|---|---|---|---|---|
|Plate RT60 1 kHz (s)|0.38|0.82|1.82|4.02|8.82|
|Plate law `plate_mid_decay` (s)|0.36|0.80|1.80|4.02|9.00|
|Plate NED mixing time (ms)|82|122|248|301|297|
|Canyon RT60 1 kHz (s)|0.49|2.02|7.47|24.3|36.7|
|Canyon law `canyon_feedback` (%)|60|83.1|95|99.4|100|
|Canyon NED mixing time (ms)|182|211|278|332|368|

RT60 rises with space on both; the Canyon's values above 10 s are the fitted slope of a 10 s render.

The Canyon's sweep changes the read distance of lines inside the feedback loop, so each one bends the pitch by up to π × depth × rate (depth in s, rate in Hz), and the bends add on every pass round the loop. At the defaults (`canyon_master_rate` 15, `canyon_depth_1..6` 5–10 ms) the steepest line (line 6) peaks at about 6.5 cents over a 4–11 s cycle.

## Open questions

1. The `nm -u` heap check covers the project's own objects (`daisy/build/hothouse/*.o`). The linked image still contains newlib's `_malloc_r`, pulled in by `vsnprintf` but not called for a fixed buffer.
