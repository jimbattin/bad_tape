# Bad Tape: EQ specification (five-band parametric)

This is the specification for the pedal's EQ stage: its controls and ranges, the exact linear model that defines its curves, a reference response, and the behaviour an implementation must show. It is written so the EQ can be implemented and tested from this file alone.

Status: built (`daisy/src/dsp/eq.h`). The EQ is off by default; `eq_position` places it before the delay or after the mix.

## 1. Controls

Five bands: a low shelf, three bells and a high shelf, with frequency ranges for instruments from baritone guitar to mandolin. The ranges overlap, so two or three bands can work on the same region. The EQ has no knob or switch on the pedal: it is set from the editor (and by NRPN and SysEx like every other parameter), or from a knob slot assigned to one of its parameters, and every parameter is saveable as a power-up value.

|Id|Name|Unit / values|Min|Max|Default|Taper|Smoothing|
|---|---|---|---|---|---|---|---|
|74|`eq_enable`|0 off, 1 on|0|1|0|enum|stage|
|75|`eq_position`|0 pre, 1 post|0|1|0|enum|stage|
|76|`eq_low_gain`|dB|-12|12|0|linear|20 ms|
|77|`eq_low_freq`|Hz|40|600|100|log|20 ms|
|78|`eq_mid_gain_1`|dB|-12|12|0|linear|20 ms|
|79|`eq_mid_freq_1`|Hz|40|800|250|log|20 ms|
|80|`eq_mid_q_1`|Q|0.4|8|1|log|20 ms|
|81|`eq_high_gain`|dB|-12|12|0|linear|20 ms|
|82|`eq_high_freq`|Hz|1500|16000|6000|log|20 ms|
|83|`eq_mid_freq_2`|Hz|200|5000|1000|log|20 ms|
|84|`eq_mid_q_2`|Q|0.4|8|1|log|20 ms|
|85|`eq_mid_gain_2`|dB|-12|12|0|linear|20 ms|
|128|`eq_mid_gain_3`|dB|-12|12|0|linear|20 ms|
|129|`eq_mid_freq_3`|Hz|800|12000|3000|log|20 ms|
|130|`eq_mid_q_3`|Q|0.4|8|1|log|20 ms|

"stage" means the EQ handles the change itself (`smoothMs` 0 in `params.h`): a 20 ms crossfade. With the defaults the EQ is off; on with every gain at 0 dB it is the identity.

- A bell's frequency is its centre: the bell reaches its gain exactly there. Its Q sets the width (RBJ definition: Q 0.4 is about 3.4 octaves between the half-gain points, Q 1 1.9 octaves, Q 8 0.18 octaves).
- A shelf's frequency is its half-gain point: the shelf reaches half its gain (in dB) there and its full gain far beyond it. The shelves' slope is fixed at Q 0.7071 (the steepest without overshoot).
- `eq_enable` off is a true bypass of the stage.

## 2. Signal model (normative)

The EQ is linear, time-invariant when its controls are still, mono and minimum-phase, with no latency. Its response is the product of five sections, in this order (the order matters only for rounding and while parameters move):

```
H(z) = LOW(z) · BELL1(z) · BELL2(z) · BELL3(z) · HIGH(z)
```

Each section is an analogue prototype (the RBJ Audio EQ Cookbook) mapped by the bilinear transform prewarped at the section's frequency `f0`. With `fs` the sample rate (96 kHz), `A = 10^(gain/40)` and `Q` the section's Q (the shelves: 1/√2), the response at frequency `f` is the prototype at

```
s = j · tan(π f / fs) / tan(π f0 / fs)
```

|Section|Prototype|
|---|---|
|Bell|`(s² + s·A/Q + 1) / (s² + s/(A·Q) + 1)`|
|Low shelf|`A · (s² + (√A/Q)·s + A) / (A·s² + (√A/Q)·s + 1)`|
|High shelf|`A · (A·s² + (√A/Q)·s + 1) / (s² + (√A/Q)·s + A)`|

Consequences, all exact:

- A bell's gain at `f0` is `gain`; a shelf's gain at `f0` is `gain / 2`. A low shelf's gain is `gain` at DC, a high shelf's at Nyquist.
- A section at 0 dB is exactly 1.
- Negating a section's gain gives its exact inverse response (`H(−g) · H(g) = 1`).

## 3. Reference response (model)

Setting A: low +6 dB at 80 Hz; bell 1 −4 dB at 250 Hz, Q 1.4; bell 2 +3 dB at 1.2 kHz, Q 0.7; bell 3 +5 dB at 3.5 kHz, Q 2; high −6 dB at 8 kHz. Values in dB at 96 kHz.

|30 Hz|50 Hz|80 Hz|120 Hz|250 Hz|500 Hz|1 kHz|1.2 kHz|2 kHz|3.5 kHz|5 kHz|8 kHz|12 kHz|16 kHz|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|+5.85|+5.09|+2.78|+0.42|−3.66|+0.31|+2.79|+3.09|+2.64|+5.48|+1.14|−2.53|−4.89|−5.63|

## 4. Required behaviour

Accuracy:

- The stage's magnitude is within ±0.05 dB of the model from 20 Hz to 20 kHz at 96 kHz, controls still, for every band at the ends and the default of its frequency range, at gains ±12 dB and (bells) Q 0.4 and 8, and for setting A.

Symmetry:

- Negating every gain gives the inverse response: the two magnitudes sum to 0 dB within 0.1 dB at every frequency.

Identity:

- With `eq_enable` off, or with every gain at 0 dB, the stage's output is bit-identical to its input once any transition has finished. That includes a gain at the 14-bit code nearest 0 dB (MIDI's grid does not hold 0 dB exactly) and a gain returned to 0 dB. The default settings are in this state.

Changes while audio plays:

- Gains glide in dB, frequencies in log frequency and Qs in log Q, each with 20 ms smoothing, with no zipper noise.
- A jump of any parameter between its extremes, including `eq_enable`, passes the engine's no-click invariant (`NoClickOnParamJump`, `tests/dsp_invariants_test.cpp`).
- The stage stays stable and bounded for any sequence of parameter changes, with identical output across block sizes.
- No NaN or Inf, and no denormal stall, under silence, full-scale noise or DC.

Level:

- Nothing inside the EQ clips. One band adds at most +12 dB. Bands that overlap add in dB: five boosts stacked near 700 Hz (low at 600 Hz, the three bells at 800 Hz Q 0.4, high at 1.5 kHz) reach +40.6 dB.

Placement:

- `eq_position` pre: the first stage, `x = EQ(in)` feeds the delay, both dry paths and the reverb, as in the signal-flow formulas of `daisy/README.md` ("Signal flow"). Post: the last stage, on the mixed output (`bad_tape.cpp` copies L Out to R). A change fades the EQ out over 20 ms, moves it and fades it back in.
- In FS1 bypass the output is the unequalised input plus the tails; post, the tails pass through the EQ. LED2 keeps metering the raw input.

Cost:

- Five second-order sections, mono. The pedal's worst-case load with the EQ on must stay under the 85 % ceiling (`daisy/README.md`, Performance).

## 5. As built

- `daisy/src/params.h`: ids 74 to 85 (group `eq`), which held the earlier Neve 31102 model's parameters, are reused: `eq_enable`, `eq_position`, `eq_low_gain`, `eq_low_freq`, `eq_mid_gain_1`, `eq_mid_freq_1`, `eq_mid_q_1`, `eq_high_gain`, `eq_high_freq`, `eq_mid_freq_2`, `eq_mid_q_2`, `eq_mid_gain_2`; the third bell is appended at 128 to 130 (`eq_mid_gain_3`, `eq_mid_freq_3`, `eq_mid_q_3`). Power-up values saved by Neve-era firmware are read into the reused ids and clamped into the new ranges: old gains (and the old trim at 85) land on gains, old enum positions on frequencies and Qs (their minimum, inaudible while that band's gain is 0).
- `daisy/src/dsp/eq.h`: each section is A. Simper's linear trapezoidal state-variable filter (`btdsp::SvfT<float>`, `blocks.h`) mixed to the prototype (`eq::Design`: output `m0·x + m1·band + m2·low`), which is exactly the bilinear transform of section 2. Coefficients are designed in double, once per 32-sample control tick for a section whose glides moved; the sections filter in 32-bit float. `daisy/src/dsp/engine.h` places the stage.
- `tests/eq_model.h`: the model of section 2 in closed form; `tests/eq_test.cpp` checks it against sections 2 and 3 and the stage against it (section 4).
- A gain within one 14-bit code step of 0 dB counts as 0 (`Eq::SetParam`), and a section whose gain rests at 0 dB for 100 ms stops: the identity of section 4 needs an exact 0.
- `editor/src/eqModel.ts`: the same model for the editor's response graph (`editor/src/components/EqGraph.tsx`); `editor/src/eqModel.test.ts` checks it against section 3.
- Measured digital error against the model at 96 kHz, worst over section 4's cases and a 1/6-octave grid from 20 Hz to 20 kHz: 0.0022 dB (bell 1, +12 dB at 40 Hz, Q 8, at 40 Hz).
- A coefficient change ramps linearly across its 32-sample control tick (`SectionF::RampTo`): stepping the coefficients once per tick clicked on fast Q and gain jumps (`NoClickOnParamJump`). A section starting from 0 dB is set to its identity coefficients and ramps from there to its glide's first step: set there at once, the high shelf (whose `m0 = A²` scales the whole signal) stepped the output (`BandStartingFromFlatDoesNotClick`).
- Cost on the Seed (`daisy/README.md` Performance, 2026-11-12, 96 kHz, 32-sample blocks): the EQ alone with all five sections live 62.9 ‰ of the block period, 110.6 ‰ average and 120.4 ‰ at most while every gain and frequency moves. On the pedal the worst row (reverse delay, the Hall) is `load avg 666-668 max 729 permille`, and 751 ‰ at most while the five gains flip between ±12 dB on the reverse/Plate row: under the 850 ‰ budget.
