// The Plate (SW3 Down): a native reverb aiming for the character of the Lexicon 224XL Rich Plate: a figure-8 plate
// tank under the 224XL's controls. Lengths are 48 kHz samples scaled by SIZE (s = 0.5 + size / 140: 1 at the default
// 70 %); ms parameters convert with fs, and every read distance stays inside its line's capacity (PlateReadsInRange;
// the predelay is clamped).
//
// Input: HF BANDWIDTH (6 dB/oct lowpass) into a predelay line, read at PREDELAY x kPlateInGain, then three series
// allpasses per side (L 45/143/362, R 52/177/302, gain -0.8125 x DIFFUSION / 80 up to 80 %, on to -0.9 at 100 %): a
// mono input decorrelates.
// Tank, a figure-8 of five segments, every loop allpass at gain c x opt (DEFINITION: c = 0.6 - 0.4 x def / 100):
//   side A: x_L - B2(D2B) -> AP1A -> D1A -> TREBLE DECAY one-pole -> A1 -> AP2A -> D2A -> A2 -> AP3A -> D3A
//   side B: x_R + A3(D3A) -> AP1B -> D1B -> TREBLE DECAY one-pole -> B1 -> AP2B -> D2B -> (B2 into side A)
// Segment k (AP + D, T_k samples x s) is a crossover shelf y -> g y + (gl - g) LP(y) with LP a one-pole at CROSSOVER,
// g = 10^(-3 T_k / (MID DECAY fs)) and gl the same with LF DECAY: its magnitude never exceeds max(g, gl) < 1, so the
// loop is stable for any pair. CHORUS sweeps D1A and D1B by up to kPlateChorusMax samples with two slow sines.
// Output per side: the 15 output taps (kPlateTapTable, at their lengths x s) plus DEPTH's attack weight x the
// diffused input (kPlateAttack x 2 (1 - depth / 100): the early/late balance, 0.68 at the default 50 %), a
// kPlateToneHz one-pole and kPlateOutGain.
// DECAY OPT: while the input gate (kPlateGateDb, kPlateGateReleaseS release) is closed the loop allpass gains fall
// linearly to kPlateOptFloor of themselves over kPlateOptFallS (less ringing density in the free tail) and rise back in
// kPlateOptRiseS once it opens. STOP DECAY: once the gate has been closed for STOP DELAY, MID and LF decay move to
// their STOP values (as the 224XL's gated stop).
// Updates: parameters with a table smoothMs smooth at the control rate (every kReverbControlInterval samples, where the
// coefficients are recomputed; the chorus depth and attack weight ramp across the tick); MID and LF DECAY smooth as
// decay rates (1 / RT), each chasing its STOP value while stopped. SIZE (kSizeGlideS: at most 5 % read speed)
// and PREDELAY glide per sample, so a SIZE change bends the tail's pitch slightly and never clears it. plate_space
// (knob 3) is not a DSP parameter: the store turns it into MID and LF DECAY, SIZE and DEPTH (params.h Links).
// Everything counts samples, so the output does not depend on how the caller splits its blocks. No libDaisy, no
// allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "../memory.h"
#include "../params.h"
#include "reverb.h"

namespace btdsp {

enum PlateLine {
    kPlateDiffL0, kPlateDiffL1, kPlateDiffL2, kPlateDiffR0, kPlateDiffR1, kPlateDiffR2,
    kPlateAp1A, kPlateD1A, kPlateAp2A, kPlateD2A, kPlateAp3A, kPlateD3A,
    kPlateAp1B, kPlateD1B, kPlateAp2B, kPlateD2B, kPlateLines
};
// 48 kHz samples at SIZE 70 %.
constexpr int kPlateLen[kPlateLines] = {45,   143,  362,  52,   177, 302,  728,  4848,
                                        1522, 4053, 2551, 2224, 800, 5577, 2893, 2600};
struct PlateTap {
    int line, delay;
    float gain;
};
constexpr int kPlateTaps = 15;
constexpr float kPlateTapHi = 0.453125f, kPlateTapLo = 0.33984375f;  // 29/64 and 87/256
constexpr PlateTap kPlateTapTable[2][kPlateTaps] = {
    {{kPlateD1A, 308, kPlateTapHi},   {kPlateD1A, 2092, -kPlateTapHi}, {kPlateD1A, 3883, kPlateTapLo},
     {kPlateAp2A, 767, -kPlateTapHi}, {kPlateD2A, 1154, kPlateTapLo},  {kPlateD2A, 3181, kPlateTapLo},
     {kPlateD1B, 1082, -kPlateTapHi}, {kPlateD1B, 2690, kPlateTapHi},  {kPlateD1B, 4500, kPlateTapLo},
     {kPlateAp2B, 1076, kPlateTapHi}, {kPlateAp2B, 2768, -kPlateTapLo}, {kPlateD2B, 1692, kPlateTapLo},
     {kPlateAp3A, 774, kPlateTapHi},  {kPlateD3A, 243, -kPlateTapLo},  {kPlateD3A, 2132, kPlateTapLo}},
    {{kPlateD1A, 1266, -kPlateTapHi}, {kPlateD1A, 2841, kPlateTapHi},  {kPlateD1A, 4730, kPlateTapLo},
     {kPlateD2A, 197, -kPlateTapHi},  {kPlateD2A, 2197, kPlateTapLo},  {kPlateD2A, 3988, -kPlateTapLo},
     {kPlateAp1B, 657, kPlateTapHi},  {kPlateD1B, 2014, kPlateTapHi},  {kPlateD1B, 3831, -kPlateTapLo},
     {kPlateD1B, 5517, kPlateTapLo},  {kPlateAp2B, 1811, -kPlateTapHi}, {kPlateD2B, 688, kPlateTapLo},
     {kPlateD2B, 2577, kPlateTapLo},  {kPlateAp3A, 1934, kPlateTapHi}, {kPlateD3A, 1254, kPlateTapLo}}};
constexpr float kPlateInGain = 0.625f;  // the predelay tap's gain
constexpr float kPlateAttack = 0.68f;   // diffused input in the output at DEPTH 50 %
constexpr float kPlateToneHz = 12000.0f;
constexpr float kPlateSizeMin = 0.5f;    // the SIZE scale at 0 %
constexpr float kPlateSizeMax = 1.215f;  // the SIZE scale's maximum (0.5 + 100 / 140), rounded up
constexpr int kPlateChorusMax = 160;     // samples, peak-to-peak sweep of D1A/D1B at CHORUS 100 %
constexpr float kPlateChorusHz[2] = {0.53f, 0.71f};
constexpr float kPlateGateDb = -50.0f;  // input gate threshold (DECAY OPT, STOP DECAY)
constexpr float kPlateGateReleaseS = 0.05f;
constexpr float kPlateOptFloor = 1.0f / 13.0f;  // DECAY OPT: loop allpass gain factor in a free tail
constexpr float kPlateOptFallS = 0.5f;
constexpr float kPlateOptRiseS = 0.02f;
constexpr float kPlateOutGain = 1.67f;  // wet RMS = input RMS on noise at the defaults

// Segments A1, A2, A3, B1, B2: the allpass and the line each loop gain covers.
constexpr int kPlateSegAp[5] = {kPlateAp1A, kPlateAp2A, kPlateAp3A, kPlateAp1B, kPlateAp2B};
constexpr int PlateCap(int k) {
    return k < kPlateAp1A ? kPlateLen[k] + 3
                          : int(float(kPlateLen[k]) * kPlateSizeMax) + 3 +
                                (k == kPlateD1A || k == kPlateD1B ? kPlateChorusMax : 0);
}
// Every tank and output-tap read lies in Line::Read's [1, cap - 2] for s in [kPlateSizeMin, kPlateSizeMax] (D1A and
// D1B with the full chorus sweep, plus 0.01 for the LFO's rounding), so Process reads them unclamped.
constexpr bool PlateReadsInRange() {
    for (int k = kPlateAp1A; k < kPlateLines; ++k) {
        const float sweep = k == kPlateD1A || k == kPlateD1B ? float(kPlateChorusMax) + 0.01f : 0.0f;
        if (float(kPlateLen[k]) * kPlateSizeMin < 1.0f ||
            float(kPlateLen[k]) * kPlateSizeMax + sweep > float(PlateCap(k) - 2))
            return false;
    }
    for (const auto& side : kPlateTapTable)
        for (const PlateTap& t : side)
            if (float(t.delay) * kPlateSizeMin < 1.0f || float(t.delay) * kPlateSizeMax > float(PlateCap(t.line) - 2))
                return false;
    return true;
}
static_assert(PlateReadsInRange(), "a Plate read can leave its line");
// Line capacities in 48 kHz floats (the layout's design rate).
constexpr size_t kPlateFastFloats = [] {
    size_t n = 0;
    for (int k = 0; k < kPlateLines; ++k) n += size_t(PlateCap(k));
    return n;
}();
constexpr int kPlateInCap = int(btparam::kParams[btparam::kPlatePredelay].max * 48.0f) + 4;  // max predelay
constexpr size_t kPlateSlowFloats = size_t(kPlateInCap);

class Plate final : public Reverb {
public:
    void Init(float sampleRate, const ReverbPool& pool) override {
        fs_ = sampleRate;
        float* f = pool.fast;
        for (int k = 0; k < kPlateLines; ++k) Carve(f, unit_[k].line, PlateCap(k));
        float* s = pool.slow;
        Carve(s, in_, kPlateInCap);
        gc_.Init(fs_);
        aTone_ = OnePoleCoef(kPlateToneHz, fs_);
        rel_ = std::exp(-1.0f / (kPlateGateReleaseS * fs_));
        thr_ = DbToGain(kPlateGateDb);
        for (int j = 0; j < 2; ++j) lfo_[j].SetRate(2.0f * kPi * kPlateChorusHz[j] / fs_);
        Params::ForEachDefault([this](uint8_t id, float v) { SetParam(id, v); });
        Reset();
        Snap();
    }

    void Reset() override {
        bw_ = OnePole{};
        env_ = 0.0f;
        for (OnePole& p : tr_) p = OnePole{};
        for (OnePole& p : tone_) p = OnePole{};
        for (OnePole& p : lp_) p = OnePole{};
        quiet_ = 0;
        opt_ = 1.0f;
        lfo_[0].c = 1.0f;  // phase 0
        lfo_[0].s = 0.0f;
        lfo_[1].c = 0.0f;  // phase pi / 2
        lfo_[1].s = 1.0f;
        ctl_ = 1;
    }

    void SetParam(uint8_t id, float v) override {
        using namespace btparam;
        if (!Params::Owns(id)) return;
        p_.Set(id, v);
        if (id == kPlateSize) sizeT_ = kPlateSizeMin + v / 140.0f;
        else if (id == kPlatePredelay) preT_ = std::clamp(v * fs_ / 1000.0f, 0.0f, float(kPlateInCap - 2));
    }

    void Snap() override {
        p_.Snap(fs_, [](int i) { return Ticked(i); }, [this](int i) { return Target(i); });
        size_.Snap(sizeT_);
        pre_.Snap(preT_);
        opt_ = OptFalling() ? kPlateOptFloor : 1.0f;
        Update();
        amp_.Snap();
        att_.Snap();
    }

    BT_ITCM void Process(const float* in, float* outL, float* outR, size_t n) override {
        const float sizeRate = kSizeGlideMaxRate / float(kPlateLen[kPlateD1B]);
        for (size_t i = 0; i < n; ++i) {
            if (--ctl_ == 0) {
                ctl_ = kReverbControlInterval;
                Tick();
            }
            // 1. Glides.
            const float s =
                std::clamp(size_.Process(sizeT_, gc_.aSize, gc_.b, sizeRate, 1e-7f), kPlateSizeMin, kPlateSizeMax);
            const float pd = std::max(1.0f, pre_.Process(preT_, gc_.a, gc_.b, kGlideMaxRate, 1e-3f));

            // 2. Input: HF BANDWIDTH, predelay, the gate envelope.
            const float x = in[i];
            const float bw = bw_.Process(x, aBw_);
            const float xp = kPlateInGain * Tap(in_, pd, 1.0f);
            in_.Write(bw);
            env_ = Flush(std::max(std::fabs(x), env_ * rel_));
            if (env_ < thr_) quiet_ = std::min(quiet_ + 1, kQuietMax);
            else quiet_ = 0;

            // 3. Input diffusers.
            float xL = xp, xR = xp;
            for (int j = 0; j < 3; ++j) {
                const int kl = kPlateDiffL0 + j, kr = kPlateDiffR0 + j;
                xL = unit_[kl].Feed(xL, Ln(kl).ReadAt(kPlateLen[kl]), gD_);
                xR = unit_[kr].Feed(xR, Ln(kr).ReadAt(kPlateLen[kr]), gD_);
            }

            // 4. Tank: every line read before its write.
            for (Oscillator& o : lfo_) o.Step();
            const float amp = amp_.Step();
            const float r1a = Ln(kPlateD1A).Read(Len(kPlateD1A, s) + amp * (1.0f - lfo_[0].c));
            const float r1b = Ln(kPlateD1B).Read(Len(kPlateD1B, s) + amp * (1.0f - lfo_[1].c));
            const float r2a = Ln(kPlateD2A).Read(Len(kPlateD2A, s));
            const float r2b = Ln(kPlateD2B).Read(Len(kPlateD2B, s));
            const float r3a = Ln(kPlateD3A).Read(Len(kPlateD3A, s));
            const float g = c_ * opt_;
            Ln(kPlateD1A).Write(Flush(unit_[kPlateAp1A].Process(xL - Shelf(4, r2b), Len(kPlateAp1A, s), g)));
            Ln(kPlateD1B).Write(Flush(unit_[kPlateAp1B].Process(xR + Shelf(2, r3a), Len(kPlateAp1B, s), g)));
            const float t1a = tr_[0].Process(r1a, aTr_), t1b = tr_[1].Process(r1b, aTr_);
            Ln(kPlateD2A).Write(Flush(unit_[kPlateAp2A].Process(Shelf(0, t1a), Len(kPlateAp2A, s), g)));
            Ln(kPlateD2B).Write(Flush(unit_[kPlateAp2B].Process(Shelf(3, t1b), Len(kPlateAp2B, s), g)));
            Ln(kPlateD3A).Write(Flush(unit_[kPlateAp3A].Process(Shelf(1, r2a), Len(kPlateAp3A, s), g)));

            // 5. Output: the taps, the attack, the tone one-pole.
            const float y0 = Taps<0>(s, std::make_index_sequence<kPlateTaps>{});
            const float y1 = Taps<1>(s, std::make_index_sequence<kPlateTaps>{});
            const float att = att_.Step();
            outL[i] = kPlateOutGain * tone_[0].Process(y0 + att * xL, aTone_);
            outR[i] = kPlateOutGain * tone_[1].Process(y1 + att * xR, aTone_);
        }
    }

private:
    using Params = AlgoParams<btparam::kPlateMidDecay, btparam::kPlateStopDelay>;
    static constexpr uint8_t kFirst = Params::kFirst;
    static constexpr int kQuietMax = 1 << 30;  // quiet_'s cap: no int overflow in long silence

    BT_ITCM Line& Ln(int k) { return unit_[k].line; }
    static BT_ITCM float Len(int k, float s) { return float(kPlateLen[k]) * s; }

    // Side's output taps at SIZE scale s, summed in table order (as the table loop did); compile-time indices, so the
    // line, delay and gain of each tap are constants.
    template <int Side, size_t... I>
    [[gnu::always_inline]] inline float Taps(float s, std::index_sequence<I...>) {
        float acc = 0.0f;
        ((acc += kPlateTapTable[Side][I].gain *
                 Ln(kPlateTapTable[Side][I].line).Read(float(kPlateTapTable[Side][I].delay) * s)),
         ...);
        return acc;
    }

    // Segment k's crossover shelf.
    BT_ITCM float Shelf(int k, float y) {
        return g_[k] * y + (gl_[k] - g_[k]) * lp_[k].Process(y, aX_);
    }

    BT_ITCM bool GateClosed() const { return env_ < thr_; }
    BT_ITCM bool OptFalling() const { return p_.T(btparam::kPlateDecayOpt) > 0.5f && GateClosed(); }
    BT_ITCM bool Stopped() const {
        return p_.T(btparam::kPlateStopDecay) > 0.5f &&
               float(quiet_) >= p_.T(btparam::kPlateStopDelay) * fs_ / 1000.0f;
    }

    // Whether value i smooths at the control rate: a table smoothMs, except the STOP decays, which MID and LF DECAY's
    // smoothers chase instead.
    static BT_ITCM bool Ticked(int i) {
        const int id = kFirst + i;
        return btparam::kParams[id].smoothMs > 0.0f && id != btparam::kPlateLfStopDecay &&
               id != btparam::kPlateMidStopDecay;
    }
    // What smoother i chases: MID and LF DECAY as decay rates (1 / RT, their STOP values while stopped), the rest in
    // their units.
    BT_ITCM float Target(int i) const {
        using namespace btparam;
        const int id = kFirst + i;
        if (id == kPlateMidDecay) return 1.0f / (Stopped() ? p_.T(kPlateMidStopDecay) : p_.T(kPlateMidDecay));
        if (id == kPlateLfDecay) return 1.0f / (Stopped() ? p_.T(kPlateLfStopDecay) : p_.T(kPlateLfDecay));
        return p_.At(i);
    }

    BT_ITCM void Tick() {
        p_.Tick([](int i) { return Ticked(i); }, [this](int i) { return Target(i); });
        Update();
        constexpr float kTick = float(kReverbControlInterval);
        if (OptFalling()) opt_ = std::max(kPlateOptFloor, opt_ - kTick / (kPlateOptFallS * fs_));
        else opt_ = std::min(1.0f, opt_ + kTick / (kPlateOptRiseS * fs_));
        for (Oscillator& o : lfo_) o.Renormalize();  // keep the rotations on the unit circle
    }

    // The coefficients from the smoothed values and the current SIZE.
    BT_ITCM void Update() {
        using namespace btparam;
        const float s = size_.Value();
        const float rateMid = p_.Smoothed(kPlateMidDecay), rateLf = p_.Smoothed(kPlateLfDecay);  // 1 / RT
        for (int k = 0; k < 5; ++k) {
            const float len = float(kPlateLen[kPlateSegAp[k]] + kPlateLen[kPlateSegAp[k] + 1]) * s / fs_;
            g_[k] = std::exp(-kLn1000 * len * rateMid);
            gl_[k] = std::exp(-kLn1000 * len * rateLf);
        }
        aX_ = OnePoleCoef(p_.Smoothed(kPlateCrossover), fs_);
        aTr_ = OnePoleCoef(p_.Smoothed(kPlateTrebleDecay), fs_);
        aBw_ = OnePoleCoef(p_.Smoothed(kPlateHfBandwidth), fs_);
        const float d = p_.Smoothed(kPlateDiffusion);
        gD_ = d <= 80.0f ? -0.8125f * d / 80.0f : -(0.8125f + 0.0875f * (d - 80.0f) / 20.0f);
        c_ = 0.6f - 0.4f * p_.Smoothed(kPlateDefinition) / 100.0f;
        amp_.Set(float(kPlateChorusMax) * p_.Smoothed(kPlateChorus) / 100.0f * 0.5f);
        att_.Set(kPlateAttack * 2.0f * (1.0f - p_.Smoothed(kPlateDepth) / 100.0f));
    }

    float fs_ = 0.0f;  // Init
    // Lines: the diffusers and the tank's allpasses are Allpass units; a tank delay's unit is only its line.
    Allpass unit_[kPlateLines];
    Line in_;
    // Parameters: table values and smoothers, glides and their targets.
    Params p_;  // smoothers only for the Ticked() ids; MID and LF DECAY as 1 / RT
    float sizeT_ = 1.0f, preT_ = 0.0f;  // glides: scale, samples
    Glide size_, pre_;
    GlideCoefs gc_;
    float rel_ = 0.0f, thr_ = 0.0f;
    int ctl_ = 1;
    // Coefficients (Update).
    float g_[5] = {}, gl_[5] = {}, aX_ = 0.0f, aTr_ = 1.0f, aBw_ = 1.0f, aTone_ = 1.0f, gD_ = 0.0f, c_ = 0.0f;
    ReverbTickRamp amp_, att_;  // chorus depth, attack weight: ramped per sample
    // State.
    OnePole bw_, tr_[2], lp_[5], tone_[2];
    float env_ = 0.0f, opt_ = 1.0f;
    Oscillator lfo_[2];  // the chorus sines, D1A and D1B (Reset sets their phases)
    int quiet_ = 0;
};

}  // namespace btdsp
