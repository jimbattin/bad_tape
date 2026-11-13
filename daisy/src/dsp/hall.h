// The Hall: a native reverb aiming for the character of the Lexicon 480L Random Hall. Lengths are 48 kHz samples at
// SIZE 30 m, scaled linearly by SIZE in metres. The tank lengths and the WANDER depth are 48 kHz sample counts at any
// rate, so every tank and output-tap read stays inside its line (HallReadsInRange); PREDELAY, the echo delays and
// SPREAD convert with fs, and their reads are clamped.
//
// Input: a 20 Hz one-pole highpass, then HF CUTOFF (6 dB/oct lowpass) into a predelay line. The tank is fed from
// kHallSpreadTaps taps spread from the predelay across the SPREAD window (scaled by SIZE), weighted by SHAPE (0: front-
// loaded, 50: flat, 100: slow build; the weights sum to 1 before ageing, below), through four fixed input diffusers
// (allpasses, DIFFUSION). Pre-echoes 1 and 3 (L) and 2 and 4 (R) read the same line and reach the output only, at
// unity under HF CUTOFF and OUTPUT SHELF.
// Tank: a figure-8 of two halves, each allpass A (-c) -> line -> RT HF CUT one-pole -> crossover one-pole splitting
// the gain into gMid above and gBass below -> allpass B (+c), its output feeding the other half's input.
// RT law: the per-half targets are 10^(-3 L / (RT * fs)) over the half's length L, and RT * BASS MULT for the bass.
// RT MID holds at kHallRtRefHz: the crossover's bleed of the bass gain, RT HF CUT and the random taps' fade losses
// (kHallFadeLossDb for the line, kHallApFadeLossDb per allpass, each scaled by its WANDER depth) are divided out there;
// a compensated gain never exceeds the larger of its target and kHallGainCap, so the loop gain stays below 1. c is
// held to kHallApRing x the RT decay over allpass B, so the allpasses' own ringing never outlasts a short RT.
// Random taps: every loop element (allpass A, the line and allpass B, in each half) is read at nominal + an offset; a
// new random offset is cross-faded in with smoothstep gains that sum to 1 (never above the RT gain; up to 3 dB dip
// mid-fade between decorrelated taps): the line's in +-E (WANDER, scaled by SIZE) every P/2 samples (+-15 %) over P/4,
// an allpass's in +-E x its length over the line's every P (+-15 %, as often as the 480L moves a tap at SPIN 50) over
// P/16, so the allpasses fade a sixteenth of the time and lose little (kHallLineMove, kHallApMove). P falls
// exponentially with SPIN. No loop element keeps a fixed delay, so no fixed-delay path survives a pass and the allpass
// delays leave no periodicity in the tail (the 480L moves its tank taps the same way). A WANDER change applies at the
// next relocation, a SPIN change at the next interval.
// Output: each side reads only its own half (L half 0, R half 1, as the 480L's two DSPs): one tap each in the line,
// allpass B and allpass A, at fixed fractions of their current lengths (kHallTapFrac). So the mono fold (kReverbFold)
// never sums two reads of one line at a fixed spacing, a static comb that no random tap moves. Each tap is weighted
// (kHallTapWeight) and aged by the RT decay over its content's age in its half (as are the SPREAD taps), so a pass
// decays smoothly between taps even at short RT; plus the pre-echoes, through a first-order high shelf at kHallShelfHz
// (OUTPUT SHELF), with the tank (not the pre-echoes) at kHallOutGain.
// Updates: parameters with a table smoothMs smooth at the control rate (every kReverbControlInterval samples, where the
// coefficients are recomputed; the output tap weights, the loop gains and the echo levels ramp across the tick, so a
// gain change never steps the signal); echo levels smooth as linear gains and RT MID as a decay rate (1 / RT).
// SIZE, PREDELAY, the echo delays and the SPREAD window (despite its table smoothMs: it moves read positions) glide per
// sample (the tape glide's law; read speed 0.75-1.25, SIZE 0.95-1.05 and slower), so a SIZE change bends the tail's
// pitch slightly and never clears it. hall_space (knob 3) is not a DSP parameter: the store turns it into RT MID, SIZE,
// SHAPE and SPREAD (params.h Links). Everything counts samples, so the output does not depend on how the caller splits
// its blocks. No libDaisy, no allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../memory.h"
#include "../params.h"
#include "reverb.h"

namespace btdsp {

constexpr float kHallSizeRef = 30.0f;  // m, the SIZE the base lengths are at
constexpr float kHallSizeMin = btparam::kParams[btparam::kHallSize].min;  // m
constexpr float kHallSizeMax = btparam::kParams[btparam::kHallSize].max;  // m
constexpr float kHallScaleMin = kHallSizeMin / kHallSizeRef, kHallScaleMax = kHallSizeMax / kHallSizeRef;
constexpr int kHallApA[2] = {1218, 1342};          // loop allpass A, half 0 / half 1
constexpr int kHallLine[2] = {6804, 6553};         // loop line, carries the random taps
constexpr int kHallApB[2] = {2152, 1987};          // loop allpass B
constexpr int kHallDiff[4] = {229, 173, 611, 447};  // input diffusers, fixed (not SIZE-scaled)
constexpr float kHallWanderMaxMs = 25.0f;  // WANDER 100 % at 30 m (40 % default = 10 ms, as on the 480L)
constexpr float kHallSpinSlowS = 2.0f;     // tap period P at SPIN 0
constexpr float kHallSpinFastS = 0.05f;    // tap period P at SPIN 100 % (50 % -> 0.32 s)
constexpr float kHallSpreadMaxMs = 200.0f;  // SPREAD window at 100 % and SIZE 39
constexpr int kHallSpreadTaps = 4;          // injection taps across the SPREAD window
constexpr float kHallShelfHz = 2500.0f;     // OUTPUT SHELF corner
constexpr float kHallInHpHz = 20.0f;        // input DC/subsonic highpass
constexpr float kHallOutGain = 6.24f;       // reverb (tank) level, wet RMS = input RMS on noise at the defaults; the
                                            // pre-echoes stay at unity
// Smoothstep fades half the time lose -10 log10(1 - 2 * mean(s (1 - s)) / 2) = 0.6 dB per pass on decorrelated
// content; the RT gains win it back (the 480L also programs RT above the display), never above kHallGainCap.
constexpr float kHallFadeLossDb = 0.6f;
// An allpass fades a sixteenth of the time: -10 log10(1 - 2 * mean(s (1 - s)) / 16) = 0.07 dB per pass (to first
// order the same through its recirculation: the share of energy that reaches the read is 1 - c^2 per pass, and it
// decays). Fading more often would lower the longest reachable RT: the compensated gains meet kHallGainCap sooner.
constexpr float kHallApFadeLossDb = 0.07f;
// A random tap's relocation interval and cross-fade, x P.
struct HallMove {
    float every, fade;
};
constexpr HallMove kHallLineMove = {0.5f, 0.25f};
constexpr HallMove kHallApMove = {1.0f, 0.0625f};
constexpr float kHallDecorrMs = 1.0f;     // WANDER depth from which the fades count as fully decorrelated
constexpr float kHallGainCap = 0.999f;    // compensated per-half gains stay at or below this (or the uncompensated)
constexpr float kHallRtRefHz = 1000.0f;   // RT MID holds here
constexpr float kHallApRing = 0.5f;       // loop allpass |c| limit relative to the RT decay over allpass B
constexpr uint32_t kHallSeed = 0x2545F491u;
// Output taps per side, half 0 (L) then half 1 (R): its line, allpass B (subtracted), allpass A (subtracted).
// kHallTapFrac: fractions of each element's length; the content's age in its half is A + f L, (f - 1) B (allpass B's
// content already carries the half's gain) and f A.
constexpr float kHallTapWeight[6] = {0.75f, 0.375f, 0.1875f, 0.75f, 0.375f, 0.1875f};
constexpr float kHallTapFrac[6] = {0.30f, 0.50f, 0.50f, 0.60f, 0.40f, 0.60f};

// Line capacities in 48 kHz floats (the layout's design rate), fixed so the layout has no failure path.
constexpr int HallCap(int base) { return int(float(base) * kHallSizeMax / kHallSizeRef) + 3; }
constexpr int HallDiffCap(int i) { return kHallDiff[i] + 3; }
constexpr int kHallWanderCap = int(kHallWanderMaxMs * kReverbRate / 1000.0f * kHallSizeMax / kHallSizeRef);  // 1560
constexpr int HallLineCap(int k) { return HallCap(kHallLine[k]) + kHallWanderCap; }
// A loop allpass of base length in half k: its random offset reaches the line's x base / kHallLine[k].
constexpr int HallApCap(int base, int k) { return HallCap(base) + kHallWanderCap * base / kHallLine[k] + 1; }
constexpr size_t kHallFastFloats =
    size_t(HallDiffCap(0) + HallDiffCap(1) + HallDiffCap(2) + HallDiffCap(3)) +
    size_t(HallApCap(kHallApA[0], 0) + HallLineCap(0) + HallApCap(kHallApB[0], 0)) +
    size_t(HallApCap(kHallApA[1], 1) + HallLineCap(1) + HallApCap(kHallApB[1], 1));
// The input line: max predelay + max SPREAD window.
constexpr int kHallInCap = int((btparam::kParams[btparam::kHallPredelay].max + kHallSpreadMaxMs) * 48.0f) + 4;
constexpr size_t kHallSlowFloats = size_t(kHallInCap);

// Every loop element's read, at its upper end, and every output tap, at both ends, lies in Line::Read's [1, cap - 2]
// for s in [kHallScaleMin, kHallScaleMax] (Process and Update clamp the SIZE glide there). A random offset is at
// most the WANDER depth at the SIZE it was drawn at, so nominal x kHallScaleMax + the depth at kHallScaleMax (+ 0.01
// for rounding) bounds every tank read. The lower end keeps TapRead's clamp at 2: an offset drawn at a larger SIZE
// can exceed a nominal that has since shrunk.
constexpr bool HallReadsInRange() {
    const float depth = 100.0f / 100.0f * kHallWanderMaxMs * kReverbRate / 1000.0f * kHallScaleMax;
    for (int k = 0; k < 2; ++k) {
        const int len[3] = {kHallApA[k], kHallLine[k], kHallApB[k]};
        const int cap[3] = {HallApCap(kHallApA[k], k), HallLineCap(k), HallApCap(kHallApB[k], k)};
        for (int m = 0; m < 3; ++m) {
            const float d = m == 1 ? depth : depth * float(len[m]) / float(kHallLine[k]);
            if (float(len[m]) * kHallScaleMax + d + 0.01f > float(cap[m] - 2)) return false;
        }
        const int tl[3] = {kHallLine[k], kHallApB[k], kHallApA[k]};  // Side's order: line, allpass B, allpass A
        const int tc[3] = {HallLineCap(k), HallApCap(kHallApB[k], k), HallApCap(kHallApA[k], k)};
        for (int m = 0; m < 3; ++m) {
            const float f = kHallTapFrac[3 * k + m] * float(tl[m]);
            if (f * kHallScaleMin < 1.0f || f * kHallScaleMax > float(tc[m] - 2)) return false;
        }
    }
    return true;
}
static_assert(HallReadsInRange(), "a Hall read can leave its line");

class Hall final : public Reverb {
public:
    void Init(float sampleRate, const ReverbPool& pool) override {
        fs_ = sampleRate;
        float* f = pool.fast;
        for (int i = 0; i < 4; ++i) Carve(f, diff_[i].line, HallDiffCap(i));
        for (int k = 0; k < 2; ++k) {
            Carve(f, apA_[k].line, HallApCap(kHallApA[k], k));
            Carve(f, line_[k], HallLineCap(k));
            Carve(f, apB_[k].line, HallApCap(kHallApB[k], k));
        }
        float* s = pool.slow;
        Carve(s, in_, kHallInCap);
        gc_.Init(fs_);
        a20_ = OnePoleCoef(kHallInHpHz, fs_);
        aS_ = OnePoleCoef(kHallShelfHz, fs_);
        refCos_ = std::cos(2.0f * kPi * kHallRtRefHz / fs_);
        refSin_ = std::sin(2.0f * kPi * kHallRtRefHz / fs_);
        keyed_ = false;
        Params::ForEachDefault([this](uint8_t id, float v) { SetParam(id, v); });
        Reset();
        Snap();
    }

    void Reset() override {
        o_[0] = o_[1] = 0.0f;
        hfk_[0] = hfk_[1] = lo_[0] = lo_[1] = lpS_[0] = lpS_[1] = lp20_ = hf_ = OnePole{};
        rng_ = XorShift32{kHallSeed};
        const float p = Period(p_.T(btparam::kHallSpin));
        for (RandomTap(&half)[3] : tap_) {
            for (int m = 0; m < 3; ++m) {  // the taps start staggered across their intervals
                half[m] = RandomTap{};
                half[m].count = int(rng_.Uniform() * p * (m == kTapLine ? kHallLineMove : kHallApMove).every);
            }
        }
        ctl_ = 1;
    }

    void SetParam(uint8_t id, float v) override {
        using namespace btparam;
        if (!Params::Owns(id)) return;
        p_.Set(id, v);
        const float inMax = float(kHallInCap - 2);
        if (id == kHallSize) sizeT_ = v / kHallSizeRef;
        else if (id == kHallSpread)
            spreadT_ = v / 100.0f * kHallSpreadMaxMs * fs_ / 1000.0f * kHallSizeRef / kHallSizeMax;
        else if (id == kHallPredelay) preT_ = std::clamp(v * fs_ / 1000.0f, 0.0f, inMax);
        else if (id >= kHallEchoDelay1 && id <= kHallEchoDelay4)
            echoDT_[id - kHallEchoDelay1] = std::clamp(v * fs_ / 1000.0f, 0.0f, inMax);
        else if (id >= kHallEchoLevel1 && id <= kHallEchoLevel4)
            echoGT_[id - kHallEchoLevel1] = v <= -59.999f ? 0.0f : DbToGain(v);
    }

    void Snap() override {
        p_.Snap(fs_, [](int i) { return Ticked(i); }, [this](int i) { return Target(i); });
        size_.Snap(sizeT_);
        spread_.Snap(spreadT_);
        pre_.Snap(preT_);
        for (int j = 0; j < 4; ++j) echoD_[j].Snap(echoDT_[j]);
        Update();
        for (ReverbTickRamp& r : tw_) r.Snap();
        for (ReverbTickRamp& r : eg_) r.Snap();
        for (int k = 0; k < 2; ++k) {
            gMid_[k].Snap();
            gBass_[k].Snap();
        }
    }

    BT_ITCM void Process(const float* in, float* outL, float* outR, size_t n) override {
        const float sizeRate = kSizeGlideMaxRate / float(kHallLine[0]);
        for (size_t i = 0; i < n; ++i) {
            if (--ctl_ == 0) {
                ctl_ = kReverbControlInterval;
                p_.Tick([](int i) { return Ticked(i); }, [this](int i) { return Target(i); });
                for (ReverbTickRamp& r : eg_) r.Snap();  // the last ramp ends exactly on its target
                Update();
            }
            // 1. Glides.
            const float s = Scale(size_.Process(sizeT_, gc_.aSize, gc_.b, sizeRate, 1e-7f));
            const float pd = std::max(1.0f, pre_.Process(preT_, gc_.a, gc_.b, kGlideMaxRate, 1e-3f));
            const float wStep = spread_.Process(spreadT_, gc_.a, gc_.b, kGlideMaxRate, 1e-3f) * s *
                                (1.0f / float(kHallSpreadTaps - 1));
            float ed[4];
            for (int j = 0; j < 4; ++j) ed[j] = echoD_[j].Process(echoDT_[j], gc_.a, gc_.b, kGlideMaxRate, 1e-3f);

            // 2. Input: 20 Hz highpass, HF CUTOFF.
            const float xin = in[i];
            const float hf = hf_.Process(xin - lp20_.Process(xin, a20_), aHf_);

            // 3. Injection across the SPREAD window and the pre-echoes, read before the write.
            float u = 0.0f;
            for (int j = 0; j < kHallSpreadTaps; ++j) u += w_[j] * Tap(in_, pd + float(j) * wStep, 1.0f);
            float eL = 0.0f, eR = 0.0f;
            float eg[4];
            for (int j = 0; j < 4; ++j) eg[j] = eg_[j].Step();  // the echo levels ramp across the tick
            if (eg[0] != 0.0f) eL += eg[0] * Tap(in_, ed[0], 1.0f);
            if (eg[2] != 0.0f) eL += eg[2] * Tap(in_, ed[2], 1.0f);
            if (eg[1] != 0.0f) eR += eg[1] * Tap(in_, ed[1], 1.0f);
            if (eg[3] != 0.0f) eR += eg[3] * Tap(in_, ed[3], 1.0f);
            in_.Write(hf);

            // 4. Input diffusers.
            u = diff_[0].Feed(u, diff_[0].line.ReadAt(kHallDiff[0]), gIn1_);
            u = diff_[1].Feed(u, diff_[1].line.ReadAt(kHallDiff[1]), gIn1_);
            u = diff_[2].Feed(u, diff_[2].line.ReadAt(kHallDiff[2]), gIn2_);
            u = diff_[3].Feed(u, diff_[3].line.ReadAt(kHallDiff[3]), gIn2_);

            // 5. Tank: each half takes the other's previous output; every element reads at its random tap.
            const float o0 = Half<0>(u + o_[1], s);
            const float o1 = Half<1>(u + o_[0], s);
            o_[0] = o0;
            o_[1] = o1;

            // 6. Outputs (each side its own half, aged; Update), OUTPUT SHELF.
            float tw[6];
            for (int j = 0; j < 6; ++j) tw[j] = tw_[j].Step();  // the output taps ramp across the tick (they follow RT)
            const float xL = Side(0, tw, s) + eL;
            const float xR = Side(1, tw + 3, s) + eR;
            outL[i] = xL + (shelfG_ - 1.0f) * (xL - lpS_[0].Process(xL, aS_));
            outR[i] = xR + (shelfG_ - 1.0f) * (xR - lpS_[1].Process(xR, aS_));
        }
    }

private:
    using Params = AlgoParams<btparam::kHallRtMid, btparam::kHallOutputShelf>;
    static constexpr uint8_t kFirst = Params::kFirst;

    struct RandomTap {
        float off[2] = {0.0f, 0.0f};
        int active = 0, count = 0;
        Crossfade fade;
    };
    // A half's random taps (tap_[k][]).
    static constexpr int kTapA = 0, kTapLine = 1, kTapB = 2;

    // The SIZE glide's length scale, held to the table's SIZE range (HallReadsInRange's proof depends on it).
    static BT_ITCM float Scale(float glide) { return std::clamp(glide, kHallScaleMin, kHallScaleMax); }

    // Tank half K on x (the diffused input plus the other half's previous output); every element reads at its random
    // tap. Compile-time K: the lengths and the state are constants (always_inline: ../memory.h).
    template <int K> [[gnu::always_inline]] inline float Half(float x, float s) {
        RandomTap* t = tap_[K];
        const float zA = TapRead(t[kTapA], apA_[K].line, float(kHallApA[K]) * s, apDepth_[K][0], kHallApMove);
        const float v = apA_[K].Feed(x, zA, -c_);
        float r = TapRead(t[kTapLine], line_[K], float(kHallLine[K]) * s, depth_, kHallLineMove);
        line_[K].Write(Flush(v));
        r = hfk_[K].Process(r, aRtHf_);
        const float lo = lo_[K].Process(r, aX_);
        // The loop gains ramp across the tick: a step would click on every pass.
        const float gm = gMid_[K].Step(), gb = gBass_[K].Step();
        r = gm * r + (gb - gm) * lo;
        const float zB = TapRead(t[kTapB], apB_[K].line, float(kHallApB[K]) * s, apDepth_[K][1], kHallApMove);
        return Flush(apB_[K].Feed(r, zB, c_));
    }

    // The one-pole lowpass y += a (x - y) at kHallRtRefHz: re + j im.
    BT_ITCM void OnePoleAtRef(float a, float& re, float& im) const {
        const float dr = 1.0f - (1.0f - a) * refCos_, di = (1.0f - a) * refSin_;
        const float m = a / (dr * dr + di * di);
        re = m * dr;
        im = -m * di;
    }
    // The random-tap period P in samples at SPIN spin (%).
    BT_ITCM float Period(float spin) const {
        return kHallSpinSlowS * std::pow(kHallSpinFastS / kHallSpinSlowS, spin / 100.0f) * fs_;
    }

    // Whether value i smooths at the control rate: a table smoothMs, except SPREAD, whose window moves read
    // positions and so glides per sample like SIZE.
    static BT_ITCM bool Ticked(int i) {
        return btparam::kParams[kFirst + i].smoothMs > 0.0f && kFirst + i != btparam::kHallSpread;
    }
    // What smoother i chases: the echo levels as linear gains, RT MID as a decay rate (1 / RT, so a jump from the
    // shortest RT does not lengthen most of the way in one tick), the rest in their units.
    BT_ITCM float Target(int i) const {
        const int e = i - (btparam::kHallEchoLevel1 - kFirst);
        if (e >= 0 && e < 4) return echoGT_[e];
        return i == btparam::kHallRtMid - kFirst ? 1.0f / p_.At(i) : p_.At(i);
    }

    // The coefficients from the smoothed values and the current SIZE. Every coefficient it writes is a function of the
    // key (and fs_), so an equal key leaves them as they are: only the ramps' targets are re-issued, as the full path
    // would.
    BT_ITCM void Update() {
        using namespace btparam;
        const float s = Scale(size_.Value()), spread = spread_.Value();
        const float rtRate = p_.Smoothed(kHallRtMid), bass = p_.Smoothed(kHallBassMult);
        const float spin = p_.Smoothed(kHallSpin), wander = p_.Smoothed(kHallWander);
        const float xover = p_.Smoothed(kHallCrossover), rtHf = p_.Smoothed(kHallRtHfCut);
        const float hfCut = p_.Smoothed(kHallHfCutoff), diffusion = p_.Smoothed(kHallDiffusion);
        const float shelf = p_.Smoothed(kHallOutputShelf), shape = p_.Smoothed(kHallShape);
        const float key[kKeyLen] = {s, spread, rtRate, bass, spin, wander, xover, rtHf, hfCut, diffusion, shelf, shape,
                                    p_.Smoothed(kHallEchoLevel1), p_.Smoothed(kHallEchoLevel2),
                                    p_.Smoothed(kHallEchoLevel3), p_.Smoothed(kHallEchoLevel4),
                                    echoGT_[0], echoGT_[1], echoGT_[2], echoGT_[3]};
        bool same = keyed_;
        for (int i = 0; same && i < kKeyLen; ++i) same = key[i] == key_[i];
        if (same) {
            for (ReverbTickRamp& r : tw_) r.Set(r.t);
            for (ReverbTickRamp& r : eg_) r.Set(r.t);
            for (int k = 0; k < 2; ++k) {
                gMid_[k].Set(gMid_[k].t);
                gBass_[k].Set(gBass_[k].t);
            }
            return;
        }
        for (int i = 0; i < kKeyLen; ++i) key_[i] = key[i];
        keyed_ = true;
        const float rt = 1.0f / rtRate;
        period_ = Period(spin);
        depth_ = wander / 100.0f * kHallWanderMaxMs * kReverbRate / 1000.0f * s;
        constexpr float kLn1000 = 6.9077553f, kDbToLn = 0.11512925f;  // ln(10) / 20
        aX_ = OnePoleCoef(xover, fs_);
        aRtHf_ = OnePoleCoef(rtHf, fs_);
        // RT MID holds at kHallRtRefHz: the loop's gain there (crossover bleed of the bass gain, RT HF CUT, and the
        // random-tap fades' losses on decorrelated content) is divided out, never above kHallGainCap.
        float xr, xi, hr, hi;
        OnePoleAtRef(aX_, xr, xi);
        OnePoleAtRef(aRtHf_, hr, hi);
        const float hf = std::sqrt(hr * hr + hi * hi), decorrS = kHallDecorrMs * fs_ / 1000.0f;
        const float lineDb = kHallFadeLossDb * std::min(1.0f, depth_ / decorrS);
        for (int k = 0; k < 2; ++k) {
            apDepth_[k][0] = depth_ * float(kHallApA[k]) / float(kHallLine[k]);
            apDepth_[k][1] = depth_ * float(kHallApB[k]) / float(kHallLine[k]);
            const float fadeDb = lineDb + kHallApFadeLossDb * (std::min(1.0f, apDepth_[k][0] / decorrS) +
                                                               std::min(1.0f, apDepth_[k][1] / decorrS));
            const float loss = hf * std::exp(-fadeDb * kDbToLn);
            const float len = float(kHallApA[k] + kHallLine[k] + kHallApB[k]) * s;
            const float gm = std::exp(-kLn1000 * len / (rt * fs_));
            const float gb = std::exp(-kLn1000 * len / (rt * bass * fs_));
            const float beta = gb / gm, br = 1.0f + (beta - 1.0f) * xr, bi = (beta - 1.0f) * xi;
            const float g = gm / (std::sqrt(br * br + bi * bi) * loss);
            gMid_[k].Set(std::min(g, std::max(gm, kHallGainCap)));
            gBass_[k].Set(std::min(g * beta, std::max(gb, kHallGainCap)));
        }
        // Output taps: weight x the RT decay over the content's age in its half (allpass B's content already carries
        // the half's gain), so every pass, the first included, decays at RT MID between its taps.
        const float age = -kLn1000 * s / (rt * fs_);
        for (int k = 0; k < 2; ++k) {
            const int j = 3 * k;
            const float fA = float(kHallApA[k]), fL = float(kHallLine[k]), fB = float(kHallApB[k]);
            const float ages[3] = {fA + kHallTapFrac[j] * fL, (kHallTapFrac[j + 1] - 1.0f) * fB, kHallTapFrac[j + 2] * fA};
            for (int m = 0; m < 3; ++m) tw_[j + m].Set(kHallOutGain * kHallTapWeight[j + m] * std::exp(age * ages[m]));
        }
        aHf_ = OnePoleCoef(hfCut, fs_);
        const float d = diffusion / 100.0f;
        gIn1_ = 0.75f * d;
        gIn2_ = 0.625f * d;
        // The loop allpasses' own recirculation (c per d samples) must decay well inside RT MID, or their modes
        // outlast it: c is held to kHallApRing x the RT decay over the longer allpass B.
        const float ringMax = kHallApRing * std::exp(-6.9077553f * float(kHallApB[0]) * s / (rt * fs_));
        c_ = std::min(0.6f * (0.4f + 0.6f * d), ringMax);
        for (int j = 0; j < 4; ++j) {
            const float g = key[kKeyEcho + j];
            eg_[j].Set(key[kKeyEcho + 4 + j] == 0.0f && g < 1e-6f ? 0.0f : g);
        }
        shelfG_ = DbToGain(shelf);
        const float sh = shape / 100.0f;
        // SHAPE weights over the SPREAD taps, aged like the output taps: a late tap's content arrives decayed (tap j by
        // r^j, r the decay over one step).
        const float step = spread * s * (1.0f / float(kHallSpreadTaps - 1));
        const float r = std::exp(age * step);
        float q = 1.0f;
        for (int j = 0; j < kHallSpreadTaps; ++j) {
            const float x = float(j) / float(kHallSpreadTaps - 1);
            w_[j] = ((1.0f - sh) * (1.0f - x) + sh * x) / (float(kHallSpreadTaps) / 2.0f) * q;
            q *= r;
        }
    }

    // Line l read at nominal + t's random offset (in +-depth, moved as mv says); called once per sample, before the
    // line's write. noclone: a local clone (.constprop) beside an out-of-line COMDAT copy in one translation unit is a
    // section type conflict (../memory.h), which the bench image's inlining decisions produce.
    [[gnu::noclone]] BT_ITCM float TapRead(RandomTap& t, const Line& l, float nominal, float depth, HallMove mv) {
        if (t.count <= 0 && !t.fade.on) {
            t.off[1 - t.active] = depth * rng_.Bipolar();
            t.fade.Start(std::max(1, int(period_ * mv.fade)));
            t.count = int(period_ * mv.every * (0.85f + 0.3f * rng_.Uniform()));
        }
        float y = l.Read(std::max(2.0f, nominal + t.off[t.active]));
        if (t.fade.on) {
            y += t.fade.Weight() * (l.Read(std::max(2.0f, nominal + t.off[1 - t.active])) - y);
            if (t.fade.Advance()) t.active ^= 1;
        }
        --t.count;
        return y;
    }

    // Side k's tank output: half k's line, allpass B and allpass A taps, weighted by w[0..2] (in range:
    // HallReadsInRange).
    BT_ITCM float Side(int k, const float* w, float s) const {
        const int j = 3 * k;
        return w[0] * line_[k].Read(kHallTapFrac[j] * float(kHallLine[k]) * s) -
               w[1] * apB_[k].line.Read(kHallTapFrac[j + 1] * float(kHallApB[k]) * s) -
               w[2] * apA_[k].line.Read(kHallTapFrac[j + 2] * float(kHallApA[k]) * s);
    }

    float fs_ = 0.0f;  // Init
    // Lines.
    Allpass diff_[4], apA_[2], apB_[2];
    Line line_[2], in_;
    // Parameters: targets (table units), smoothed values, glides and their targets.
    Params p_;  // smoothers only for the Ticked() ids; echo levels as linear gains
    float sizeT_ = 1.0f, spreadT_ = 0.0f, preT_ = 0.0f, echoDT_[4] = {}, echoGT_[4] = {};  // glides: scale, samples
    Glide size_, spread_, pre_, echoD_[4];  // spread_: the SPREAD window at scale 1
    GlideCoefs gc_;
    float refCos_ = 1.0f, refSin_ = 0.0f;
    int ctl_ = 1;
    // Coefficients (Update).
    float aX_ = 0.0f, aRtHf_ = 0.0f, aHf_ = 1.0f, a20_ = 0.0f, aS_ = 0.0f;
    float gIn1_ = 0.0f, gIn2_ = 0.0f, c_ = 0.0f, shelfG_ = 1.0f;
    float w_[kHallSpreadTaps] = {}, period_ = 1.0f, depth_ = 0.0f;
    float apDepth_[2][2] = {};  // half k's allpass A and B WANDER depths, samples
    // Update's inputs at its last full recomputation: s, SPREAD, the smoothed values (RT MID as 1 / RT, BASS MULT,
    // SPIN, WANDER, CROSSOVER, RT HF CUT, HF CUTOFF, DIFFUSION, OUTPUT SHELF, SHAPE, echo levels 1-4 from kKeyEcho),
    // then the echo level targets (kKeyEcho + 4).
    static constexpr int kKeyLen = 20, kKeyEcho = 12;
    float key_[kKeyLen] = {};
    bool keyed_ = false;  // key_ holds a recomputation's inputs (Init clears it)
    // Ramped per sample (Update sets their targets).
    ReverbTickRamp eg_[4];                // echo levels
    ReverbTickRamp tw_[6];                // output taps: L line, B, A; R line, B, A
    ReverbTickRamp gMid_[2], gBass_[2];  // loop gains
    // State.
    float o_[2] = {};
    OnePole hfk_[2], lo_[2], lp20_, hf_, lpS_[2];
    RandomTap tap_[2][3];  // per half: allpass A, line, allpass B
    XorShift32 rng_{kHallSeed};
};

}  // namespace btdsp
