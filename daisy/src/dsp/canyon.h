// The Canyon (SW3 Up): a native reverb aiming for the character of the Eventide H3000 Breathing Canyon ($6A SWEPT
// REVERB). The network is our own design: a six-line modulated feedback delay network. Lengths are 48 kHz samples; ms
// parameters convert with fs, and every read distance is clamped to its line's capacity.
//
// Input: IN TRIM, then two series allpasses (kCanyonDiff, gain kCanyonDiffGain) into every line, with alternating sign
// and 1 / sqrt(6).
// Lines j = 0..5: read at D_j = B_j + S_j, B_j = DELAY j x MASTER DELAY (the base), S_j = DEPTH j x MASTER DEPTH x
// 0.5 (1 - cos phi_j) (the breathing sweep), with phi_j a sine at kCanyonRateHz x RATE j x MASTER RATE / 50
// (starting at j pi / 3). Each read passes a kCanyonDampHz one-pole, then a 6 x 6 Householder matrix (orthogonal) and
// the feedback gain g = kCanyonFbMax x FEEDBACK (negated by FEEDBACK INVERT) back into the lines: with |g| < 1 and the
// one-poles the loop is stable for any delays.
// Output: L = y0 - y2 + y4, R = y1 - y3 + y5 (the undamped reads) at kCanyonOutGain, mixed with the trimmed input by
// MIX (the dry is split so the fold gives (1 - mix) x the trimmed input exactly), then OUT TRIM.
// Updates: parameters with a table smoothMs smooth at the control rate (every kReverbControlInterval samples; the gain,
// the sweep depths and the mix and trims ramp across the tick, the LFO rates step); FEEDBACK smooths signed, so INVERT
// passes through 0; the trims smooth as linear gains. The bases: with DELAY GLIDING on they glide (the tape glide's
// law, GLIDE RATE setting its speed, at most kSizeGlideMaxRate read-speed change: under a semitone, so canyon_space
// on knob 3 bends the pitch only slightly); off, a change of more than half a sample cross-fades to the new base over
// kCanyonJumpS (two reads; a change during a cross-fade is picked up when it ends). canyon_space (knob 3) is not a DSP
// parameter: the store turns it into FEEDBACK, MASTER DELAY and MASTER DEPTH (params.h Links). Everything counts
// samples, so the output does not depend on how the caller splits its blocks. No libDaisy, no allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../memory.h"
#include "../params.h"
#include "reverb.h"

namespace btdsp {

constexpr int kCanyonLines = 6;
// Line capacities in 48 kHz floats (the layout's design rate): a line holds max DELAY + max DEPTH.
constexpr int kCanyonLineCap =
    int((btparam::kParams[btparam::kCanyonDelay1].max + btparam::kParams[btparam::kCanyonDepth1].max) * 48.0f) + 4;
constexpr int kCanyonDiff[2] = {113, 337};
constexpr float kCanyonDiffGain = 0.5f;
constexpr float kCanyonFbMax = 0.985f;  // g at FEEDBACK 100 %
constexpr float kCanyonRateHz = 0.05f;  // Hz per RATE unit at MASTER RATE 50 %
constexpr float kCanyonDampHz = 12000.0f;
constexpr float kCanyonJumpS = 0.02f;  // DELAY GLIDING off: the base cross-fade
constexpr float kCanyonGlideSlowS = 2.0f;   // the base glide's time at GLIDE RATE 0
constexpr float kCanyonGlideFastS = 0.02f;  // and at 100 %
constexpr float kCanyonOutGain = 1.01f;  // wet RMS = input RMS on noise at the defaults
constexpr int CanyonDiffCap(int i) { return kCanyonDiff[i] + 3; }
constexpr size_t kCanyonFastFloats = size_t(CanyonDiffCap(0) + CanyonDiffCap(1)) + 5 * size_t(kCanyonLineCap);
constexpr size_t kCanyonSlowFloats = size_t(kCanyonLineCap);  // line 5

class Canyon final : public Reverb {
public:
    void Init(float sampleRate, const ReverbPool& pool) override {
        fs_ = sampleRate;
        float* f = pool.fast;
        for (int i = 0; i < 2; ++i) Carve(f, diff_[i].line, CanyonDiffCap(i));
        for (int j = 0; j < kCanyonLines - 1; ++j) Carve(f, line_[j], kCanyonLineCap);
        float* s = pool.slow;
        Carve(s, line_[kCanyonLines - 1], kCanyonLineCap);
        gc_.Init(fs_);
        aDamp_ = OnePoleCoef(kCanyonDampHz, fs_);
        fadeLen_ = std::max(1, int(kCanyonJumpS * fs_));
        Params::ForEachDefault([this](uint8_t id, float v) { SetParam(id, v); });
        Reset();
        Snap();
    }

    void Reset() override {
        for (int j = 0; j < kCanyonLines; ++j) {
            lp_[j] = OnePole{};
            fade_[j] = Crossfade{};
            lfo_[j].c = std::cos(float(j) * kPi / 3.0f);
            lfo_[j].s = std::sin(float(j) * kPi / 3.0f);
        }
        ctl_ = 1;
    }

    void SetParam(uint8_t id, float v) override {
        using namespace btparam;
        if (!Params::Owns(id)) return;
        p_.Set(id, v);
        if (id == kCanyonMasterDelay || (id >= kCanyonDelay1 && id <= kCanyonDelay6)) {
            for (int j = 0; j < kCanyonLines; ++j)
                baseT_[j] = p_.T(uint8_t(kCanyonDelay1 + j)) * p_.T(kCanyonMasterDelay) / 100.0f * fs_ / 1000.0f;
        } else if (id == kCanyonGlideRate) {
            const float tau = kCanyonGlideSlowS * std::pow(kCanyonGlideFastS / kCanyonGlideSlowS, v / 100.0f);
            gA_ = TimeCoef(tau, fs_);
        } else if (id == kCanyonInTrim) {
            inT_ = DbToGain(v);
        } else if (id == kCanyonOutTrim) {
            outT_ = DbToGain(v);
        }
    }

    void Snap() override {
        p_.Snap(fs_, [](int i) { return Ticked(i); }, [this](int i) { return Target(i); });
        gliding_ = Gliding();
        for (int j = 0; j < kCanyonLines; ++j) {
            base_[j] = baseT_[j];
            glide_[j].Snap(baseT_[j]);
            fade_[j] = Crossfade{};
        }
        Update();
        g_.Snap();
        mix_.Snap();
        in_.Snap();
        out_.Snap();
        for (ReverbTickRamp& d : depth_) d.Snap();
    }

    BT_ITCM void Process(const float* in, float* outL, float* outR, size_t n) override {
        constexpr float kInvSqrt6 = 0.40824829f, kHouse = 2.0f / float(kCanyonLines);
        const bool gl = Gliding();
        if (gl && !gliding_)  // the glides take over from wherever the bases are
            for (int j = 0; j < kCanyonLines; ++j) glide_[j].Snap(base_[j]);
        gliding_ = gl;
        for (size_t i = 0; i < n; ++i) {
            if (--ctl_ == 0) {
                ctl_ = kReverbControlInterval;
                Tick();
            }
            const float xt = in_.Step() * in[i];
            float u = diff_[0].Process(xt, float(kCanyonDiff[0]), kCanyonDiffGain);
            u = diff_[1].Process(u, float(kCanyonDiff[1]), kCanyonDiffGain) * kInvSqrt6;

            // Every line read before its write.
            float y[kCanyonLines], sum = 0.0f;
            for (int j = 0; j < kCanyonLines; ++j) {
                lfo_[j].Step();
                y[j] = Read(j, depth_[j].Step() * (1.0f - lfo_[j].c));
                sum += lp_[j].Process(y[j], aDamp_);
            }
            const float g = g_.Step(), h = kHouse * sum;
            for (int j = 0; j < kCanyonLines; ++j)
                line_[j].Write(Flush((j & 1 ? -u : u) + g * (lp_[j].y - h)));

            const float mix = mix_.Step(), ot = out_.Step();
            const float dry = (1.0f - mix) * xt * (0.5f / kReverbFold), wet = mix * kCanyonOutGain;
            outL[i] = ot * (wet * (y[0] - y[2] + y[4]) + dry);
            outR[i] = ot * (wet * (y[1] - y[3] + y[5]) + dry);
        }
    }

private:
    using Params = AlgoParams<btparam::kCanyonFeedback, btparam::kCanyonDepth6>;
    static constexpr uint8_t kFirst = Params::kFirst;

    BT_ITCM bool Gliding() const { return p_.T(btparam::kCanyonDelayGliding) > 0.5f; }

    // Line j at its base + sweep; called once per sample, before the line's write.
    BT_ITCM float Read(int j, float sweep) {
        Crossfade& f = fade_[j];
        if (!f.on) {
            if (gliding_) {
                base_[j] = glide_[j].Process(baseT_[j], gA_, gc_.b, kSizeGlideMaxRate, 1e-3f);
            } else if (std::fabs(baseT_[j] - base_[j]) > 0.5f) {
                next_[j] = baseT_[j];
                f.Start(fadeLen_);
            }
        }
        float y = Tap(line_[j], base_[j] + sweep, 1.0f);
        if (f.on) {
            y += f.Weight() * (Tap(line_[j], next_[j] + sweep, 1.0f) - y);
            if (f.Advance()) {
                base_[j] = next_[j];
                glide_[j].Snap(next_[j]);  // gliding turned on mid-fade continues from the new base
            }
        }
        return y;
    }

    // Whether value i smooths at the control rate (a table smoothMs).
    static BT_ITCM bool Ticked(int i) { return btparam::kParams[kFirst + i].smoothMs > 0.0f; }
    // What smoother i chases: FEEDBACK signed by INVERT, the trims as linear gains, the rest in their units.
    BT_ITCM float Target(int i) const {
        using namespace btparam;
        const int id = kFirst + i;
        if (id == kCanyonFeedback) return p_.T(kCanyonFeedbackInvert) > 0.5f ? -p_.At(i) : p_.At(i);
        if (id == kCanyonInTrim) return inT_;
        if (id == kCanyonOutTrim) return outT_;
        return p_.At(i);
    }

    BT_ITCM void Tick() {
        p_.Tick([](int i) { return Ticked(i); }, [this](int i) { return Target(i); });
        Update();
        for (Oscillator& o : lfo_) o.Renormalize();  // keep the rotations on the unit circle
    }

    // The ramp targets and LFO steps from the smoothed values.
    BT_ITCM void Update() {
        using namespace btparam;
        g_.Set(kCanyonFbMax * p_.Smoothed(kCanyonFeedback) / 100.0f);
        mix_.Set(p_.Smoothed(kCanyonMix) / 100.0f);
        in_.Set(p_.Smoothed(kCanyonInTrim));
        out_.Set(p_.Smoothed(kCanyonOutTrim));
        const float rate = kCanyonRateHz * p_.Smoothed(kCanyonMasterRate) / 50.0f;
        const float depth = p_.Smoothed(kCanyonMasterDepth) / 100.0f * fs_ / 1000.0f * 0.5f;
        for (int j = 0; j < kCanyonLines; ++j) {
            lfo_[j].SetRate(2.0f * kPi * rate * p_.Smoothed(uint8_t(kCanyonRate1 + j)) / fs_);
            depth_[j].Set(p_.Smoothed(uint8_t(kCanyonDepth1 + j)) * depth);
        }
    }

    float fs_ = 0.0f;  // Init
    // Lines.
    Allpass diff_[2];
    Line line_[kCanyonLines];
    // Parameters: table values and smoothers, the bases and their glides.
    Params p_;  // smoothers only for the Ticked() ids; FEEDBACK signed, the trims linear
    float inT_ = 1.0f, outT_ = 1.0f, baseT_[kCanyonLines] = {}, base_[kCanyonLines] = {};
    Glide glide_[kCanyonLines];
    float next_[kCanyonLines] = {};  // DELAY GLIDING off: the base each cross-fade moves to
    Crossfade fade_[kCanyonLines];
    bool gliding_ = true;
    GlideCoefs gc_;
    float gA_ = 0.0f, aDamp_ = 1.0f;
    int fadeLen_ = 1, ctl_ = 1;
    // Ramped per sample (Update sets their targets).
    ReverbTickRamp g_, mix_, in_, out_, depth_[kCanyonLines];
    // State.
    OnePole lp_[kCanyonLines];
    Oscillator lfo_[kCanyonLines];  // the breathing sweeps (Reset sets their phases, Update their rates)
};

}  // namespace btdsp
