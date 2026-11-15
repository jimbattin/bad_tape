// The EQ (docs/EQ.md): a five-band parametric, five second-order sections in series: low shelf, bells 1-3, high
// shelf. Mono, no latency, no libDaisy, no allocation.
// Laws: each section is the RBJ cookbook bell or shelf (eq::Design), its analogue prototype under the bilinear
// transform prewarped at the section's frequency, realised as A. Simper's linear trapezoidal state-variable filter
// mixed to that response (m0 x + m1 band + m2 low). The shelves' slope is fixed (eq::kShelfQ). Coefficients are designed
// in double (the Seed's M7 has a double-precision FPU, fpv5-d16); every section filters in float (at 96 kHz within
// docs/EQ.md section 4's tolerance, tests/eq_test.cpp); the input, the output and the enable crossfade are float.
// Changes: gains glide in dB, frequencies in log2 Hz and Qs in log2 Q, over their smoothMs (one-pole, stepped every
// kControlInterval samples); a section whose glides moved redesigns its coefficients on that tick and ramps them
// linearly across it (SectionF::RampTo), so the output has no steps. A section starting from 0 dB starts at its
// identity and ramps from there.
// eq_enable and eq_position crossfade over kFadeSamples. A section whose gain rests at 0 dB for kIdleTicks stops and
// its state is zeroed, so the flat EQ's output is bit-identical to its input; eq_enable off returns the input
// untouched. A placement change (eq_position) fades the stage out; its owner (engine.h) then calls Move() and it fades
// back in at the new place.
// Every update counts samples, so the output does not depend on how the caller splits its blocks.
#pragma once

#include <cmath>
#include <cstdint>

#include "../memory.h"
#include "../params.h"
#include "blocks.h"

namespace btdsp {

// The sections and their design (docs/EQ.md section 2), shared with the tests' model (tests/eq_model.h).
namespace eq {

enum Kind : uint8_t { kLowShelf, kBell, kHighShelf };
constexpr int kBands = 5;  // processing order: low shelf, bells 1-3, high shelf
inline constexpr Kind kKind[kBands] = {kLowShelf, kBell, kBell, kBell, kHighShelf};
inline constexpr uint8_t kGainId[kBands] = {btparam::kEqLowGain, btparam::kEqMidGain1, btparam::kEqMidGain2,
                                            btparam::kEqMidGain3, btparam::kEqHighGain};
inline constexpr uint8_t kFreqId[kBands] = {btparam::kEqLowFreq, btparam::kEqMidFreq1, btparam::kEqMidFreq2,
                                            btparam::kEqMidFreq3, btparam::kEqHighFreq};
inline constexpr uint8_t kQId[kBands] = {btparam::kNoParam, btparam::kEqMidQ1, btparam::kEqMidQ2, btparam::kEqMidQ3,
                                         btparam::kNoParam};
constexpr double kShelfQ = 0.70710678118654752;  // the shelves' fixed slope (no overshoot)

// The SVF's coefficients (a1..a3) and output mix (m0 input, m1 band, m2 low).
struct SvfCoefs {
    double a1, a2, a3, m0, m1, m2;
};
// A. Simper's linear trapezoidal SVF mixed to the RBJ cookbook bell or shelf of gain db at f0 (bells: the centre;
// shelves: the half-gain frequency), quality q: exactly their bilinear transform prewarped at f0.
BT_ITCM inline SvfCoefs Design(Kind kind, double f0, double q, double db, double fs) {
    const double A = std::pow(10.0, db / 40.0);
    double g = std::tan(kPiT<double> * f0 / fs), k = 1.0 / q, m0 = 1.0, m1 = 0.0, m2 = 0.0;
    switch (kind) {
        case kBell:
            k = 1.0 / (q * A);
            m1 = k * (A * A - 1.0);
            break;
        case kLowShelf:
            g /= std::sqrt(A);
            m1 = k * (A - 1.0);
            m2 = A * A - 1.0;
            break;
        case kHighShelf:
            g *= std::sqrt(A);
            m0 = A * A;
            m1 = k * (1.0 - A) * A;
            m2 = 1.0 - A * A;
            break;
    }
    const double a1 = 1.0 / (1.0 + g * (g + k));
    return {a1, g * a1, g * g * a1, m0, m1, m2};
}

// One section in float, its coefficients designed in double. A coefficient change ramps linearly over a control tick:
// a step in a coefficient is a step in the output.
struct SectionF {
    Svf svf;
    float m0 = 1.0f, m1 = 0.0f, m2 = 0.0f;
    float to[6] = {}, step[6] = {};  // the ramp's target {a1, a2, a3, m0, m1, m2} and per-sample step
    int ramp = 0;                    // samples left in the ramp
    // At once.
    BT_ITCM void Set(const SvfCoefs& c) {
        Targets(c);
        Assign();
        ramp = 0;
    }
    // Linearly over the next kControlInterval samples, reaching c exactly.
    BT_ITCM void RampTo(const SvfCoefs& c) {
        Targets(c);
        const float now[6] = {svf.a1, svf.a2, svf.a3, m0, m1, m2};
        for (int i = 0; i < 6; ++i) step[i] = (to[i] - now[i]) * (1.0f / float(kControlInterval));
        ramp = kControlInterval;
    }
    BT_ITCM float Process(float v) {
        if (ramp > 0) {
            if (--ramp == 0) {
                Assign();
            } else {
                svf.a1 += step[0];
                svf.a2 += step[1];
                svf.a3 += step[2];
                m0 += step[3];
                m1 += step[4];
                m2 += step[5];
            }
        }
        float bp;
        const float lp = svf.Process(v, bp);
        return m0 * v + m1 * bp + m2 * lp;
    }
    void Zero() { svf.Clear(); }

private:
    BT_ITCM void Targets(const SvfCoefs& c) {
        to[0] = float(c.a1);
        to[1] = float(c.a2);
        to[2] = float(c.a3);
        to[3] = float(c.m0);
        to[4] = float(c.m1);
        to[5] = float(c.m2);
    }
    BT_ITCM void Assign() {
        svf.a1 = to[0];
        svf.a2 = to[1];
        svf.a3 = to[2];
        m0 = to[3];
        m1 = to[4];
        m2 = to[5];
    }
};

// Every non-enum EQ parameter has the gains' smoothMs (Eq keeps one smoother coefficient).
constexpr bool SharedSmoothing() {
    using namespace btparam;
    for (const ParamDef& d : kParams)
        if (d.group == Group::kEq && d.taper != Taper::kEnum && d.smoothMs != kParams[kEqLowGain].smoothMs)
            return false;
    return true;
}

}  // namespace eq

class Eq {
public:
    static constexpr int kFadeSamples = int(0.02f * kSampleRate);  // enable / position fade (20 ms)
    static constexpr int kIdleTicks = 300;  // control ticks (100 ms at 3 kHz) at identity before a section stops

    // Every eq_* at its table default, state zero, fade 0.
    void Init(float sampleRate) {
        using namespace btparam;
        static_assert(eq::SharedSmoothing(), "the EQ's gains, frequencies and Qs share one smoother coefficient");
        fs_ = sampleRate;
        a_ = double(TimeCoef(SmoothSeconds(kEqLowGain), sampleRate / float(kControlInterval)));
        for (int b = 0; b < eq::kBands; ++b)
            if (eq::kKind[b] != eq::kBell) band_[b].lqT = std::log2(eq::kShelfQ);
        for (const ParamDef& d : kParams)
            if (d.group == Group::kEq) SetParam(d.id, d.def);
        post_ = wantPost_;
        fade_ = 0;
        Start();
    }
    // Group::kEq ids, v already btparam::Clamp()ed; others ignored. A gain within one 14-bit code step of 0 dB is 0:
    // 0 dB is not on the code grid (MIDI's code for 0 dB on a +-12 dB range is +0.0007 dB), and only an exact 0 lets
    // a section stop and the flat EQ stay bit-exact.
    void SetParam(uint8_t id, float v) {
        using namespace btparam;
        if (id == kEqEnable) {
            enable_ = v > 0.5f;
            return;
        }
        if (id == kEqPosition) {
            wantPost_ = v > 0.5f;
            return;
        }
        for (int b = 0; b < eq::kBands; ++b) {
            Band& s = band_[b];
            if (id == eq::kGainId[b]) {
                const ParamDef& p = kParams[id];
                s.dbT = std::fabs(v) < (p.max - p.min) / float(kCodeMax) ? 0.0 : double(v);
            } else if (id == eq::kFreqId[b]) {
                s.lfT = std::log2(double(v));
            } else if (id == eq::kQId[b]) {
                s.lqT = std::log2(double(v));
            }
        }
    }
    // Smoothers at their targets, the fade at its target, the placement at eq_position, state zero.
    void Snap() {
        post_ = wantPost_;
        fade_ = enable_ ? kFadeSamples : 0;
        Start();
    }
    // The stage currently runs after the mix.
    bool Post() const { return post_; }
    // eq_position differs from Post(): the samples until the fade reaches 0 (0: Move() now); else -1.
    int SamplesToMove() const { return wantPost_ == post_ ? -1 : fade_; }
    // Post() = eq_position, all state zeroed. Call only when SamplesToMove() == 0.
    BT_ITCM void Move() {
        post_ = wantPost_;
        ZeroAll();
    }

    // One sample at the current placement.
    BT_ITCM float Process(float x) {
        const int want = enable_ && wantPost_ == post_ ? kFadeSamples : 0;
        if (fade_ < want) {
            if (fade_++ == 0) Start();  // the stage starts at its current settings
        } else if (fade_ > want) {
            --fade_;
        }
        if (fade_ == 0) return x;
        if (++tick_ >= kControlInterval) {
            tick_ = 0;
            Control();
        }
        float v = x;
        for (Band& b : band_)
            if (b.on) v = b.f.Process(v);
        if (fade_ == kFadeSamples) return v;
        const float g = float(fade_) / float(kFadeSamples);
        return (1.0f - g) * x + g * v;
    }

private:
    static constexpr double kDbEps = 1e-3, kLogEps = 1e-5;

    // A one-pole glide stepped per control tick that snaps to its target once within eps (idling needs exact
    // targets).
    struct Smooth {
        double y = 0.0;
        // false when already at t.
        BT_ITCM bool Step(double t, double a, double eps) {
            if (y == t) return false;
            y += a * (t - y);
            if (std::fabs(t - y) < eps) y = t;
            return true;
        }
    };
    // One section: targets (dB, log2 Hz, log2 Q), their glides and the filter.
    struct Band {
        double dbT = 0.0, lfT = 0.0, lqT = 0.0;
        Smooth db, lf, lq;
        bool on = false, dirty = false;  // dirty: a glide moved since the last design
        int idle = 0;                    // control ticks at 0 dB
        eq::SectionF f;
    };

    // Smoothers at targets, all state zero, the sections not at 0 dB on with their coefficients.
    [[gnu::noinline]] void Start() {
        for (int i = 0; i < eq::kBands; ++i) {
            Band& b = band_[i];
            b.db.y = b.dbT;
            b.lf.y = b.lfT;
            b.lq.y = b.lqT;
            b.f.Zero();
            b.on = b.dbT != 0.0;
            b.idle = 0;
            b.dirty = false;
            if (b.on) b.f.Set(Design(i, b.db.y));
        }
        tick_ = 0;
    }
    void ZeroAll() {
        for (Band& b : band_) b.f.Zero();
    }

    // Once per control tick: the glides, sections starting and stopping, coefficients of what moved.
    // noinline (with Start): inlined, their double-precision design made Process save d8-d14 on every sample.
    [[gnu::noinline]] BT_ITCM void Control() {
        for (int i = 0; i < eq::kBands; ++i) {
            Band& b = band_[i];
            bool moved = b.db.Step(b.dbT, a_, kDbEps);
            moved = b.lf.Step(b.lfT, a_, kLogEps) || moved;
            moved = b.lq.Step(b.lqT, a_, kLogEps) || moved;
            b.dirty = b.dirty || moved;
            if (!b.on) {
                if (b.dbT != 0.0) {
                    b.on = true;  // from zero state at its identity (0 dB), ramped below to the glide's first step
                    b.idle = 0;
                    b.f.Set(Design(i, 0.0));
                    b.dirty = true;
                }
            } else if (b.db.y == 0.0 && b.dbT == 0.0) {
                if (++b.idle >= kIdleTicks) {
                    b.on = false;
                    b.dirty = false;
                    b.f.Zero();
                }
            } else {
                b.idle = 0;
            }
            if (b.on && b.dirty) {
                b.f.RampTo(Design(i, b.db.y));
                b.dirty = false;
            }
        }
    }

    // Band i's coefficients at gain db and its glides' current frequency and Q.
    BT_ITCM eq::SvfCoefs Design(int i, double db) const {
        const Band& b = band_[i];
        return eq::Design(eq::kKind[i], std::exp2(b.lf.y), std::exp2(b.lq.y), db, double(fs_));
    }

    float fs_ = 0.0f;  // Init
    double a_ = 1.0;   // the glides' one-pole coefficient per control tick
    bool enable_ = false, wantPost_ = false;
    bool post_ = false;
    int fade_ = 0;  // the stage's crossfade x kFadeSamples
    int tick_ = 0;
    Band band_[eq::kBands];
};

}  // namespace btdsp
