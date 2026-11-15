// Helpers shared by the test files: sample counts, test signals, levels, a click measure, a tone's amplitude, an engine
// rig with the engine render and the delay rig built on it, the color scopes' loop probe, a rig running one reverb
// algorithm on its own zeroed pool, an RT60 measure and the checks every reverb algorithm meets.
#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "dsp/blocks.h"
#include "dsp/engine.h"
#include "dsp/reverb_pool.h"
#include "params.h"

namespace bttest {

constexpr float kFs = btdsp::kSampleRate;
constexpr float kReverbFs = btdsp::kReverbRate;  // the reverbs' rate (the engine runs them at half kFs)
constexpr size_t kBlock = btdsp::kBlockSize;

// One algorithm R on a zeroed pool of its own, at kReverbFs.
template <class R> struct ReverbRig {
    std::vector<float> fast = std::vector<float>(btdsp::kReverbFastFloats);
    std::vector<float> slow = std::vector<float>(btdsp::kReverbSlowFloats);
    R rev;
    bool primed = false;
    ReverbRig() { rev.Init(kReverbFs, btdsp::ReverbPool{fast.data(), slow.data()}); }
    void Set(uint8_t id, float v) { rev.SetParam(id, btparam::Clamp(btparam::kParams[id], v)); }
    // Like the engine: parameters set before the first call apply without smoothing.
    void Process(const float* in, float* l, float* r, size_t n) {
        if (!primed) rev.Snap();
        primed = true;
        rev.Process(in, l, r, n);
    }
    // L and R of in, in kBlock-sample blocks.
    void Render(const std::vector<float>& in, std::vector<float>& l, std::vector<float>& r) {
        l.assign(in.size(), 0.0f);
        r.assign(in.size(), 0.0f);
        for (size_t i = 0; i < in.size(); i += kBlock)
            Process(in.data() + i, l.data() + i, r.data() + i, std::min(kBlock, in.size() - i));
    }
    // The mono fold, kReverbFold * (L + R).
    std::vector<float> Render(const std::vector<float>& in) {
        std::vector<float> l, r;
        Render(in, l, r);
        for (size_t i = 0; i < l.size(); ++i) l[i] = btdsp::kReverbFold * (l[i] + r[i]);
        return l;
    }
};

inline size_t Samples(double s, float fs = kFs) { return size_t(std::lround(s * fs)); }

inline std::vector<float> Noise(size_t n, float amp, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(-amp, amp);
    std::vector<float> x(n);
    for (float& v : x) v = u(rng);
    return x;
}

inline std::vector<float> Impulse(size_t n, float amp) {
    std::vector<float> x(n, 0.0f);
    x[0] = amp;
    return x;
}

inline double Rms(const std::vector<float>& y, size_t from, size_t to) {
    double acc = 0.0;
    for (size_t i = from; i < to; ++i) acc += double(y[i]) * y[i];
    return std::sqrt(acc / double(to - from));
}

// n samples of a sine at hz with peak amp.
inline std::vector<float> Sine(size_t n, float hz, float amp, float fs = kFs) {
    std::vector<float> x(n);
    for (size_t i = 0; i < n; ++i) x[i] = amp * std::sin(2.0f * float(M_PI) * hz * float(i) / fs);
    return x;
}

inline float MaxAbsDiff(const std::vector<float>& a, const std::vector<float>& b, size_t from, size_t to) {
    float m = 0.0f;
    for (size_t i = from; i < to; ++i) m = std::max(m, std::fabs(a[i] - b[i]));
    return m;
}

// The largest second difference from sample `from` on: a step or kink shows up here far above a smooth signal's level.
inline float Click(const std::vector<float>& y, size_t from = 0) {
    float m = 0.0f;
    for (size_t i = std::max<size_t>(from, 2); i < y.size(); ++i)
        m = std::max(m, std::fabs(y[i] - 2.0f * y[i - 1] + y[i - 2]));
    return m;
}

// The amplitude of y[from, to) at hz (Goertzel; a sine of peak a reads a).
inline double Amplitude(const std::vector<float>& y, size_t from, size_t to, double hz, float fs = kFs) {
    const double w = 2.0 * M_PI * hz / fs, c = 2.0 * std::cos(w);
    double s1 = 0.0, s2 = 0.0;
    for (size_t i = from; i < to; ++i) {
        const double s0 = y[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    const double re = s1 - s2 * std::cos(w), im = s2 * std::sin(w);
    return 2.0 * std::sqrt(re * re + im * im) / double(to - from);
}

// Rising crossings of y[from, to) with +-0.05 hysteresis: the mean frequency between the first and the last.
inline double Frequency(const std::vector<float>& y, size_t from, size_t to, float fs = kFs) {
    bool low = false;
    size_t first = 0, last = 0, n = 0;
    for (size_t i = from; i < to; ++i) {
        if (y[i] < -0.05f) {
            low = true;
        } else if (low && y[i] > 0.05f) {
            low = false;
            if (n++ == 0) first = i;
            last = i;
        }
    }
    return n < 2 ? 0.0 : double(n - 1) * fs / double(last - first);
}

// The SOS line the rigs use by default: 2^15 samples (0.34 s), so the ~5 000 rigs of NoClickOnParamJump stay fast;
// the SOS tests pass btdsp::kSosSamples.
constexpr size_t kTestSosSamples = size_t(1) << 15;

// The whole chain on zeroed memory, initialised at kFs, with an SOS line of sosSamples.
struct EngineRig {
    std::vector<float> mem = std::vector<float>(btdsp::kDelaySamples);
    std::vector<float> sos;
    std::vector<float> fast = std::vector<float>(btdsp::kReverbFastFloats);
    std::vector<float> slow = std::vector<float>(btdsp::kReverbSlowFloats);
    btdsp::Engine e;
    explicit EngineRig(size_t sosSamples = kTestSosSamples) : sos(sosSamples) {
        e.Init(kFs, mem.data(), btdsp::SosLine{sos.data(), sos.size()},
               btdsp::ReverbPool{fast.data(), slow.data()});
    }
    void Set(uint8_t id, float v) { e.SetParam(id, v); }
    void Process(const float* in, float* out, size_t n) { e.Process(in, out, n); }
    // in[from, to) into out, in blocks of block.
    void Feed(const std::vector<float>& in, std::vector<float>& out, size_t from, size_t to, size_t block = kBlock) {
        for (size_t i = from; i < to; i += block) e.Process(in.data() + i, out.data() + i, std::min(block, to - i));
    }
    // in through the chain, in kBlock blocks.
    std::vector<float> Render(const std::vector<float>& in) {
        std::vector<float> out(in.size());
        Feed(in, out, 0, in.size());
        return out;
    }
};

// The delay tests' signal amplitude.
constexpr float kAmp = 0.25f;

// An engine with the delay at 100 % wet, no feedback, digital without color, forward; the tape and BBD colors in the
// loop (scope repeats; the scope tests set all).
struct DelayRig : EngineRig {
    explicit DelayRig(size_t sosSamples = kTestSosSamples) : EngineRig(sosSamples) {
        Set(btparam::kReverbMix, 0.0f);  // the delay alone
        Set(btparam::kDelayMix, 100.0f);
        Set(btparam::kDelayFeedback, 0.0f);
        Set(btparam::kDelayType, 2.0f);
        Set(btparam::kDelayColor, 0.0f);
        Set(btparam::kDelayMode, 2.0f);
        Set(btparam::kDelayTapeScope, 0.0f);
        Set(btparam::kDelayBbdScope, 0.0f);
    }
};

// The color scopes' loop probe: a 50 ms burst of hz at kAmp through r at 100 ms and 50 % feedback; the largest
// |second repeat - 0.5 x first repeat| over the burst (0 for a linear loop at unity).
inline float SecondRepeatError(EngineRig& r, float hz) {
    std::vector<float> in = Sine(Samples(0.05), hz, kAmp);
    in.resize(Samples(0.4), 0.0f);
    r.Set(btparam::kDelayTime, 100.0f);
    r.Set(btparam::kDelayFeedback, 50.0f);
    const std::vector<float> out = r.Render(in);
    const size_t d = Samples(0.1);
    float m = 0.0f;
    for (size_t i = 0; i < Samples(0.05); ++i) m = std::max(m, std::fabs(out[2 * d + i] - 0.5f * out[d + i]));
    return m;
}

// RT60 at hz: y through two cascaded SVF bandpasses at hz, levels in 20 ms blocks, and a least-squares line over the
// blocks after the peak from 5 dB to 35 dB below it; -1 when fewer than 2 blocks fall in that range. An approximation
// of tools/features.py rt60(), not a copy.
inline double Rt60(const std::vector<float>& y, float hz, float fs = kFs) {
    btdsp::Svf f[2];
    constexpr float k = 0.7071f;
    for (btdsp::Svf& s : f) s.Set(hz / fs, k);
    const size_t blk = Samples(0.02, fs);
    std::vector<double> lv;
    double acc = 0.0;
    for (size_t i = 0; i < y.size(); ++i) {
        float b;
        f[0].Process(y[i], b);
        f[1].Process(k * b, b);
        acc += double(k * b) * (k * b);
        if ((i + 1) % blk == 0) {
            lv.push_back(10.0 * std::log10(acc / double(blk) + 1e-30));
            acc = 0.0;
        }
    }
    const size_t ip = size_t(std::max_element(lv.begin(), lv.end()) - lv.begin());
    std::vector<double> t, l;
    for (size_t i = ip + 1; i < lv.size() && lv[i] >= lv[ip] - 35.0; ++i) {
        if (lv[i] > lv[ip] - 5.0) continue;
        t.push_back(double(i) * 0.02);
        l.push_back(lv[i]);
    }
    if (t.size() < 2) return -1.0;
    double mt = 0, ml = 0;
    for (size_t i = 0; i < t.size(); ++i) {
        mt += t[i];
        ml += l[i];
    }
    mt /= double(t.size());
    ml /= double(t.size());
    double num = 0, den = 0;
    for (size_t i = 0; i < t.size(); ++i) {
        num += (t[i] - mt) * (l[i] - ml);
        den += (t[i] - mt) * (t[i] - mt);
    }
    const double slope = num / den;
    return slope < 0.0 ? -60.0 / slope : -1.0;
}

// The sample index of the peak 10 ms RMS block.
inline size_t EnvelopePeak(const std::vector<float>& y, float fs = kFs) {
    const size_t blk = Samples(0.01, fs);
    size_t best = 0;
    double bestRms = -1.0;
    for (size_t i = 0; i + blk <= y.size(); i += blk) {
        const double r = Rms(y, i, i + blk);
        if (r > bestRms) {
            bestRms = r;
            best = i;
        }
    }
    return best;
}

// Every 2^8 min/max corner of the eight loop parameters ids: finite, bounded, and decaying. The decay is measured on
// the L and R energy, not the fold: the tank's energy never grows, but at the slowest corners (the Hall's RT x BASS
// MULT 80 s in a 1 m room) the fold's L/R phase relation drifts by about 1 dB while the tank barely decays.
template <class R> void ExpectDecayStableAtCorners(const uint8_t (&ids)[8]) {
    using btparam::kParams;
    std::vector<float> in = Noise(Samples(3.0, kReverbFs), 0.5f, 7);
    std::fill(in.begin() + Samples(0.05, kReverbFs), in.end(), 0.0f);
    for (unsigned corner = 0; corner < 256; ++corner) {
        SCOPED_TRACE(corner);
        ReverbRig<R> r;
        for (int b = 0; b < 8; ++b) r.Set(ids[b], (corner >> b) & 1 ? kParams[ids[b]].max : kParams[ids[b]].min);
        std::vector<float> l, rr, e(in.size());
        r.Render(in, l, rr);
        float peak = 0.0f;
        size_t bad = 0;
        for (size_t i = 0; i < in.size(); ++i) {
            bad += !std::isfinite(l[i]) || !std::isfinite(rr[i]);
            const float fold = btdsp::kReverbFold * (l[i] + rr[i]);
            peak = std::max({peak, std::fabs(l[i]), std::fabs(rr[i]), std::fabs(fold)});
            e[i] = std::sqrt(0.5f * (l[i] * l[i] + rr[i] * rr[i]));
        }
        EXPECT_EQ(bad, 0u);
        EXPECT_LE(peak, 4.0f);
        EXPECT_LE(Rms(e, Samples(2.5, kReverbFs), Samples(3.0, kReverbFs)),
                  Rms(e, Samples(0.5, kReverbFs), Samples(1.0, kReverbFs)) + 1e-9);
    }
}

// |correlation| of L and R for 2 s of noise, after the first 0.5 s.
template <class R> double StereoCorrelation() {
    ReverbRig<R> r;
    std::vector<float> l, rr;
    r.Render(Noise(Samples(2.0, kReverbFs), 0.5f, 8), l, rr);
    double ll = 0, rrs = 0, lr = 0;
    for (size_t i = Samples(0.5, kReverbFs); i < l.size(); ++i) {
        ll += double(l[i]) * l[i];
        rrs += double(rr[i]) * rr[i];
        lr += double(l[i]) * rr[i];
    }
    return std::fabs(lr) / std::sqrt(ll * rrs);
}

}  // namespace bttest
