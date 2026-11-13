// The EQ (dsp/eq.h, docs/EQ.md): the model (eq_model.h) against the specification's laws and reference table, the
// digital stage against the model within the section 4 tolerance, symmetry, identity, a click-free section onset
// and robustness; and its placement in the engine (pre, post, bypass).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <random>
#include <string>
#include <vector>

#include "dsp/eq.h"
#include "eq_model.h"
#include "test_util.h"

namespace {

using namespace bttest;
using namespace btparam;
namespace eqm = btdsp::eq;
constexpr double kPi = btdsp::kPiT<double>;

// A +12 dB Q 8 bell at 40 Hz has a pole Q of 16: a 127 ms time constant; 2^17 samples are ten of them.
constexpr size_t kIrSamples = size_t(1) << 17;

// docs/EQ.md section 4: the stage within this of the model, 20 Hz to 20 kHz.
constexpr double kTol = 0.05;
// Values printed to 2 decimals.
constexpr double kTol2 = 0.0051;

// The stage on its own, enabled unless told otherwise. Step() does what the engine does with a pending placement
// change: Move() once the stage has faded out.
struct EqRig {
    btdsp::Eq eq;
    explicit EqRig(bool enable = true) {
        eq.Init(kFs);
        Set(kEqEnable, enable ? 1.0f : 0.0f);
    }
    void Set(uint8_t id, float v) { eq.SetParam(id, Clamp(kParams[id], v)); }
    void Apply(const EqSettings& s) {
        for (int b = 0; b < eqm::kBands; ++b) {
            Set(eqm::kGainId[b], float(s.band[b].gain));
            Set(eqm::kFreqId[b], float(s.band[b].freq));
            if (eqm::kQId[b] != kNoParam) Set(eqm::kQId[b], float(s.band[b].q));
        }
    }
    void Snap() { eq.Snap(); }
    float Step(float x) {
        if (eq.SamplesToMove() == 0) eq.Move();
        return eq.Process(x);
    }
    std::vector<float> Process(const std::vector<float>& in) {
        std::vector<float> y(in.size());
        for (size_t i = 0; i < in.size(); ++i) y[i] = Step(in[i]);
        return y;
    }
};

// 61 points, 1/6 octave apart, 20 Hz to 20 kHz.
std::vector<double> Grid() {
    std::vector<double> f(61);
    for (int k = 0; k <= 60; ++k) f[size_t(k)] = 20.0 * std::pow(1000.0, k / 60.0);
    return f;
}

// The rig's magnitude (dB) at freqs: its settings snapped, the DFT of its impulse response.
std::vector<double> DigitalDb(EqRig& r, const std::vector<double>& freqs) {
    r.Snap();
    std::vector<float> y(kIrSamples);
    for (size_t n = 0; n < kIrSamples; ++n) y[n] = r.Step(n == 0 ? 1.0f : 0.0f);
    std::vector<double> db;
    for (double f : freqs) {
        const Cplx rot = std::polar(1.0, -2.0 * kPi * f / double(kFs));
        Cplx z = 1.0, acc = 0.0;
        for (float v : y) {
            acc += double(v) * z;
            z *= rot;
        }
        db.push_back(Db(acc));
    }
    return db;
}

std::vector<double> ModelDb(const EqSettings& s, const std::vector<double>& freqs) {
    std::vector<double> db;
    for (double f : freqs) db.push_back(EqModelDb(s, f));
    return db;
}

// Setting A (docs/EQ.md section 3): low +6 dB at 80 Hz; bell 1 -4 dB at 250 Hz Q 1.4; bell 2 +3 dB at 1.2 kHz Q 0.7;
// bell 3 +5 dB at 3.5 kHz Q 2; high -6 dB at 8 kHz.
EqSettings SettingA() {
    EqSettings s;
    s.band[0] = {6.0, 80.0, eqm::kShelfQ};
    s.band[1] = {-4.0, 250.0, 1.4};
    s.band[2] = {3.0, 1200.0, 0.7};
    s.band[3] = {5.0, 3500.0, 2.0};
    s.band[4] = {-6.0, 8000.0, eqm::kShelfQ};
    return s;
}

}  // namespace

TEST(EqModel, LawsHold) {
    for (int b = 0; b < eqm::kBands; ++b) {
        const eqm::Kind k = eqm::kKind[b];
        const double f0 = kParams[eqm::kFreqId[b]].def;
        const double q = k == eqm::kBell ? 1.4 : eqm::kShelfQ;
        for (double g : {-12.0, -6.0, 6.0, 12.0}) {
            SCOPED_TRACE(::testing::Message() << kParams[eqm::kGainId[b]].name << " " << g << " dB");
            const BandSettings up{g, f0, q}, down{-g, f0, q};
            EXPECT_NEAR(Db(BandModel(k, f0, up)), k == eqm::kBell ? g : 0.5 * g, 1e-9) << "gain at f0";
            for (double f : {20.0, f0, 10000.0})
                EXPECT_NEAR(std::abs(BandModel(k, f, up) * BandModel(k, f, down) - 1.0), 0.0, 1e-9) << "mirror at " << f;
        }
    }
}

TEST(EqModel, WholeSettingsMatchTable) {
    const double f[] = {30, 50, 80, 120, 250, 500, 1000, 1200, 2000, 3500, 5000, 8000, 12000, 16000};
    const double a[] = {5.85, 5.09, 2.78, 0.42, -3.66, 0.31, 2.79, 3.09, 2.64, 5.48, 1.14, -2.53, -4.89, -5.63};
    const EqSettings s = SettingA();
    for (size_t i = 0; i < std::size(f); ++i) EXPECT_NEAR(EqModelDb(s, f[i]), a[i], kTol2) << "A at " << f[i] << " Hz";
}

TEST(Eq, MatchesModelWithinTolerance) {
    const std::vector<double> freqs = Grid();
    const auto check = [&freqs](const EqSettings& s, const std::string& what) {
        EqRig r;
        r.Apply(s);
        const std::vector<double> d = DigitalDb(r, freqs), m = ModelDb(s, freqs);
        for (size_t i = 0; i < freqs.size(); ++i) EXPECT_NEAR(d[i], m[i], kTol) << what << " at " << freqs[i] << " Hz";
    };
    for (int b = 0; b < eqm::kBands; ++b) {
        const ParamDef& fp = kParams[eqm::kFreqId[b]];
        const std::vector<double> qs =
            eqm::kKind[b] == eqm::kBell ? std::vector<double>{0.4, 8.0} : std::vector<double>{eqm::kShelfQ};
        for (double f0 : {double(fp.min), double(fp.def), double(fp.max)})
            for (double g : {-12.0, 12.0})
                for (double q : qs) {
                    EqSettings s;
                    s.band[b] = {g, f0, q};
                    check(s, (::testing::Message() << kParams[eqm::kGainId[b]].name << " " << g << " dB at " << f0
                                                   << " Hz Q " << q)
                                 .GetString());
                }
    }
    check(SettingA(), "setting A");
}

TEST(Eq, CutMirrorsBoost) {
    const std::vector<double> freqs = Grid();
    EqSettings up = SettingA(), down = up;
    for (BandSettings& b : down.band) b.gain = -b.gain;
    EqRig ru, rd;
    ru.Apply(up);
    rd.Apply(down);
    const std::vector<double> du = DigitalDb(ru, freqs), dd = DigitalDb(rd, freqs);
    for (size_t i = 0; i < freqs.size(); ++i) EXPECT_LE(std::fabs(du[i] + dd[i]), 0.1) << "at " << freqs[i] << " Hz";
}

TEST(Eq, IdentityIsBitExact) {
    const std::vector<float> x = Noise(Samples(1.0), 0.5f, 3);
    const auto same = [](const float* a, const float* b, size_t n) { return std::memcmp(a, b, n * sizeof(float)) == 0; };
    {  // (a) off, with settings that would color it
        EqRig r(false);
        r.Set(kEqLowGain, 12.0f);
        r.Set(kEqMidGain2, 6.0f);
        r.Snap();
        EXPECT_TRUE(same(r.Process(x).data(), x.data(), x.size())) << "eq_enable off";
    }
    {  // (b) on and flat
        EqRig r;
        r.Snap();
        EXPECT_TRUE(same(r.Process(x).data(), x.data(), x.size())) << "on, flat";
    }
    {  // (d) on, the gains at 0 dB as MIDI delivers them (the 14-bit code nearest 0 dB)
        EqRig r;
        for (uint8_t id : eqm::kGainId) r.Set(id, FromCode(kParams[id], ToCode(kParams[id], 0.0f)));
        r.Snap();
        EXPECT_TRUE(same(r.Process(x).data(), x.data(), x.size())) << "on, 0 dB by code";
    }
    {  // (c) on, a boost returned to 0 dB
        EqRig r;
        r.Set(kEqLowGain, 12.0f);
        r.Snap();
        const std::vector<float> in = Noise(Samples(1.5), 0.5f, 4);
        std::vector<float> y(in.size());
        const size_t change = Samples(0.5);
        for (size_t i = 0; i < in.size(); ++i) {
            if (i == change) r.Set(kEqLowGain, 0.0f);
            y[i] = r.Step(in[i]);
        }
        EXPECT_FALSE(same(y.data(), in.data(), change)) << "the boost is heard";
        const size_t from = Samples(1.0);
        EXPECT_TRUE(same(y.data() + from, in.data() + from, in.size() - from)) << "back at 0 dB";
    }
}

// A section starting from 0 dB (off, zero state) ramps from its identity coefficients: no step in the output (the
// high shelf's m0 = A^2 scales the whole signal, so starting it at its first glide step was a click).
TEST(Eq, BandStartingFromFlatDoesNotClick) {
    const size_t half = Samples(0.5);
    const std::vector<float> in = Sine(2 * half, 220.0f, 0.25f);
    for (int b = 0; b < eqm::kBands; ++b) {
        for (float g : {12.0f, -12.0f}) {
            EqRig r;
            r.Snap();
            std::vector<float> y(in.size());
            for (size_t i = 0; i < in.size(); ++i) {
                if (i == half) r.Set(eqm::kGainId[b], g);
                y[i] = r.Step(in[i]);
            }
            const std::vector<float> held(y.begin(), y.begin() + std::ptrdiff_t(half));
            EXPECT_LE(Click(y, half), 2.0f * Click(held) + 1e-3f) << "band " << b << " to " << g << " dB";
        }
    }
}

TEST(Eq, RandomChangesStayBounded) {
    EqRig r;
    const std::vector<float> x = Noise(Samples(20.0), 1.0f, 6);
    std::vector<uint8_t> ids;
    for (const ParamDef& d : kParams)
        if (d.group == Group::kEq) ids.push_back(d.id);
    std::mt19937 rng(5);
    std::uniform_int_distribution<size_t> pick(0, ids.size() - 1);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    size_t bad = 0;
    float peak = 0.0f;
    for (size_t i0 = 0; i0 < x.size(); i0 += 48) {
        if (i0 % 480 == 0) {
            const ParamDef& d = kParams[ids[pick(rng)]];
            r.Set(d.id, d.min + unit(rng) * (d.max - d.min));
        }
        for (size_t i = i0; i < std::min(i0 + 48, x.size()); ++i) {
            const float y = r.Step(x[i]);
            bad += !std::isfinite(y) || std::fpclassify(y) == FP_SUBNORMAL;
            peak = std::max(peak, std::fabs(y));
        }
    }
    EXPECT_EQ(bad, 0u);
    EXPECT_LE(peak, 64.0f);
}

namespace {

// The placement tests' chain: tape at full color (nonlinear, so the EQ's place matters), both mixes at 50 %.
void SetChain(EngineRig& r) {
    r.Set(kDelayType, 1.0f);
    r.Set(kDelayColor, 100.0f);
    r.Set(kDelayMix, 50.0f);
    r.Set(kReverbMix, 50.0f);
}

// Low +12 dB at 100 Hz, bell 2 -9 dB at 1.6 kHz Q 1, high +8 dB at 8 kHz, bell 1 +6 dB at 300 Hz Q 2.
template <class R> void SetPlacementEq(R& r) {
    r.Set(kEqLowGain, 12.0f);
    r.Set(kEqLowFreq, 100.0f);
    r.Set(kEqMidGain2, -9.0f);
    r.Set(kEqMidFreq2, 1600.0f);
    r.Set(kEqMidQ2, 1.0f);
    r.Set(kEqHighGain, 8.0f);
    r.Set(kEqHighFreq, 8000.0f);
    r.Set(kEqMidGain1, 6.0f);
    r.Set(kEqMidFreq1, 300.0f);
    r.Set(kEqMidQ1, 2.0f);
}

std::vector<float> RenderEngine(const std::vector<float>& in, bool eq, float position) {
    EngineRig r;
    SetChain(r);
    if (eq) {
        r.Set(kEqEnable, 1.0f);
        r.Set(kEqPosition, position);
        SetPlacementEq(r);
    }
    std::vector<float> y(in.size());
    r.Feed(in, y, 0, in.size());
    return y;
}

std::vector<float> RenderEq(const std::vector<float>& in) {
    EqRig r;
    SetPlacementEq(r);
    r.Snap();
    return r.Process(in);
}

}  // namespace

TEST(EqPlacement, PreFeedsTheChain) {
    const std::vector<float> in = Noise(Samples(2.0), 0.25f, 9);
    const std::vector<float> pre = RenderEngine(in, true, 0.0f);
    EXPECT_LE(MaxAbsDiff(pre, RenderEngine(RenderEq(in), false, 0.0f), 0, in.size()), 1e-5f);
    EXPECT_GT(MaxAbsDiff(pre, RenderEngine(in, true, 1.0f), 0, in.size()), 1e-3f);  // the place matters
}

TEST(EqPlacement, PostFollowsTheChain) {
    const std::vector<float> in = Noise(Samples(2.0), 0.25f, 9);
    const std::vector<float> post = RenderEngine(in, true, 1.0f);
    EXPECT_LE(MaxAbsDiff(post, RenderEq(RenderEngine(in, false, 0.0f)), 0, in.size()), 1e-5f);
    EXPECT_GT(MaxAbsDiff(post, RenderEngine(in, true, 0.0f), 0, in.size()), 1e-3f);
}

// Routing.BypassIsInputPlusTailsThatDie with the EQ on in either place: new input passes unequalised.
TEST(EqPlacement, BypassPassesTheDryUnequalised) {
    const size_t n = Samples(6.0);
    // New input starts at quiet, after the 5 ms input and bypass ramps.
    const size_t engage = Samples(0.1), quiet = Samples(0.11);
    const std::vector<float> inB = Impulse(n, 0.5f);
    std::vector<float> inA = inB;
    const std::vector<float> noise = Noise(n, 0.5f, 11);
    for (size_t i = quiet; i < n; ++i) inA[i] += noise[i];
    for (float position : {0.0f, 1.0f}) {
        SCOPED_TRACE(position);
        const auto render = [&](const std::vector<float>& in) {
            EngineRig r;
            r.Set(kEqEnable, 1.0f);
            r.Set(kEqPosition, position);
            r.Set(kEqLowGain, 12.0f);
            r.Set(kEqMidGain1, 6.0f);
            r.Set(kDelayTime, 100.0f);
            r.Set(kDelayFeedback, 50.0f);
            r.Set(kDelayMix, 50.0f);
            r.Set(kReverbMix, 50.0f);
            r.Set(kHallRtMid, 1.0f);
            std::vector<float> y(n);
            r.Feed(in, y, 0, engage);
            r.Set(kBypass, 1.0f);
            r.Feed(in, y, engage, n);
            return y;
        };
        const std::vector<float> yA = render(inA), yB = render(inB);
        float diff = 0.0f;
        for (size_t i = quiet; i < n; ++i) diff = std::max(diff, std::fabs((yA[i] - inA[i]) - yB[i]));
        EXPECT_LE(diff, 1e-6f);  // new input passes at unity, never equalised, delayed or reverberated
        EXPECT_GT(Rms(yB, Samples(0.15), Samples(0.5)), 1e-4);  // the tails spill
        EXPECT_LE(Rms(yB, Samples(5.5), n), 1e-5);              // and reach silence
    }
}
