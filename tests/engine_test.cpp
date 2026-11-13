// The chain through btdsp::Engine: the routing (MIDI), the equal-power mixes, the reverb wet levels and bypass
// spillover.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;

// x through a 4-pole Butterworth lowpass at 18 kHz: the band the half-rate reverb passes (halfband.h).
std::vector<float> LowPass18k(std::vector<float> x) {
    btdsp::Butterworth4 f;
    f.Set(18000.0f / kFs);
    for (float& v : x) v = f.Process(v);
    return x;
}

// A 300 ms forward echo without feedback or color.
void SetEcho(btdsp::Engine& e) {
    e.SetParam(btparam::kDelayTime, 300.0f);
    e.SetParam(btparam::kDelayFeedback, 0.0f);
    e.SetParam(btparam::kDelayColor, 0.0f);
    e.SetParam(btparam::kDelayMode, 2.0f);
}

// An impulse through the 300 ms echo at delay mix 0, reverb mix 100: the output is the reverb wet only.
std::vector<float> RenderImpulseWet(float routing) {
    EngineRig r;
    SetEcho(r.e);
    r.e.SetParam(btparam::kRouting, routing);
    r.e.SetParam(btparam::kDelayMix, 0.0f);
    r.e.SetParam(btparam::kReverbMix, 100.0f);
    const std::vector<float> in = Impulse(Samples(0.6), 0.5f);
    std::vector<float> y(in.size());
    r.Feed(in, y, 0, in.size());
    return y;
}

}  // namespace

TEST(Routing, DownFeedsOnlyTheDelayWet) {
    const std::vector<float> y = RenderImpulseWet(0.0f);
    float early = 0.0f;
    for (size_t i = 0; i < Samples(0.25); ++i) early = std::max(early, std::fabs(y[i]));
    EXPECT_LE(early, 1e-6f);  // the dry click is not reverberated
    EXPECT_GT(Rms(y, Samples(0.35), Samples(0.6)), 1e-4);  // the repeat is, although delay mix is 0
}

TEST(Routing, MiddleFeedsTheDelayMix) {
    EngineRig r;
    r.e.SetParam(btparam::kDelayMix, 100.0f);
    r.e.SetParam(btparam::kDelayTime, 300.0f);
    r.e.SetParam(btparam::kDelayFeedback, 0.0f);
    r.e.SetParam(btparam::kReverbMix, 100.0f);
    const std::vector<float> in = Impulse(Samples(0.6), 0.5f);
    std::vector<float> y(in.size());
    r.Feed(in, y, 0, in.size());
    float early = 0.0f;
    for (size_t i = 0; i < Samples(0.25); ++i) early = std::max(early, std::fabs(y[i]));
    EXPECT_LE(early, 1e-6f);  // only the delayed repeat reaches the Hall
    EXPECT_GT(Rms(y, Samples(0.35), Samples(0.6)), 1e-4);
}

// Delay mix 100, reverb mix 100: the output is the reverb wet only. With the routing at dry it must not change whether
// the delay ever recorded anything.
TEST(Routing, UpLeavesRepeatsDry) {
    const auto render = [](float routing, float mode) {
        EngineRig r;
        SetEcho(r.e);
        r.e.SetParam(btparam::kDelayFeedback, 50.0f);
        r.e.SetParam(btparam::kDelayMode, mode);
        r.e.SetParam(btparam::kRouting, routing);
        r.e.SetParam(btparam::kDelayMix, 100.0f);
        r.e.SetParam(btparam::kReverbMix, 100.0f);
        const std::vector<float> in = Impulse(Samples(1.5), 0.5f);
        std::vector<float> y(in.size());
        r.Feed(in, y, 0, in.size());
        return y;
    };
    const size_t end = Samples(1.5);
    EXPECT_LE(MaxAbsDiff(render(2.0f, 2.0f), render(2.0f, 1.0f), 0, end), 1e-6f);
    EXPECT_GT(MaxAbsDiff(render(1.0f, 2.0f), render(1.0f, 1.0f), 0, end), 1e-3f);  // Middle reverberates the repeats
}

TEST(Routing, BypassIsInputPlusTailsThatDie) {
    const size_t n = Samples(6.0);
    // New input starts at quiet, after the 5 ms input and bypass ramps.
    const size_t engage = Samples(0.1), quiet = Samples(0.11);
    const std::vector<float> inB = Impulse(n, 0.5f);
    std::vector<float> inA = inB;
    const std::vector<float> noise = Noise(n, 0.5f, 11);
    for (size_t i = quiet; i < n; ++i) inA[i] += noise[i];
    for (float routing : {0.0f, 1.0f, 2.0f}) {
        SCOPED_TRACE(routing);
        const auto render = [&](const std::vector<float>& in) {
            EngineRig r;
            r.e.SetParam(btparam::kRouting, routing);
            r.e.SetParam(btparam::kDelayTime, 100.0f);
            r.e.SetParam(btparam::kDelayFeedback, 50.0f);
            r.e.SetParam(btparam::kDelayMix, 50.0f);
            r.e.SetParam(btparam::kReverbMix, 50.0f);
            r.e.SetParam(btparam::kHallRtMid, 1.0f);
            std::vector<float> y(n);
            r.Feed(in, y, 0, engage);
            r.e.SetParam(btparam::kBypass, 1.0f);
            r.Feed(in, y, engage, n);
            return y;
        };
        const std::vector<float> yA = render(inA), yB = render(inB);
        float diff = 0.0f;
        for (size_t i = quiet; i < n; ++i) diff = std::max(diff, std::fabs((yA[i] - inA[i]) - yB[i]));
        EXPECT_LE(diff, 1e-6f);  // new input passes at unity, never delayed or reverberated
        EXPECT_GT(Rms(yB, Samples(0.15), Samples(0.5)), 1e-4);  // the tails spill
        EXPECT_LE(Rms(yB, Samples(5.5), n), 1e-5);              // and reach silence
    }
}

// Knob 1 crosses over at 50 %, knob 2 at 75 %.
TEST(Mix, EqualPowerAtCrossover) {
    const float half = 0.5f * std::sqrt(0.5f);
    {
        EngineRig r;
        r.e.SetParam(btparam::kDelayMix, 50.0f);
        r.e.SetParam(btparam::kReverbMix, 0.0f);
        r.e.SetParam(btparam::kDelayTime, 100.0f);
        r.e.SetParam(btparam::kDelayFeedback, 0.0f);
        r.e.SetParam(btparam::kDelayColor, 0.0f);
        const std::vector<float> in = Impulse(Samples(0.2), 0.5f);
        std::vector<float> y(in.size());
        r.Feed(in, y, 0, in.size());
        EXPECT_NEAR(y[0], half, 1e-4f);
        float peak = 0.0f;
        for (size_t i = Samples(0.1) - 5; i <= Samples(0.1) + 5; ++i) peak = std::max(peak, std::fabs(y[i]));
        EXPECT_NEAR(peak, half, 1e-3f);  // the repeat at 100 ms
    }
    {
        EngineRig r;
        r.e.SetParam(btparam::kDelayMix, 0.0f);
        r.e.SetParam(btparam::kReverbMix, 75.0f);
        const std::vector<float> in = Impulse(Samples(0.05), 0.5f);
        std::vector<float> y(in.size());
        r.Feed(in, y, 0, in.size());
        EXPECT_NEAR(y[0], half, 1e-4f);  // the Hall's pre-delay keeps its wet out of sample 0
    }
}

// The reverb wet level against a steady noise input, at each algorithm's defaults.
TEST(Engine, WetLevelMatchesInputForEveryAlgorithm) {
    for (float algo : {0.0f, 1.0f, 2.0f}) {
        SCOPED_TRACE(algo);
        EngineRig r;
        r.e.SetParam(btparam::kDelayMix, 0.0f);
        r.e.SetParam(btparam::kReverbMix, 100.0f);
        r.e.SetParam(btparam::kReverbAlgo, algo);
        const std::vector<float> in = LowPass18k(Noise(Samples(2.0), 0.5f, 9));
        std::vector<float> y(in.size());
        r.Feed(in, y, 0, in.size());
        const double db = 20.0 * std::log10(Rms(y, Samples(1.0), y.size()) / Rms(in, Samples(1.0), in.size()));
        EXPECT_NEAR(db, 0.0, 2.0) << db;
    }
}
