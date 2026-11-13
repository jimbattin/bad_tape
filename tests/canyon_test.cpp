// The Canyon reverb: FEEDBACK sets the decay, the decay stays bounded at every corner of the
// loop and sweep parameters, the stereo pair is decorrelated (the fold gain), MIX 0 passes the trimmed input,
// canyon_space (knob 3) lengthens the decay, and DELAY GLIDING turned on mid-cross-fade keeps the new base.
#include "dsp/canyon.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;
using CanyonRig = ReverbRig<btdsp::Canyon>;

}  // namespace

TEST(Canyon, FeedbackSetsDecay) {
    double last = 0.0;
    for (float fb : {50.0f, 75.0f, 90.0f, 100.0f}) {
        SCOPED_TRACE(fb);
        CanyonRig r;
        r.Set(btparam::kCanyonFeedback, fb);
        const double m = Rt60(r.Render(Impulse(Samples(10.0, kReverbFs), 0.5f)), 1000.0f, kReverbFs);
        EXPECT_GT(m, last);
        last = m;
    }
}

TEST(Canyon, DecayStableAtEveryCorner) {
    using namespace btparam;
    const uint8_t ids[] = {kCanyonFeedback,    kCanyonFeedbackInvert, kCanyonMasterDelay, kCanyonMasterRate,
                           kCanyonMasterDepth, kCanyonDelay1,         kCanyonRate1,       kCanyonDepth1};
    ExpectDecayStableAtCorners<btdsp::Canyon>(ids);
}

// Justifies kReverbFold = 1/sqrt(2): decorrelated L and R add in power.
TEST(Canyon, StereoIsDecorrelated) { EXPECT_LT(StereoCorrelation<btdsp::Canyon>(), 0.5); }

TEST(Canyon, MixZeroPassesTrimmedInput) {
    CanyonRig r;
    r.Set(btparam::kCanyonMix, 0.0f);
    r.Set(btparam::kCanyonInTrim, -6.0f);
    const std::vector<float> in = Noise(Samples(0.5, kReverbFs), 0.5f, 11);
    const std::vector<float> y = r.Render(in);
    const float g = std::pow(10.0f, -6.0f / 20.0f);
    float err = 0.0f;
    for (size_t i = 0; i < in.size(); ++i) err = std::max(err, std::fabs(y[i] - g * in[i]));
    EXPECT_LE(err, 1e-5f);
}

// Knob 3: each canyon_space target at its law's value; RT60 rises with space.
TEST(Canyon, SpaceLengthensDecay) {
    using namespace btparam;
    double last = 0.0;
    for (float space : {0.0f, 50.0f, 100.0f}) {
        SCOPED_TRACE(space);
        CanyonRig r;
        for (const Link& l : kLinks)
            if (l.source == kCanyonSpace) r.Set(l.target, LinkValue(l, space));
        const double m = Rt60(r.Render(Impulse(Samples(8.0, kReverbFs), 0.5f)), 1000.0f, kReverbFs);
        EXPECT_GT(m, last);
        last = m;
    }
}

// DELAY GLIDING turned on during a base cross-fade continues from the new base once the fade ends: the output then
// matches turning it on after the fade.
TEST(Canyon, GlidingOnDuringBaseFadeMatchesAfter) {
    const size_t change = Samples(1.0, kReverbFs), end = change + 2048 + Samples(0.5, kReverbFs);
    const std::vector<float> in = Sine(end, 220.0f, 0.25f, kReverbFs);
    const auto render = [&](size_t glideAt) {
        CanyonRig r;
        r.Set(btparam::kCanyonDelayGliding, 0.0f);
        std::vector<float> l(end), rr(end), y(end);
        for (size_t i = 0; i < end; i += kBlock) {
            if (i == change) r.Set(btparam::kCanyonMasterDelay, 25.0f);  // default 75: a base jump, so a fade
            if (i == glideAt) r.Set(btparam::kCanyonDelayGliding, 1.0f);
            r.Process(in.data() + i, l.data() + i, rr.data() + i, std::min(kBlock, end - i));
        }
        for (size_t i = 0; i < end; ++i) y[i] = btdsp::kReverbFold * (l[i] + rr[i]);
        return y;
    };
    const std::vector<float> mid = render(change + 256), after = render(change + 2048);
    const size_t from = change + 2048 + kBlock;
    EXPECT_EQ(std::memcmp(mid.data() + from, after.data() + from, (end - from) * sizeof(float)), 0);
}
