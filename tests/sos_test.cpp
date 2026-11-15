// Sound on sound (dsp/delay.h) through btdsp::Engine: the 301 head plays once at delay_sos_length, the 501 head
// repeats at delay_sos_ratio x the delay time through the delay feedback, overdub keeps delay_sos_feedback of the loop
// per pass without the main head echoing it, and SOS off leaves the output bit-identical.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;
using namespace btparam;

constexpr size_t kD = 9600;     // 100 ms
constexpr size_t kL = 960000;   // 10 s

// An impulse kAmp at 0 through r (100 ms delay), n samples.
std::vector<float> ImpulseThrough(DelayRig& r, size_t n) {
    r.Set(kDelayTime, 100.0f);
    return r.Render(Impulse(n, kAmp));
}

float MaxAbs(const std::vector<float>& y, size_t from, size_t to) {
    float m = 0.0f;
    for (size_t i = from; i < to; ++i) m = std::max(m, std::fabs(y[i]));
    return m;
}

}  // namespace

TEST(Sos, Mode301PlaysOnceAtLength) {
    DelayRig r(btdsp::kSosSamples);
    r.Set(kDelayFeedback, 0.0f);
    r.Set(kDelaySosMode, 1.0f);
    r.Set(kDelaySos, 1.0f);
    r.Set(kDelaySosLength, 10.0f);
    const std::vector<float> out = ImpulseThrough(r, 2 * kL + 201);
    EXPECT_NEAR(std::fabs(out[kL]), std::fabs(out[kD]), 0.02f * std::fabs(out[kD]));
    size_t peak = kL - 200;
    for (size_t i = kL - 200; i <= kL + 200; ++i)
        if (std::fabs(out[i]) > std::fabs(out[peak])) peak = i;
    EXPECT_EQ(peak, kL);
    EXPECT_LT(MaxAbs(out, 2 * kL - 200, 2 * kL + 201), 1e-4f);
}

TEST(Sos, Mode501RepeatsAtRatio) {
    const auto render = [](float sos) {
        DelayRig r;
        r.Set(kDelayFeedback, 50.0f);
        r.Set(kDelaySosMode, 2.0f);
        r.Set(kDelaySos, sos);
        r.Set(kDelaySosRatio, 2.5f);
        return ImpulseThrough(r, 4 * kD);
    };
    const std::vector<float> on = render(1.0f);
    EXPECT_NEAR(std::fabs(on[5 * kD / 2]), kAmp, 0.02f * kAmp);
    // Main then SOS (D + 2.5 D) and SOS then main (2.5 D + D), each at the 50 % feedback.
    EXPECT_NEAR(std::fabs(on[7 * kD / 2]), 2.0f * 0.5f * kAmp, 0.03f * kAmp);
    EXPECT_LT(std::fabs(render(0.0f)[7 * kD / 2]), 1e-4f);
}

TEST(Sos, OverdubKeepsLoop) {
    DelayRig r(btdsp::kSosSamples);
    r.Set(kDelayFeedback, 0.0f);
    r.Set(kDelaySosMode, 3.0f);
    r.Set(kDelaySos, 1.0f);
    r.Set(kDelaySosLength, 10.0f);
    r.Set(kDelaySosFeedback, 50.0f);
    const std::vector<float> out = ImpulseThrough(r, 2 * kL + 1);
    EXPECT_NEAR(std::fabs(out[kL]), kAmp, 0.02f * kAmp);
    EXPECT_NEAR(std::fabs(out[2 * kL]), 0.5f * kAmp, 0.02f * 0.5f * kAmp);
    EXPECT_LT(std::fabs(out[kL + kD]), 1e-4f);  // the main head does not echo the loop
}

TEST(Sos, IdleIsBitIdentical) {
    const std::vector<float> in = Noise(size_t(kFs), kAmp, 11);
    const auto render = [&](int mode, float sos) {
        DelayRig r;
        r.Set(kDelayTime, 100.0f);
        r.Set(kDelayFeedback, 50.0f);
        if (mode >= 0) {
            r.Set(kDelaySosMode, float(mode));
            r.Set(kDelaySos, sos);
        }
        return r.Render(in);
    };
    const std::vector<float> ref = render(-1, 0.0f);
    for (const auto& [mode, sos] : {std::pair<int, float>{1, 0.0f}, std::pair<int, float>{0, 1.0f}}) {
        const std::vector<float> y = render(mode, sos);
        EXPECT_EQ(std::memcmp(ref.data(), y.data(), ref.size() * sizeof(float)), 0) << "mode " << mode << " sos "
                                                                                        << sos;
    }
}
