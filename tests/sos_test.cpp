// Sound on sound (dsp/delay.h) through btdsp::Engine: the 301 head plays once at delay_sos_length, the 501 head
// repeats at delay_sos_ratio x the delay time through the delay feedback, overdub keeps delay_sos_feedback of the loop
// per pass without the main head echoing it, and SOS off leaves the output bit-identical. Reverse (SW1 down): the 301
// head turns around where it plays, overdub plays its loop backwards, and the 301 window is capped by the line.
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

// The index of the largest |y| in [from, to].
size_t Peak(const std::vector<float>& y, size_t from, size_t to) {
    size_t p = from;
    for (size_t i = from; i <= to; ++i)
        if (std::fabs(y[i]) > std::fabs(y[p])) p = i;
    return p;
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

// Flipped to reverse at tf, the 301 head (10 s back) turns around where it plays: out[tf + u] = in[tf - kL - u], within
// 25 ms of the flip and still 4 s later (the first grain lasts half the length).
TEST(Sos, Reverse301TurnsAroundAtPlayhead) {
    DelayRig r(btdsp::kSosSamples);
    r.Set(kDelayTime, 100.0f);
    r.Set(kDelaySosMode, 1.0f);
    r.Set(kDelaySos, 1.0f);
    r.Set(kDelaySosLength, 10.0f);
    const size_t tf = Samples(15.0);
    std::vector<float> in = Noise(Samples(10.0), kAmp, 7);
    in.resize(tf + Samples(4.5), 0.0f);
    std::vector<float> out(in.size());
    r.Feed(in, out, 0, tf);
    r.Set(kDelayMode, 0.0f);
    r.Feed(in, out, tf, in.size());
    const size_t n = Samples(0.5);
    for (size_t u0 : {Samples(0.025), Samples(4.0)}) {
        SCOPED_TRACE(u0);
        EXPECT_GE(Corr(&out[tf + u0], &in[tf - kL - u0 - n + 1], n, true), 0.9);
        EXPECT_LT(Corr(&out[tf + u0], &in[tf - kL - u0 - n + 1], n, false), 0.5);
    }
}

// Overdub flipped to reverse at tf plays the loop backwards from the newest pass: the copies of the impulses at 0 and
// 1 s written one pass later (at kL and kL + 1 s, at the 50 % keep) play at 2 tf - 2 - their write time, the later one
// first.
TEST(Sos, ReverseOverdubPlaysLoopBackwards) {
    DelayRig r(btdsp::kSosSamples);
    r.Set(kDelaySosMode, 3.0f);
    r.Set(kDelaySos, 1.0f);
    r.Set(kDelaySosLength, 10.0f);
    r.Set(kDelaySosFeedback, 50.0f);
    std::vector<float> in(Samples(16.5), 0.0f), out(in.size());
    in[0] = in[Samples(1.0)] = kAmp;
    const size_t tf = Samples(13.0);
    r.Feed(in, out, 0, tf);
    r.Set(kDelayMode, 0.0f);
    r.Feed(in, out, tf, in.size());
    const size_t e1 = 2 * tf - 2 - (kL + Samples(1.0)), e0 = 2 * tf - 2 - kL;
    for (size_t e : {e1, e0}) {
        SCOPED_TRACE(e);
        EXPECT_EQ(Peak(out, e - 200, e + 200), e);
        EXPECT_NEAR(std::fabs(out[e]), 0.5f * kAmp, 0.02f * 0.5f * kAmp);
    }
}

// On the test rigs' short line the 301 length clamps to 32 765 samples while the reverse window is capped at
// SosRevMaxHop(32 768) = 14 333: reversed from the start, the turnaround grain starts at 32 765 for 1 sample (plus its
// 1 920-sample fade), grains launch at 1 920 and 16 253, and an impulse at 9 600 plays at 2 x 16 253 - 9 600 - 2.
TEST(Sos, Reverse301WindowCappedByLine) {
    static_assert(btdsp::SosRevMaxHop(kTestSosSamples) == 14333);
    std::vector<float> in(Samples(0.34), 0.0f);
    in[Samples(0.1)] = kAmp;
    const auto render = [&](float sos) {
        DelayRig r;
        r.Set(kDelayMode, 0.0f);
        r.Set(kDelaySosMode, 1.0f);
        r.Set(kDelaySos, sos);
        return r.Render(in);
    };
    std::vector<float> d = render(1.0f);
    const std::vector<float> off = render(0.0f);
    for (size_t i = 0; i < d.size(); ++i) d[i] -= off[i];
    constexpr size_t e = 2 * 16253 - 9600 - 2;
    EXPECT_EQ(Peak(d, 0, d.size() - 1), e);
    EXPECT_NEAR(std::fabs(d[e]), kAmp, 0.02f * kAmp);
}
