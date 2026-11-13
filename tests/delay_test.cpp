// The delay through btdsp::Engine: timing, feedback decay and its bounded self-oscillation, click-free time changes,
// SW1 off and bypass spillover, and reverse (time-reversed repeats, click-free direction switches, SW1 off keeping the
// direction).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;

// Impulse A at 0, the parameter `id` set to `v` at 50 ms, impulse B at 150 ms; 100 ms delay, 50 % feedback.
std::vector<float> SpillRender(uint8_t id, float v) {
    std::vector<float> in(Samples(0.3), 0.0f), out(in.size());
    in[0] = kAmp;
    in[Samples(0.15)] = kAmp;
    DelayRig r;
    r.e.SetParam(btparam::kDelayTime, 100.0f);
    r.e.SetParam(btparam::kDelayFeedback, 50.0f);
    r.Feed(in, out, 0, Samples(0.05));
    r.e.SetParam(id, v);
    r.Feed(in, out, Samples(0.05), in.size());
    return out;
}

// n zeros with an asymmetric burst at s0: a decaying 300 Hz -> 2 kHz chirp of length len.
std::vector<float> Burst(size_t n, size_t s0, size_t len) {
    std::vector<float> x(n, 0.0f);
    const double k = 1700.0 / (2.0 * double(len));
    for (size_t i = 0; i < len; ++i) {
        const double t = double(i);
        x[s0 + i] = float(kAmp * std::min(1.0, t / (0.002 * kFs)) * std::exp(-3.0 * t / double(len)) *
                          std::sin(2.0 * M_PI * (300.0 * t + k * t * t) / kFs));
    }
    return x;
}

// Normalised correlation of a[0, n) (read backwards if reverseA) with b[0, n).
double Corr(const float* a, const float* b, size_t n, bool reverseA) {
    double ab = 0.0, aa = 0.0, bb = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double va = a[reverseA ? n - 1 - i : i], vb = b[i];
        ab += va * vb;
        aa += va * va;
        bb += vb * vb;
    }
    return ab / std::sqrt(aa * bb);
}

}  // namespace

TEST(Delay, ImpulseLandsAtSetTime) {
    for (float t : {10.0f, 12.34f, 350.0f, 1000.0f, 1999.9f, 2000.0f}) {
        SCOPED_TRACE(t);
        const float d = t * kFs / 1000.0f;
        std::vector<float> in(size_t(d) + 200, 0.0f), out(in.size());
        in[0] = kAmp;
        DelayRig r;
        r.e.SetParam(btparam::kDelayTime, t);
        r.Feed(in, out, 0, in.size());
        size_t peak = 0;
        for (size_t i = 0; i < out.size(); ++i)
            if (std::fabs(out[i]) > std::fabs(out[peak])) peak = i;
        EXPECT_LE(std::fabs(float(peak) - d), 1.0f);
    }
}

TEST(Delay, FeedbackDecaysGeometrically) {
    const size_t d = Samples(0.25);
    for (float fb : {35.0f, 70.0f, 100.0f}) {
        SCOPED_TRACE(fb);
        std::vector<float> in(7 * d, 0.0f), out(in.size());
        in[0] = kAmp;
        DelayRig r;
        r.e.SetParam(btparam::kDelayTime, 250.0f);
        r.e.SetParam(btparam::kDelayFeedback, fb);
        r.Feed(in, out, 0, in.size());
        EXPECT_NEAR(out[d], kAmp, 1e-5f);
        const float f = fb / 100.0f;
        for (int k = 2; k <= 6; ++k) EXPECT_NEAR(out[k * d] / out[d], std::pow(f, float(k - 1)), 1e-3f) << "k " << k;
    }
}

TEST(Delay, MaxFeedbackStaysBoundedAndRings) {
    const size_t sec = size_t(kFs);
    for (float fb : {100.0f, 110.0f}) {
        SCOPED_TRACE(fb);
        std::vector<float> in = Noise(sec, 1.0f, 3);
        in.resize(6 * sec, 0.0f);
        std::vector<float> out(in.size());
        DelayRig r;
        r.e.SetParam(btparam::kDelayTime, 300.0f);
        r.e.SetParam(btparam::kDelayFeedback, fb);
        r.Feed(in, out, 0, in.size());
        float peak = 0.0f;
        size_t bad = 0;
        for (float v : out) {
            bad += !std::isfinite(v);
            peak = std::max(peak, std::fabs(v));
        }
        EXPECT_EQ(bad, 0u);
        EXPECT_LE(peak, 1.3f);  // limiter ceiling 1.0 times the Hermite overshoot bound 1.25
        if (fb > 100.0f) {
            double acc = 0.0;
            for (size_t i = 5 * sec; i < 6 * sec; ++i) acc += double(out[i]) * out[i];
            EXPECT_GT(std::sqrt(acc / double(sec)), 0.01);
        }
    }
}

TEST(Delay, TimeSweepNoClick) {
    const size_t sec = size_t(kFs);
    const std::vector<float> in = Sine(3 * sec, 220.0f, kAmp);
    std::vector<float> ref[2];
    const float held[2] = {100.0f, 500.0f};
    for (int k = 0; k < 2; ++k) {
        DelayRig r;
        r.e.SetParam(btparam::kDelayTime, held[k]);
        ref[k].resize(in.size());
        r.Feed(in, ref[k], 0, in.size());
    }
    DelayRig r;
    r.e.SetParam(btparam::kDelayTime, 100.0f);
    std::vector<float> sweep(in.size());
    r.Feed(in, sweep, 0, sec);
    for (size_t i = sec; i < 2 * sec; i += 48) {
        r.e.SetParam(btparam::kDelayTime, 100.0f * std::pow(5.0f, float(i - sec) / float(sec)));
        r.Feed(in, sweep, i, i + 48);
    }
    r.e.SetParam(btparam::kDelayTime, 500.0f);
    r.Feed(in, sweep, 2 * sec, in.size());
    const size_t from = size_t(0.6f * kFs);  // past both references' onset kinks
    EXPECT_LE(Click(sweep, from), 2.0f * std::max(Click(ref[0], from), Click(ref[1], from)) + 1e-3f);
}

TEST(Delay, OffMutesInputAndSpillsTails) {
    const std::vector<float> y = SpillRender(btparam::kDelayMode, 1.0f);
    EXPECT_NEAR(y[Samples(0.1)], kAmp, 1e-4f);
    EXPECT_NEAR(y[Samples(0.2)], 0.5f * kAmp, 1e-4f);  // A's tail continues
    EXPECT_LT(std::fabs(y[Samples(0.25)]), 1e-4f);     // B was never recorded
}

TEST(Delay, BypassPassesDryAndSpillsTails) {
    const std::vector<float> y = SpillRender(btparam::kBypass, 1.0f);
    EXPECT_NEAR(y[Samples(0.15)], kAmp, 1e-4f);  // dry B at unity
    EXPECT_NEAR(y[Samples(0.2)], 0.5f * kAmp, 1e-4f);
    EXPECT_LT(std::fabs(y[Samples(0.25)]), 1e-4f);

    // Bit-exact dry: delay_mix 0, and bypass from the start (nothing recorded, so the wet is silent).
    const std::vector<float> in = Noise(size_t(kFs), 1.0f, 4);
    for (int bypass = 0; bypass < 2; ++bypass) {
        SCOPED_TRACE(bypass);
        DelayRig r;
        r.e.SetParam(btparam::kDelayFeedback, 50.0f);
        r.e.SetParam(btparam::kBypass, float(bypass));
        r.e.SetParam(btparam::kDelayMix, bypass ? 100.0f : 0.0f);
        std::vector<float> out(in.size());
        r.Feed(in, out, 0, in.size());
        EXPECT_EQ(std::memcmp(in.data(), out.data(), in.size() * sizeof(float)), 0);
    }
}

// Grains launch every D from sample 0, so the grain launched at 2D plays input sample s at 4D - 2 - s; the feedback
// loop records forward, so repeat 2 (feedback 0.5) plays reversed again, from the grain at 3D, at 5D - 2 - s.
TEST(Reverse, BurstComesBackReversedAndRepeatsStayReversed) {
    const size_t D = Samples(0.3), L = Samples(0.05), s0 = D + Samples(0.1);
    const std::vector<float> in = Burst(5 * D + Samples(0.1), s0, L);
    std::vector<float> out(in.size());
    DelayRig r;
    r.e.SetParam(btparam::kDelayMode, 0.0f);
    r.e.SetParam(btparam::kDelayTime, 300.0f);
    r.e.SetParam(btparam::kDelayFeedback, 50.0f);
    r.Feed(in, out, 0, in.size());
    const size_t n0 = 4 * D - 2 - (s0 + L - 1), n1 = n0 + D;
    EXPECT_GE(Corr(&out[n0], &in[s0], L, true), 0.9);
    EXPECT_LT(Corr(&out[n0], &in[s0], L, false), 0.5);  // reversed, not merely delayed
    EXPECT_GE(Corr(&out[n1], &in[s0], L, true), 0.9);
    EXPECT_NEAR(Rms(out, n1, n1 + L) / Rms(out, n0, n0 + L), 0.5, 0.05);
}

TEST(Reverse, DirectionSwitchNoClick) {
    const size_t sec = size_t(kFs);
    const std::vector<float> in = Sine(3 * sec, 220.0f, kAmp);
    std::vector<float> ref[2];
    const float held[2] = {2.0f, 0.0f};  // forward, reverse
    for (int k = 0; k < 2; ++k) {
        DelayRig r;
        r.e.SetParam(btparam::kDelayTime, 300.0f);
        r.e.SetParam(btparam::kDelayMode, held[k]);
        ref[k].resize(in.size());
        r.Feed(in, ref[k], 0, in.size());
    }
    DelayRig r;
    r.e.SetParam(btparam::kDelayTime, 300.0f);
    std::vector<float> sw(in.size());
    r.Feed(in, sw, 0, sec);
    r.e.SetParam(btparam::kDelayMode, 0.0f);
    r.Feed(in, sw, sec, 2 * sec);
    r.e.SetParam(btparam::kDelayMode, 2.0f);
    r.Feed(in, sw, 2 * sec, in.size());
    const size_t from = size_t(0.6f * kFs);  // past both references' onset kinks
    EXPECT_LE(Click(sw, from), 2.0f * std::max(Click(ref[0], from), Click(ref[1], from)) + 1e-3f);
}

// SW1 off after reverse keeps playing reversed: the burst, recorded before the switch, comes back time-reversed.
TEST(Reverse, OffKeepsDirection) {
    const size_t D = Samples(0.1), L = Samples(0.05), s0 = Samples(0.1);
    const std::vector<float> in = Burst(5 * D, s0, L);
    std::vector<float> out(in.size());
    DelayRig r;
    r.e.SetParam(btparam::kDelayMode, 0.0f);
    r.e.SetParam(btparam::kDelayTime, 100.0f);
    r.e.SetParam(btparam::kDelayFeedback, 50.0f);
    r.Feed(in, out, 0, 2 * D);
    r.e.SetParam(btparam::kDelayMode, 1.0f);
    r.Feed(in, out, 2 * D, in.size());
    EXPECT_GE(Corr(&out[4 * D - 2 - (s0 + L - 1)], &in[s0], L, true), 0.9);
}
