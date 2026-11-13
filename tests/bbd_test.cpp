// The BBD type (SW2 down) through btdsp::Engine: at color 0 it already band-limits the repeats while a guitar-range
// note keeps its level; color lowers the clock until the repeats alias; the noise rises with color and is exactly 0
// once the repeats are gone; full color with feedback stays bounded; a time change glides like a BBD clock; the clock
// LFO bends the pitch along a triangle, keeps the read speed bounded at its extremes and leaves tape and digital alone;
// delay_bbd_scope all puts the BBD on the dry while the loop stays clean.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;

// A DelayRig of the BBD type with knob 6 at `color`, the clock LFO off unless a test turns it on.
struct BbdRig : DelayRig {
    explicit BbdRig(float color) {
        Set(btparam::kDelayType, 0.0f);
        Set(btparam::kDelayColor, color);
        Set(btparam::kDelayBbdModDepth, 0.0f);
    }
};

}  // namespace

TEST(Bbd, ColorZeroBandLimitsAndKeepsLevel) {
    const size_t from = Samples(0.4), to = Samples(1.0);
    const auto wet = [&](EngineRig&& r, float hz) {
        r.Set(btparam::kDelayTime, 100.0f);
        return Rms(r.Render(Sine(to, hz, kAmp)), from, to);
    };
    const double low = 20.0 * std::log10(wet(BbdRig(0.0f), 300.0f) / wet(DelayRig(), 300.0f));
    EXPECT_NEAR(low, 0.0, 1.5);  // the compander tracks a note at guitar level
    EXPECT_LE(wet(BbdRig(0.0f), 8000.0f), 0.05 * wet(DelayRig(), 8000.0f));  // 26 dB down at 8 kHz
}

// 600 ms: color 0 clocks at 16384 / (2 x 0.6 s) = 13.65 kHz, fully clockwise at a quarter of that, 3.41 kHz, where a
// 2.5 kHz note folds down to 913 Hz.
TEST(Bbd, ColorLowersTheClockUntilItAliases) {
    const size_t from = Samples(1.0), to = Samples(2.0);
    const auto alias = [&](float color) {
        BbdRig r(color);
        r.Set(btparam::kDelayTime, 600.0f);
        const std::vector<float> y = r.Render(Sine(to, 2500.0f, kAmp));
        return Amplitude(y, from, to, 3413.33 - 2500.0) / Amplitude(y, from, to, 2500.0);
    };
    EXPECT_LE(alias(0.0f), 1e-3);
    EXPECT_GE(alias(100.0f), 0.1);
}

TEST(Bbd, NoiseRisesWithColorAndGatesToSilence) {
    const size_t sec = size_t(kFs);
    std::vector<float> in = Sine(sec, 220.0f, kAmp);
    in.resize(4 * sec, 0.0f);
    const auto render = [&](float color, float noiseDb) {
        BbdRig r(color);
        r.Set(btparam::kDelayTime, 100.0f);
        r.Set(btparam::kDelayBbdNoise, noiseDb);
        return r.Render(in);
    };
    const auto noise = [&](float color) {
        const std::vector<float> loud = render(color, -40.0f), quiet = render(color, -90.0f);
        std::vector<float> d(in.size());
        for (size_t i = 0; i < in.size(); ++i) d[i] = loud[i] - quiet[i];
        return Rms(d, Samples(0.3), sec);
    };
    EXPECT_GE(noise(100.0f), 5.0 * noise(0.0f));
    const std::vector<float> loud = render(100.0f, -40.0f);
    size_t nonzero = 0;
    for (size_t i = 3 * sec; i < 4 * sec; ++i) nonzero += loud[i] != 0.0f;
    EXPECT_EQ(nonzero, 0u);
}

TEST(Bbd, FullColorFeedbackStaysBounded) {
    const size_t sec = size_t(kFs);
    for (float fb : {100.0f, 110.0f}) {
        SCOPED_TRACE(fb);
        std::vector<float> in = Noise(sec, 1.0f, 3);
        in.resize(6 * sec, 0.0f);
        BbdRig r(100.0f);
        r.Set(btparam::kDelayTime, 300.0f);
        r.Set(btparam::kDelayFeedback, fb);
        const std::vector<float> out = r.Render(in);
        float peak = 0.0f;
        size_t bad = 0;
        for (float v : out) {
            bad += !std::isfinite(v);
            peak = std::max(peak, std::fabs(v));
        }
        EXPECT_EQ(bad, 0u);
        EXPECT_LE(peak, 1.3f);  // limiter ceiling 1.0 times the Hermite overshoot bound 1.25
        if (fb > 100.0f) EXPECT_GT(Rms(out, 5 * sec, 6 * sec), 0.01);
        else EXPECT_LE(Rms(out, 5 * sec, 6 * sec), Rms(out, 0, sec));
    }
}

// 100 -> 300 ms: the head glides out, so the 220 Hz repeats play flat (below 165 Hz) while it moves.
TEST(Bbd, TimeChangeGlides) {
    const size_t sec = size_t(kFs), at = sec;
    const std::vector<float> in = Sine(3 * sec, 220.0f, kAmp);
    BbdRig r(0.0f);
    r.Set(btparam::kDelayTime, 100.0f);
    std::vector<float> out(in.size());
    r.Feed(in, out, 0, at);
    r.Set(btparam::kDelayTime, 300.0f);
    r.Feed(in, out, at, in.size());
    const double f = Frequency(out, at + Samples(0.05), at + Samples(0.15));
    ASSERT_GT(f, 0.0);
    EXPECT_LT(f, 165.0);
}

// 500 ms at depth 2 %: the read speed is 1 -+ 4 rate/fs x d x 0.02 / (1 + 0.02 tri)^2, so 220 Hz plays at
// 220 (1 +- 0.04 rate) within 4 %, constant over each half of the triangle; twice the rate, twice the bend.
TEST(Bbd, ClockLfoBendsThePitchAlongATriangle) {
    const size_t sec = size_t(kFs), win = Samples(0.1), hop = Samples(0.05);
    const std::vector<float> in = Sine(4 * sec, 220.0f, kAmp);
    for (float rate : {1.0f, 2.0f}) {
        SCOPED_TRACE(rate);
        BbdRig r(0.0f);
        r.Set(btparam::kDelayTime, 500.0f);
        r.Set(btparam::kDelayBbdModRate, rate);
        r.Set(btparam::kDelayBbdModDepth, 2.0f);
        const std::vector<float> out = r.Render(in);
        double lo = 1e9, hi = 0.0;
        for (size_t i = sec; i + win <= out.size(); i += hop) {
            const double f = Frequency(out, i, i + win);
            lo = std::min(lo, f);
            hi = std::max(hi, f);
        }
        const double dev = 0.04 * rate * 220.0;
        EXPECT_NEAR(hi, 220.0 + dev, 0.1 * dev);
        EXPECT_NEAR(lo, 220.0 - dev, 0.1 * dev);
    }
}

// Depth 25 % at 10 Hz on 2 s would run the head backwards; the slew limit holds the read speed to
// 1 +- kBbdModMaxRate, and reaches it.
TEST(Bbd, ExtremeClockLfoKeepsTheReadSpeedBounded) {
    const size_t sec = size_t(kFs);
    const std::vector<float> in = Sine(5 * sec, 220.0f, kAmp);
    BbdRig r(0.0f);
    r.Set(btparam::kDelayTime, 2000.0f);
    r.Set(btparam::kDelayBbdModRate, 10.0f);
    r.Set(btparam::kDelayBbdModDepth, 25.0f);
    const std::vector<float> out = r.Render(in);
    // Rising crossings with +-0.05 hysteresis after the line has filled; every period between two of them.
    double shortest = 1e9, longest = 0.0;
    bool low = false;
    size_t prev = 0;
    for (size_t i = Samples(2.5); i < out.size(); ++i) {
        ASSERT_TRUE(std::isfinite(out[i]));
        if (out[i] < -0.05f) {
            low = true;
        } else if (low && out[i] > 0.05f) {
            low = false;
            if (prev) {
                shortest = std::min(shortest, double(i - prev));
                longest = std::max(longest, double(i - prev));
            }
            prev = i;
        }
    }
    const double fast = kFs / (220.0 * (1.0 + btdsp::kBbdModMaxRate));
    const double slow = kFs / (220.0 * (1.0 - btdsp::kBbdModMaxRate));
    EXPECT_GE(shortest, fast - 2.0);
    EXPECT_LE(longest, slow + 2.0);
    EXPECT_LE(shortest, kFs / (220.0 * 1.2));  // the limit is reached
    EXPECT_GE(longest, kFs / (220.0 * 0.8));
}

// The LFO belongs to the BBD: the digital and tape types ignore it bit-exactly.
TEST(Bbd, ClockLfoOnlyOnBbd) {
    const std::vector<float> in = Sine(size_t(kFs), 220.0f, kAmp);
    for (float type : {1.0f, 2.0f}) {
        SCOPED_TRACE(type);
        const auto render = [&](float depth) {
            DelayRig r;
            r.Set(btparam::kDelayType, type);
            r.Set(btparam::kDelayColor, 100.0f);
            r.Set(btparam::kDelayTime, 100.0f);
            r.Set(btparam::kDelayBbdModRate, 10.0f);
            r.Set(btparam::kDelayBbdModDepth, depth);
            return r.Render(in);
        };
        const std::vector<float> a = render(0.0f), b = render(25.0f);
        EXPECT_EQ(std::memcmp(a.data(), b.data(), a.size() * sizeof(float)), 0);
    }
}

// delay_bbd_scope all puts the BBD on the dry (the input, before the delay mix): a guitar-range note keeps its level
// and 8 kHz is band-limited, as on the repeats (ColorZeroBandLimitsAndKeepsLevel). Repeats leave the dry bit-exact.
TEST(Bbd, AllColorsTheDry) {
    const size_t from = Samples(0.4), to = Samples(1.0);
    const auto dry = [&](float scope, float hz) {
        BbdRig r(0.0f);
        r.Set(btparam::kDelayBbdScope, scope);
        r.Set(btparam::kDelayMix, 0.0f);
        r.Set(btparam::kDelayTime, 100.0f);
        return r.Render(Sine(to, hz, kAmp));
    };
    EXPECT_EQ(MaxAbsDiff(dry(0.0f, 8000.0f), Sine(to, 8000.0f, kAmp), 0, to), 0.0f);
    const double low = 20.0 * std::log10(Rms(dry(1.0f, 300.0f), from, to) / Rms(dry(0.0f, 300.0f), from, to));
    EXPECT_NEAR(low, 0.0, 1.5);
    EXPECT_LE(Rms(dry(1.0f, 8000.0f), from, to), 0.05 * Rms(dry(0.0f, 8000.0f), from, to));
}

// With all, the loop is clean: the second repeat is the first at the feedback gain. With repeats, each pass through
// the loop goes through the BBD again.
TEST(Bbd, AllKeepsTheLoopClean) {
    const auto err = [](float scope) {
        BbdRig r(0.0f);
        r.Set(btparam::kDelayBbdScope, scope);
        return SecondRepeatError(r, 1000.0f);
    };
    EXPECT_LE(err(1.0f), 0.01f);
    EXPECT_GE(err(0.0f), 0.01f);
}
