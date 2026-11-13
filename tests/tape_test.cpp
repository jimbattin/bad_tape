// The tape type (SW2 middle) through btdsp::Engine: age 0 is clean, wow and flutter stay within their depths, a time
// change glides at any age, maximum age with feedback stays bounded, the hiss level and gate, dropouts only at high
// age, and delay_tape_scope all coloring and wobbling the dry while the loop stays clean.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;

// A DelayRig of the tape type with knob 6 at `color` (100 = tape age 1).
struct TapeRig : DelayRig {
    explicit TapeRig(float color) {
        Set(btparam::kDelayType, 1.0f);
        Set(btparam::kDelayColor, color);
    }
};

}  // namespace

TEST(Tape, AgeZeroIsCleanFullAgeColors) {
    const std::vector<float> in = Sine(size_t(kFs), 220.0f, kAmp);
    const auto render = [&](float type, float color) {
        DelayRig r;
        r.e.SetParam(btparam::kDelayType, type);
        r.e.SetParam(btparam::kDelayColor, color);
        r.e.SetParam(btparam::kDelayTime, 100.0f);
        return r.Render(in);
    };
    const std::vector<float> digital = render(2.0f, 0.0f);
    const std::vector<float> fresh = render(1.0f, 0.0f);
    EXPECT_EQ(std::memcmp(digital.data(), fresh.data(), digital.size() * sizeof(float)), 0);
    const std::vector<float> worn = render(1.0f, 100.0f);
    EXPECT_NE(std::memcmp(digital.data(), worn.data(), digital.size() * sizeof(float)), 0);
}

TEST(Tape, ModulationWithinBounds) {
    const size_t period = Samples(0.1), n = 10 * size_t(kFs), d = Samples(0.05);  // 50 ms
    const size_t w = Samples(0.00833);  // the peak search's half window
    std::vector<float> in(n, 0.0f);
    for (size_t i = 0; i < n; i += period) in[i] = kAmp;
    const auto render = [&](bool wobble) {
        TapeRig r(100.0f);
        r.e.SetParam(btparam::kDelayTime, 50.0f);
        r.e.SetParam(btparam::kDelayTapeDropouts, 0.0f);
        if (!wobble) {
            r.e.SetParam(btparam::kDelayTapeWow, 0.0f);
            r.e.SetParam(btparam::kDelayTapeFlutter, 0.0f);
        }
        std::vector<float> out(n);
        r.Feed(in, out, 0, n);
        return out;
    };
    const std::vector<float> mod = render(true), ref = render(false);
    const auto peak = [w](const std::vector<float>& y, size_t c) {
        size_t p = c - w;
        for (size_t i = c - w; i <= c + w; ++i)
            if (std::fabs(y[i]) > std::fabs(y[p])) p = i;
        return double(p);
    };
    double maxDev = 0.0;
    for (size_t c = d; c + w < n; c += period) maxDev = std::max(maxDev, std::fabs(peak(mod, c) - peak(ref, c)));
    EXPECT_LE(maxDev, 0.002 * kFs + 0.0001 * kFs + 2.0);  // 2 ms wow + 0.1 ms flutter, +2
    EXPECT_GE(maxDev, 0.001 * kFs);
}

// Tape glides the pitch on a time change at any age (the transport, not the color); digital crossfades at pitch.
TEST(Tape, TimeChangeGlides) {
    const size_t sec = size_t(kFs), at = sec;
    const std::vector<float> in = Sine(3 * sec, 220.0f, kAmp);
    const auto render = [&](float type, float color) {
        DelayRig r;
        r.e.SetParam(btparam::kDelayType, type);
        r.e.SetParam(btparam::kDelayColor, color);
        r.e.SetParam(btparam::kDelayTapeWow, 0.0f);
        r.e.SetParam(btparam::kDelayTapeFlutter, 0.0f);
        r.e.SetParam(btparam::kDelayTapeDropouts, 0.0f);
        r.e.SetParam(btparam::kDelayTapeHiss, -90.0f);
        r.e.SetParam(btparam::kDelayTime, 100.0f);
        std::vector<float> out(in.size());
        r.Feed(in, out, 0, at);
        r.e.SetParam(btparam::kDelayTime, 300.0f);
        r.Feed(in, out, at, in.size());
        return out;
    };
    EXPECT_LT(Frequency(render(1.0f, 100.0f), at + Samples(0.05), at + Samples(0.15)), 165.0);
    EXPECT_LT(Frequency(render(1.0f, 0.0f), at + Samples(0.05), at + Samples(0.15)), 165.0);
    EXPECT_NEAR(Frequency(render(2.0f, 0.0f), at + Samples(0.05), at + Samples(0.15)), 220.0, 0.03 * 220.0);

    const size_t change = Samples(0.1), imp = change + Samples(1.5);  // 1.5 s after the change
    std::vector<float> x(imp + Samples(0.42), 0.0f), y(x.size());
    x[imp] = kAmp;
    TapeRig r(100.0f);
    r.e.SetParam(btparam::kDelayTapeWow, 0.0f);
    r.e.SetParam(btparam::kDelayTapeFlutter, 0.0f);
    r.e.SetParam(btparam::kDelayTapeDropouts, 0.0f);
    r.e.SetParam(btparam::kDelayTapeHiss, -90.0f);
    r.e.SetParam(btparam::kDelayTime, 100.0f);
    r.Feed(x, y, 0, change);
    r.e.SetParam(btparam::kDelayTime, 300.0f);
    r.Feed(x, y, change, x.size());
    size_t p = imp;
    for (size_t i = imp; i < y.size(); ++i)
        if (std::fabs(y[i]) > std::fabs(y[p])) p = i;
    // 0.125 ms: the lowpass adds about 0.08 ms of group delay.
    EXPECT_NEAR(double(p), double(imp + Samples(0.3)), 0.000125 * kFs);
}

// A long time change lands on the new time exactly: the head's glide does not stall short of a far target.
TEST(Tape, LongTimeChangeLandsOnTarget) {
    const size_t change = Samples(0.1), imp = change + 6 * size_t(kFs);  // the glide covers 1.9 s in about 3.8 s
    std::vector<float> x(imp + Samples(2.0) + Samples(0.042), 0.0f), y(x.size());
    x[imp] = kAmp;
    TapeRig r(100.0f);
    r.e.SetParam(btparam::kDelayTapeWow, 0.0f);
    r.e.SetParam(btparam::kDelayTapeFlutter, 0.0f);
    r.e.SetParam(btparam::kDelayTapeDropouts, 0.0f);
    r.e.SetParam(btparam::kDelayTapeHiss, -90.0f);
    r.e.SetParam(btparam::kDelayTime, 100.0f);
    r.Feed(x, y, 0, change);
    r.e.SetParam(btparam::kDelayTime, 2000.0f);
    r.Feed(x, y, change, x.size());
    size_t p = imp;
    for (size_t i = imp; i < y.size(); ++i)
        if (std::fabs(y[i]) > std::fabs(y[p])) p = i;
    EXPECT_NEAR(double(p), double(imp + Samples(2.0)), 0.000125 * kFs);
}

TEST(Tape, MaxAgeFeedbackStaysBounded) {
    const size_t sec = size_t(kFs);
    for (float fb : {100.0f, 110.0f}) {
        SCOPED_TRACE(fb);
        std::vector<float> in = Noise(sec, 1.0f, 3);
        in.resize(6 * sec, 0.0f);
        std::vector<float> out(in.size());
        TapeRig r(100.0f);
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
        EXPECT_LE(peak, 2.0f);  // limiter 1.0 x Hermite overshoot 1.25 x the 3 dB bump
        if (fb > 100.0f) EXPECT_GT(Rms(out, 5 * sec, 6 * sec), 0.01);
        else EXPECT_LE(Rms(out, 5 * sec, 6 * sec), Rms(out, 0, sec));
    }
}

TEST(Tape, HissLevelAndGate) {
    const size_t sec = size_t(kFs);
    std::vector<float> in = Sine(sec, 220.0f, kAmp);
    in.resize(4 * sec, 0.0f);
    const auto render = [&](float hissDb) {
        TapeRig r(100.0f);
        r.e.SetParam(btparam::kDelayTime, 100.0f);
        r.e.SetParam(btparam::kDelayTapeDropouts, 0.0f);
        r.e.SetParam(btparam::kDelayTapeHiss, hissDb);
        return r.Render(in);
    };
    const std::vector<float> loud = render(-40.0f), quiet = render(-90.0f);
    std::vector<float> diff(in.size());
    for (size_t i = 0; i < in.size(); ++i) diff[i] = loud[i] - quiet[i];
    const double hiss = Rms(diff, Samples(0.2), sec);
    EXPECT_GE(hiss, 0.0089);  // -40 dBFS +-1 dB, the gate fully open
    EXPECT_LE(hiss, 0.0112);
    size_t nonzero = 0;
    for (size_t i = 3 * sec; i < 4 * sec; ++i) nonzero += loud[i] != 0.0f;
    EXPECT_EQ(nonzero, 0u);
}

TEST(Tape, DropoutsOnlyAtHighAge) {
    const std::vector<float> in = Sine(20 * size_t(kFs), 220.0f, kAmp);
    const auto events = [&](float color) {
        TapeRig r(color);
        r.e.SetParam(btparam::kDelayTime, 100.0f);
        r.e.SetParam(btparam::kDelayTapeDropouts, 30.0f);
        r.e.SetParam(btparam::kDelayTapeWow, 0.0f);
        r.e.SetParam(btparam::kDelayTapeFlutter, 0.0f);
        r.e.SetParam(btparam::kDelayTapeHiss, -90.0f);
        std::vector<float> out(in.size());
        r.Feed(in, out, 0, in.size());
        std::vector<double> w;
        for (size_t i = size_t(0.2f * kFs); i + 480 <= out.size(); i += 480) w.push_back(Rms(out, i, i + 480));
        std::vector<double> sorted = w;
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        const double thr = 0.7 * sorted[sorted.size() / 2];
        int n = 0;
        for (size_t k = 1; k < w.size(); ++k) n += w[k] < thr && w[k - 1] >= thr;
        return n;
    };
    const int worn = events(100.0f);
    EXPECT_GE(worn, 3);
    EXPECT_LE(worn, 25);
    EXPECT_EQ(events(44.0f), 0);
}

// delay_tape_scope all colors the dry (the input, before the delay mix): a 5 kHz note through the HF loss (2.5 kHz at
// age 1, 12 dB down an octave above) and the saturation. Repeats leave the dry bit-exact.
TEST(Tape, AllColorsTheDry) {
    const std::vector<float> in = Sine(Samples(1.0), 5000.0f, kAmp);
    const size_t from = Samples(0.5), to = Samples(1.0);
    const auto render = [&](float scope) {
        TapeRig r(100.0f);
        r.Set(btparam::kDelayTapeScope, scope);
        r.Set(btparam::kDelayMix, 0.0f);
        r.Set(btparam::kDelayTapeWow, 0.0f);
        r.Set(btparam::kDelayTapeFlutter, 0.0f);
        r.Set(btparam::kDelayTapeDropouts, 0.0f);
        return r.Render(in);
    };
    EXPECT_EQ(MaxAbsDiff(render(0.0f), in, 0, to), 0.0f);
    const double a = Amplitude(render(1.0f), from, to, 5000.0);
    EXPECT_GE(a, 0.1 * kAmp);
    EXPECT_LE(a, 0.5 * kAmp);
}

// With all, wow wobbles the dry too.
TEST(Tape, AllWobblesTheDry) {
    const std::vector<float> in = Sine(Samples(1.0), 1000.0f, kAmp);
    const auto render = [&](float wow) {
        TapeRig r(100.0f);
        r.Set(btparam::kDelayTapeScope, 1.0f);
        r.Set(btparam::kDelayMix, 0.0f);
        r.Set(btparam::kDelayTapeWow, wow);
        r.Set(btparam::kDelayTapeFlutter, 0.0f);
        r.Set(btparam::kDelayTapeDropouts, 0.0f);
        r.Set(btparam::kDelayTapeHiss, -90.0f);
        return r.Render(in);
    };
    EXPECT_GE(MaxAbsDiff(render(5.0f), render(0.0f), Samples(0.5), Samples(1.0)), 0.05f);
}

// With all, the loop is clean: the second repeat is the first at the feedback gain. With repeats, each pass through
// the loop colors it again, so the second repeat is not the first scaled.
TEST(Tape, AllKeepsTheLoopClean) {
    const auto err = [](float scope) {
        TapeRig r(100.0f);
        r.Set(btparam::kDelayTapeScope, scope);
        r.Set(btparam::kDelayTapeWow, 0.0f);
        r.Set(btparam::kDelayTapeFlutter, 0.0f);
        r.Set(btparam::kDelayTapeDropouts, 0.0f);
        r.Set(btparam::kDelayTapeHiss, -90.0f);
        return SecondRepeatError(r, 1000.0f);
    };
    EXPECT_LE(err(1.0f), 0.01f);
    EXPECT_GE(err(0.0f), 0.01f);
}
