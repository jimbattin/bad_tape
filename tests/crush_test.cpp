// The digital type's knob 6 color (SW2 up, dsp/crush.h) through btdsp::Engine and on its own: the bitcrush staircase
// with the open option off and on, open letting the hold alias, clock following the delay time, bit inversion without
// DC falling silent with the repeats, wrap folding an overload to the other rail, the bit depth's hysteresis, and
// delay_crush_scope all crushing the dry and the reverb's input while the loop stays clean.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;

// The digital type at knob 6 = color with the bitcrush pinned, not at its factory defaults: the crush in the loop
// (scope repeats), 4 kHz hold rate, the anti-alias filter at 0.7 x the hold rate (2.8 kHz), wrap and bit inversion
// off; open and clock at their defaults.
struct CrushRig : DelayRig {
    explicit CrushRig(float color) {
        Set(btparam::kDelayColor, color);
        Set(btparam::kDelayCrushScope, 0.0f);
        Set(btparam::kDelayCrushRate, 4000.0f);
        Set(btparam::kDelayCrushAa, 0.7f);
        Set(btparam::kDelayCrushWrap, 0.0f);
        Set(btparam::kDelayCrushBits, 0.0f);
    }
};

}  // namespace

// Knob 6 fully clockwise on the digital type holds at 4 kHz with 4 bits (CrushRig). With the open option off, the
// anti-imaging filter smooths the hold's staircase; open leaves it bare. Without the filter the steps put the RMS
// second difference at about 0.02; the smoothed signal stays far below.
TEST(Crush, ColorCrushes) {
    const std::vector<float> in = Sine(size_t(kFs), 220.0f, kAmp);
    const auto render = [&](float color, float open) {
        CrushRig r(color);
        r.Set(btparam::kDelayTime, 100.0f);
        r.Set(btparam::kDelayCrushOpen, open);
        return r.Render(in);
    };
    const auto fizz = [](const std::vector<float>& y) {
        double acc = 0.0;
        const size_t from = Samples(0.2);
        for (size_t i = from; i < y.size(); ++i) {
            const double d2 = double(y[i]) - 2.0 * y[i - 1] + y[i - 2];
            acc += d2 * d2;
        }
        return std::sqrt(acc / double(y.size() - from));
    };
    const std::vector<float> clean = render(0.0f, 1.0f);
    const std::vector<float> smoothed = render(100.0f, 0.0f);
    EXPECT_NE(std::memcmp(clean.data(), smoothed.data(), clean.size() * sizeof(float)), 0);
    EXPECT_LT(fizz(smoothed), 0.005);
    EXPECT_GT(fizz(render(100.0f, 1.0f)), 0.01);
}

// Open: a 9 kHz note held at 4 kHz folds down to 1 kHz. The anti-alias filter (2-pole at 2.8 kHz) takes about 16 dB
// off it; opened, it folds down at almost the note's level.
TEST(Crush, OpenLetsTheHoldAlias) {
    const size_t from = Samples(0.5), to = Samples(1.0);
    const auto alias = [&](float open) {
        CrushRig r(100.0f);
        r.Set(btparam::kDelayTime, 100.0f);
        r.Set(btparam::kDelayCrushOpen, open);
        std::vector<float> out(to);
        r.Feed(Sine(to, 9000.0f, kAmp), out, 0, to);
        return Amplitude(out, from, to, 1000.0);
    };
    const double closed = alias(0.0f), opened = alias(1.0f);
    EXPECT_GE(opened, 0.5 * kAmp);
    EXPECT_GE(opened, 4.0 * closed);
}

// Clock: the hold rate is delay_crush_rate (4 kHz here) at a 1 s delay and halves at 2 s, where a 2.5 kHz note folds
// down to 500 Hz instead of 1.5 kHz. Open is off, so the anti-imaging filter keeps the hold's images out of the way.
TEST(Crush, ClockFollowsTheDelayTime) {
    const size_t from = Samples(2.5), to = Samples(3.0);
    const auto render = [&](float clock) {
        CrushRig r(100.0f);
        r.Set(btparam::kDelayTime, 2000.0f);
        r.Set(btparam::kDelayCrushClock, clock);
        r.Set(btparam::kDelayCrushOpen, 0.0f);
        std::vector<float> out(to);
        r.Feed(Sine(to, 2500.0f, kAmp), out, 0, to);
        return out;
    };
    const std::vector<float> fixed = render(0.0f), clocked = render(1.0f);
    EXPECT_GE(Amplitude(fixed, from, to, 1500.0), 5.0 * Amplitude(fixed, from, to, 500.0));
    EXPECT_GE(Amplitude(clocked, from, to, 500.0), 5.0 * Amplitude(clocked, from, to, 1500.0));
}

// Bit inversion (bits) changes the repeats without shifting their DC and falls silent with them: once the repeats are
// gone the output is exactly 0.
TEST(Crush, BitInversionFallsSilentWithTheRepeats) {
    std::vector<float> in = Sine(Samples(0.5), 220.0f, 2.0f * kAmp);
    in.resize(Samples(4.0), 0.0f);
    const auto render = [&](float bits) {
        CrushRig r(100.0f);
        r.Set(btparam::kDelayTime, 100.0f);
        r.Set(btparam::kDelayFeedback, 50.0f);
        r.Set(btparam::kDelayCrushBits, bits);
        return r.Render(in);
    };
    const std::vector<float> plain = render(0.0f), inverted = render(1.0f);
    EXPECT_GE(MaxAbsDiff(plain, inverted, 0, in.size()), 0.03f);
    const size_t active = Samples(1.5);  // the note and its repeats
    double dc = 0.0;
    for (size_t i = 0; i < active; ++i) dc += double(inverted[i]) - plain[i];
    EXPECT_LE(std::fabs(dc / double(active)), 1e-3);
    for (size_t i = Samples(3.5); i < in.size(); ++i) ASSERT_EQ(inverted[i], 0.0f) << i;
}

// Scope all: the crush sits on the chain's input, so the dry and every reverb input are crushed. A 9 kHz note held at
// 4 kHz with open on (the factory value) folds down to 1 kHz; scope repeats leaves the dry exactly the input. With
// knob 1 fully counter-clockwise the reverb's default (mix) input is the crushed dry, and knob 6 sets how hard: at
// color 0 the input crush is off and a linear reverb adds no 1 kHz. On the dry tap the reverb gets crushed sound too.
TEST(Crush, AllCrushesTheDryAndTheReverbInput) {
    const size_t from = Samples(0.5), to = Samples(1.0);
    const std::vector<float> in = Sine(to, 9000.0f, kAmp);
    const auto render = [&](float scope, float color, float delayMix, float reverbMix, float routing) {
        CrushRig r(color);
        r.Set(btparam::kDelayCrushScope, scope);
        r.Set(btparam::kDelayMix, delayMix);
        r.Set(btparam::kReverbMix, reverbMix);
        r.Set(btparam::kRouting, routing);
        r.Set(btparam::kDelayTime, 100.0f);
        std::vector<float> out(to);
        r.Feed(in, out, 0, to);
        return out;
    };
    const auto alias = [&](const std::vector<float>& y) { return Amplitude(y, from, to, 1000.0); };
    EXPECT_EQ(MaxAbsDiff(render(0.0f, 100.0f, 0.0f, 0.0f, 1.0f), in, 0, to), 0.0f);
    EXPECT_GE(alias(render(1.0f, 100.0f, 0.0f, 0.0f, 1.0f)), 0.5 * kAmp);
    EXPECT_GE(alias(render(1.0f, 100.0f, 0.0f, 100.0f, 1.0f)), 10.0 * alias(render(1.0f, 0.0f, 0.0f, 100.0f, 1.0f)));
    EXPECT_GE(alias(render(1.0f, 100.0f, 0.0f, 100.0f, 2.0f)), 10.0 * alias(render(0.0f, 100.0f, 0.0f, 100.0f, 2.0f)));
}

// Scope all keeps the feedback loop clean: each repeat is crushed once, on the way in, so the second repeat is half
// the first (50 % feedback) up to the loop's DC blocker (about 0.008 here). Scope repeats re-quantises the second
// repeat at 4 bits (about 0.038). The bound sits halfway.
TEST(Crush, AllKeepsTheLoopClean) {
    const auto err = [](float scope) {
        CrushRig r(100.0f);
        r.Set(btparam::kDelayCrushScope, scope);
        return SecondRepeatError(r, 220.0f);
    };
    const float all = err(1.0f), repeats = err(0.0f);
    EXPECT_LE(all, 0.023f);
    EXPECT_GE(repeats, 0.023f);
}

// Wrap: a signal driven past full scale wraps to the opposite rail instead of clipping. DC 0.5 at 24 dB drive is 7.9:
// clipped it holds near +1 / drive, wrapped at 7.9 - 8 = -0.1, one negative 4-bit step.
TEST(Crush, WrapFoldsOverloadToTheOtherRail) {
    const auto settle = [](float wrap) {
        btdsp::Crush c;
        c.Init(kFs);
        btdsp::CrushSettings s;
        s.amount = 1.0f;
        s.driveDb = 24.0f;
        s.wrap = wrap;
        c.Set(s);
        float y = 0.0f;
        for (size_t i = 0; i < Samples(0.1); ++i) y = c.Process(0.5f);
        return y;
    };
    EXPECT_GT(settle(0.0f), 0.05f);
    EXPECT_LT(settle(1.0f), 0.0f);
}

// Knob 6 steps the bit depth 12 -> 8 -> 4 at amounts 0.45 and 0.78, with +-0.01 of hysteresis.
TEST(Crush, BitDepthStepsWithHysteresis) {
    btdsp::Crush c;
    c.Init(kFs);
    const auto at = [&](float amount) {
        btdsp::CrushSettings s;
        s.amount = amount;
        c.Set(s);
        return c.Bits();
    };
    EXPECT_EQ(at(0.2f), 12);
    EXPECT_EQ(at(0.455f), 12);  // inside the band: stays
    EXPECT_EQ(at(0.465f), 8);
    EXPECT_EQ(at(0.445f), 8);  // inside the band on the way down: stays
    EXPECT_EQ(at(0.435f), 12);
    EXPECT_EQ(at(0.785f), 8);  // up two steps' worth, but short of 4 bits' band
    EXPECT_EQ(at(0.795f), 4);
    EXPECT_EQ(at(0.775f), 4);
    EXPECT_EQ(at(0.765f), 8);
    EXPECT_EQ(at(1.0f), 4);
    EXPECT_EQ(at(0.2f), 12);  // a jump crosses both steps at once
    btdsp::Crush fresh;
    fresh.Init(kFs);
    btdsp::CrushSettings full;
    full.amount = 1.0f;
    fresh.Set(full);
    EXPECT_EQ(fresh.Bits(), 4);
}
