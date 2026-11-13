// The Hall reverb: RT tracks hall_rt_mid, the decay stays bounded at every corner of the loop parameters, the stereo
// pair is decorrelated (the fold gain), SHAPE/SPREAD delay the build-up, a SIZE change glides without clearing the
// tail, and pre-echoes land on their side.
#include "dsp/hall.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;
using HallRig = ReverbRig<btdsp::Hall>;

}  // namespace

TEST(Hall, RtMidTracksMeasuredRt60) {
    for (float rt : {0.5f, 2.0f, 6.0f}) {
        SCOPED_TRACE(rt);
        HallRig r;
        r.Set(btparam::kHallRtMid, rt);
        const std::vector<float> y = r.Render(Impulse(Samples(1.2 * rt + 1.0, kReverbFs), 0.5f));
        const double m = Rt60(y, 1000.0f, kReverbFs);
        EXPECT_NEAR(m / rt, 1.0, 0.15) << "measured " << m;
    }
}

TEST(Hall, DecayStableAtEveryCorner) {
    using namespace btparam;
    const uint8_t ids[] = {kHallRtMid, kHallBassMult, kHallCrossover, kHallRtHfCut,
                           kHallSize,  kHallDiffusion, kHallSpin,     kHallWander};
    ExpectDecayStableAtCorners<btdsp::Hall>(ids);
}

// Justifies kReverbFold = 1/sqrt(2): decorrelated L and R add in power.
TEST(Hall, StereoIsDecorrelated) { EXPECT_LT(StereoCorrelation<btdsp::Hall>(), 0.5); }

// A delay path that the random taps never move (two output taps reading one line at a fixed spacing, a loop allpass
// at a fixed length) is a static comb on the whole tail, heard as ringing at long RT: a peak in the autocorrelation
// of the mono fold's envelope-normalised tail, 1-2.5 s after an impulse, at some lag 5-300 ms.
TEST(Hall, FoldedTailHasNoFixedComb) {
    for (float size : {30.0f, 39.0f}) {
        SCOPED_TRACE(size);
        HallRig r;
        r.Set(btparam::kHallSize, size);
        r.Set(btparam::kHallRtMid, 6.0f);
        const std::vector<float> y = r.Render(Impulse(Samples(2.5, kReverbFs), 0.5f));
        const size_t from = Samples(1.0, kReverbFs), to = y.size(), half = Samples(0.025, kReverbFs);
        std::vector<double> z(to - from);
        for (size_t i = from; i < to; ++i) {
            const size_t a = std::max(from, i - std::min(i, half)), b = std::min(to, i + half);
            z[i - from] = y[i] / (Rms(y, a, b) + 1e-30);
        }
        double worst = 0.0;
        size_t at = 0;
        for (size_t lag = Samples(0.005, kReverbFs); lag <= Samples(0.3, kReverbFs); ++lag) {
            double xy = 0.0, xx = 0.0, yy = 0.0;
            for (size_t i = 0; i + lag < z.size(); ++i) {
                xy += z[i] * z[i + lag];
                xx += z[i] * z[i];
                yy += z[i + lag] * z[i + lag];
            }
            const double c = std::fabs(xy) / std::sqrt(xx * yy);
            if (c > worst) {
                worst = c;
                at = lag;
            }
        }
        EXPECT_LT(worst, 0.2) << "at " << 1000.0 * double(at) / kFs << " ms";
    }
}

TEST(Hall, ShapeAndSpreadDelayTheBuildUp) {
    const auto peakAt = [](float v) {
        HallRig r;
        r.Set(btparam::kHallShape, v);
        r.Set(btparam::kHallSpread, v);
        return EnvelopePeak(r.Render(Impulse(Samples(1.0, kReverbFs), 0.5f)), kReverbFs);
    };
    const size_t early = peakAt(0.0f), late = peakAt(100.0f);
    EXPECT_GE(double(late), double(early) + 0.03 * kFs) << early << " " << late;
}

TEST(Hall, SizeGlidesKeepingTheTail) {
    HallRig r;
    r.Set(btparam::kHallSize, 10.0f);
    const std::vector<float> in = Impulse(Samples(1.0, kReverbFs), 0.5f);
    std::vector<float> y(in.size());
    const size_t half = Samples(0.5, kReverbFs);
    std::vector<float> l(in.size()), rr(in.size());
    for (size_t i = 0; i < in.size(); i += kBlock) {
        if (i == half) r.Set(btparam::kHallSize, 39.0f);
        r.Process(in.data() + i, l.data() + i, rr.data() + i, std::min(kBlock, in.size() - i));
    }
    for (size_t i = 0; i < y.size(); ++i) y[i] = btdsp::kReverbFold * (l[i] + rr[i]);
    EXPECT_GE(Rms(y, half, Samples(0.55, kReverbFs)), 0.5 * Rms(y, Samples(0.45, kReverbFs), half));
}

TEST(Hall, EchoTapsLandOnTheirSide) {
    HallRig r;
    r.Set(btparam::kHallPredelay, 500.0f);
    r.Set(btparam::kHallHfCutoff, 20000.0f);
    r.Set(btparam::kHallEchoLevel1, 0.0f);
    r.Set(btparam::kHallEchoDelay1, 100.0f);
    std::vector<float> l, rr;
    r.Render(Impulse(Samples(0.2, kReverbFs), 1.0f), l, rr);
    float pl = 0.0f, pr = 0.0f;
    for (size_t i = 4795; i <= 4805; ++i) pl = std::max(pl, std::fabs(l[i]));
    for (size_t i = 4700; i <= 4900; ++i) pr = std::max(pr, std::fabs(rr[i]));
    EXPECT_GE(pl, 0.5f);  // the pre-echoes are at unity, outside kHallOutGain
    EXPECT_LE(pr, 1e-3f);
}
