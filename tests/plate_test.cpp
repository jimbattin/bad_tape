// The Plate reverb: MID DECAY tracks the measured RT60, LF DECAY lengthens the bass, the
// decay stays bounded at every corner of the loop parameters, the stereo pair is decorrelated (the fold gain), a SIZE
// change glides without clearing the tail, STOP DECAY cuts the free tail, and plate_space (knob 3) lengthens it.
#include "dsp/plate.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;
using PlateRig = ReverbRig<btdsp::Plate>;

}  // namespace

TEST(Plate, MidDecayTracksMeasuredRt60) {
    for (float rt : {0.5f, 2.0f, 6.0f}) {
        SCOPED_TRACE(rt);
        PlateRig r;
        r.Set(btparam::kPlateMidDecay, rt);
        r.Set(btparam::kPlateLfDecay, rt);
        const std::vector<float> y = r.Render(Impulse(Samples(1.2 * rt + 1.0, kReverbFs), 0.5f));
        const double m = Rt60(y, 1000.0f, kReverbFs);
        EXPECT_NEAR(m / rt, 1.0, 0.15) << "measured " << m;
    }
}

TEST(Plate, LfDecayLengthensBass) {
    PlateRig r;
    r.Set(btparam::kPlateMidDecay, 1.0f);
    r.Set(btparam::kPlateLfDecay, 4.0f);
    r.Set(btparam::kPlateCrossover, 500.0f);
    const std::vector<float> y = r.Render(Impulse(Samples(6.0, kReverbFs), 0.5f));
    const double lo = Rt60(y, 125.0f, kReverbFs), hi = Rt60(y, 4000.0f, kReverbFs);
    EXPECT_GE(lo, 2.5) << lo;
    EXPECT_GE(hi, 0.7) << hi;
    EXPECT_LE(hi, 1.2) << hi;
}

TEST(Plate, DecayStableAtEveryCorner) {
    using namespace btparam;
    const uint8_t ids[] = {kPlateMidDecay, kPlateLfDecay,   kPlateCrossover, kPlateTrebleDecay,
                           kPlateSize,     kPlateDefinition, kPlateChorus,   kPlateDiffusion};
    ExpectDecayStableAtCorners<btdsp::Plate>(ids);
}

// Justifies kReverbFold = 1/sqrt(2): decorrelated L and R add in power.
TEST(Plate, StereoIsDecorrelated) { EXPECT_LT(StereoCorrelation<btdsp::Plate>(), 0.5); }

TEST(Plate, SizeGlidesKeepingTheTail) {
    PlateRig r;
    r.Set(btparam::kPlateSize, 0.0f);
    const std::vector<float> in = Impulse(Samples(1.0, kReverbFs), 0.5f);
    std::vector<float> y(in.size()), l(in.size()), rr(in.size());
    const size_t half = Samples(0.5, kReverbFs);
    for (size_t i = 0; i < in.size(); i += kBlock) {
        if (i == half) r.Set(btparam::kPlateSize, 100.0f);
        r.Process(in.data() + i, l.data() + i, rr.data() + i, std::min(kBlock, in.size() - i));
    }
    for (size_t i = 0; i < y.size(); ++i) y[i] = btdsp::kReverbFold * (l[i] + rr[i]);
    EXPECT_GE(Rms(y, half, Samples(0.55, kReverbFs)), 0.5 * Rms(y, Samples(0.45, kReverbFs), half));
}

TEST(Plate, StopDecayCutsTheTail) {
    const auto render = [](float stop) {
        PlateRig r;
        r.Set(btparam::kPlateMidDecay, 6.0f);
        r.Set(btparam::kPlateLfDecay, 6.0f);
        r.Set(btparam::kPlateMidStopDecay, 0.5f);
        r.Set(btparam::kPlateLfStopDecay, 0.5f);
        r.Set(btparam::kPlateStopDelay, 100.0f);
        r.Set(btparam::kPlateStopDecay, stop);
        std::vector<float> in = Noise(Samples(2.0, kReverbFs), 0.5f, 10);
        std::fill(in.begin() + Samples(0.5, kReverbFs), in.end(), 0.0f);
        const std::vector<float> y = r.Render(in);
        return Rms(y, Samples(1.5, kReverbFs), Samples(2.0, kReverbFs));
    };
    const double on = render(1.0f), off = render(0.0f);
    EXPECT_LE(20.0 * std::log10(on / off), -30.0) << on << " " << off;
}

// Knob 3: each plate_space target at its law's value; RT60 rises with space and follows the law's MID DECAY.
TEST(Plate, SpaceLengthensDecay) {
    using namespace btparam;
    double last = 0.0;
    for (float space : {0.0f, 50.0f, 100.0f}) {
        SCOPED_TRACE(space);
        PlateRig r;
        float mid = 0.0f;
        for (const Link& l : kLinks) {
            if (l.source != kPlateSpace) continue;
            r.Set(l.target, LinkValue(l, space));
            if (l.target == kPlateMidDecay) mid = LinkValue(l, space);
        }
        const std::vector<float> y = r.Render(Impulse(Samples(1.2 * mid + 1.0, kReverbFs), 0.5f));
        const double m = Rt60(y, 1000.0f, kReverbFs);
        EXPECT_GT(m, last);
        EXPECT_NEAR(m / mid, 1.0, 0.2) << "measured " << m;
        last = m;
    }
}
