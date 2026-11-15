// The engine invariants every DSP stage meets: block-size independence, finite output with no subnormals, bounded
// impulse response, and no click when a parameter jumps between its extremes. Stages are exercised through the engine
// with one parameter set; StageTypes lists them.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "test_util.h"

namespace {

using namespace bttest;
using namespace btparam;

// The engine with parameter Id at Value, Id2 at Value2 and Id3 at Value3 before the first Process (an id of -1: left
// at its default).
template <int Id, int Value, int Id2 = -1, int Value2 = 0, int Id3 = -1, int Value3 = 0> struct Stage : EngineRig {
    Stage() {
        if (Id >= 0) Set(uint8_t(Id), float(Value));
        if (Id2 >= 0) Set(uint8_t(Id2), float(Value2));
        if (Id3 >= 0) Set(uint8_t(Id3), float(Value3));
    }
};

// The EQ on at eq_position Position, every section live: low +8, bell 1 +9, high -6, bell 2 -6, bell 3 +4 at Q 4.
template <int Position> struct EqStage : EngineRig {
    EqStage() {
        Set(kEqEnable, 1.0f);
        Set(kEqPosition, float(Position));
        Set(kEqLowGain, 8.0f);
        Set(kEqMidGain1, 9.0f);
        Set(kEqHighGain, -6.0f);
        Set(kEqMidGain2, -6.0f);
        Set(kEqMidGain3, 4.0f);
        Set(kEqMidQ3, 4.0f);
    }
};

// The defaults (tape, color 0); knob 6 fully clockwise on each delay type (tape age 1 and the BBD fully degraded, each
// on the input, delay_tape_scope / delay_bbd_scope all, and in the loop, repeats; the bitcrush at its maximum on the
// input, delay_crush_scope all, and in the loop, delay_crush_scope repeats) and the BBD at color 0; SW1 down (reverse
// grains); the Plate (SW3 down) and the Canyon (SW3 up); the EQ on, pre and post; sound on sound on in overdub (the
// rig's short line clamps the loop to 0.34 s) and in 501. NoClickOnParamJump also jumps delay_type 0 -> 2 (SW2) and
// reverb_algo 0 -> 2 (SW3) on every stage, and every EQ parameter with the EQ live in both places.
using StageTypes =
    ::testing::Types<Stage<-1, 0>, Stage<kDelayColor, 100>, Stage<kDelayColor, 100, kDelayTapeScope, 0>,
                     Stage<kDelayType, 2, kDelayColor, 100>,
                     Stage<kDelayType, 2, kDelayColor, 100, kDelayCrushScope, 0>, Stage<kDelayType, 0>,
                     Stage<kDelayType, 0, kDelayColor, 100>, Stage<kDelayType, 0, kDelayColor, 100, kDelayBbdScope, 0>,
                     Stage<kDelayMode, 0>, Stage<kReverbAlgo, 0>, Stage<kReverbAlgo, 2>, EqStage<0>, EqStage<1>,
                     Stage<kDelaySosMode, 3, kDelaySos, 1, kDelaySosFeedback, 50>,
                     Stage<kDelaySosMode, 2, kDelaySos, 1>>;

// Processes in[from, to) into out in chunks of block.
template <class S> void Run(S& s, const std::vector<float>& in, std::vector<float>& out, size_t from, size_t to,
                            size_t block) {
    for (size_t i = from; i < to; i += block) s.Process(in.data() + i, out.data() + i, std::min(block, to - i));
}

// A fresh stage over the whole input.
template <class S> std::vector<float> Render(const std::vector<float>& in, size_t block) {
    S s;
    std::vector<float> out(in.size());
    Run(s, in, out, 0, in.size(), block);
    return out;
}

}  // namespace

template <class S> class DspInvariants : public ::testing::Test {};
TYPED_TEST_SUITE(DspInvariants, StageTypes);

TYPED_TEST(DspInvariants, BlockSizeIndependent) {
    const std::vector<float> in = Noise(size_t(kFs), 0.5f, 1);
    const std::vector<float> ref = Render<TypeParam>(in, 1);
    for (size_t block : {48u, 64u}) {
        const std::vector<float> y = Render<TypeParam>(in, block);
        EXPECT_EQ(std::memcmp(ref.data(), y.data(), ref.size() * sizeof(float)), 0) << "block " << block;
    }
}

TYPED_TEST(DspInvariants, FiniteNoSubnormal) {
    const size_t n = size_t(2 * kFs);
    const std::vector<std::vector<float>> inputs = {std::vector<float>(n, 0.0f), Noise(n, 1.0f, 2),
                                                    std::vector<float>(n, 1.0f), std::vector<float>(n, -1.0f)};
    for (size_t k = 0; k < inputs.size(); ++k) {
        const std::vector<float> y = Render<TypeParam>(inputs[k], 48);
        size_t bad = 0;
        for (float v : y) bad += !std::isfinite(v) || std::fpclassify(v) == FP_SUBNORMAL;
        EXPECT_EQ(bad, 0u) << "input " << k << " (silence, noise, DC +1, DC -1)";
    }
}

TYPED_TEST(DspInvariants, ImpulseBounded) {
    std::vector<float> in(size_t(1 + 5 * kFs), 0.0f);
    in[0] = 1.0f;
    const std::vector<float> y = Render<TypeParam>(in, 48);
    float peak = 0.0f;
    for (float v : y) peak = std::max(peak, std::fabs(v));
    EXPECT_LE(peak, 4.0f);
    const size_t half = size_t(kFs / 2);
    EXPECT_LE(Rms(y, y.size() - half, y.size()), Rms(y, 0, half) + 1e-12f);
}

TYPED_TEST(DspInvariants, NoClickOnParamJump) {
    const size_t half = size_t(kFs / 2);
    const std::vector<float> in = Sine(2 * half, 220.0f, 0.25f);
    for (const btparam::ParamDef& d : btparam::kParams) {
        SCOPED_TRACE(d.name);
        std::vector<float> held[2];
        for (int e = 0; e < 2; ++e) {
            TypeParam s;
            s.Set(d.id, e ? d.max : d.min);
            held[e].resize(in.size());
            Run(s, in, held[e], 0, in.size(), 48);
        }
        TypeParam s;
        s.Set(d.id, d.min);
        std::vector<float> jump(in.size());
        Run(s, in, jump, 0, half, 48);
        s.Set(d.id, d.max);
        Run(s, in, jump, half, in.size(), 48);
        EXPECT_LE(Click(jump), 2.0f * std::max(Click(held[0]), Click(held[1])) + 1e-3f);
    }
}
