// The SW3 algorithm switch through btdsp::Engine: the old tail is cut and the new algorithm starts on a clean pool and
// state, a return during the fade-out keeps the tail, and the switch does not depend on how the caller splits its
// blocks.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <vector>

#include "dsp/engine.h"
#include "test_util.h"

namespace {

using namespace bttest;
using btdsp::Engine;

constexpr size_t kF = size_t(Engine::kAlgoFadeSamples), kC = size_t(Engine::kAlgoClearSamples);

// An engine with only the reverb's wet at the output.
struct Rig : EngineRig {
    Rig() {
        Set(btparam::kDelayMix, 0.0f);
        Set(btparam::kReverbMix, 100.0f);
    }
};

}  // namespace

TEST(Switch, OldTailCutNewAlgorithmStartsClean) {
    const size_t s = Samples(0.3), at = s + kF + kC + kF;
    std::vector<float> inA = Noise(at + Samples(1.0), 0.5f, 12), yA(inA.size());
    std::fill(inA.begin() + Samples(0.3), inA.end(), 0.0f);
    inA[at] = 0.5f;
    Rig a;  // Hall
    a.Feed(inA, yA, 0, s);
    a.e.SetParam(btparam::kReverbAlgo, 2.0f);
    a.Feed(inA, yA, s, inA.size());

    std::vector<float> inB(kF + Samples(1.0), 0.0f), yB(inB.size());
    inB[kF] = 0.5f;
    Rig b;
    b.e.SetParam(btparam::kReverbAlgo, 2.0f);  // before the first Process: no fade, no clear
    b.Feed(inB, yB, 0, inB.size());

    float silent = 0.0f, diff = 0.0f;
    for (size_t i = s + kF; i < s + kF + kC; ++i) silent = std::max(silent, std::fabs(yA[i]));
    EXPECT_EQ(silent, 0.0f);
    for (size_t i = 0; i < Samples(1.0); ++i) diff = std::max(diff, std::fabs(yA[at + i] - yB[kF + i]));
    EXPECT_LE(diff, 1e-6f);
    EXPECT_GT(Rms(yB, kF, yB.size()), 1e-4);  // the Canyon did answer
}

TEST(Switch, ReturnDuringFadeKeepsTheTail) {
    const size_t s = Samples(0.2);
    const std::vector<float> in = Impulse(Samples(1.0), 0.5f);
    std::vector<float> ref(in.size()), y(in.size());
    Rig r0;
    r0.Feed(in, ref, 0, in.size());
    Rig r;
    r.Feed(in, y, 0, s);
    r.e.SetParam(btparam::kReverbAlgo, 0.0f);
    r.Feed(in, y, s, s + kF / 2);
    r.e.SetParam(btparam::kReverbAlgo, 1.0f);
    r.Feed(in, y, s + kF / 2, in.size());
    EXPECT_LT(std::fabs(y[s + kF / 2 - 1]), std::fabs(ref[s + kF / 2 - 1]) * 0.51f + 1e-9f);  // it did fade
    for (size_t i = s + kF; i < in.size(); ++i) ASSERT_EQ(y[i], ref[i]) << i;
}

TEST(Switch, BlockSizeIndependentAcrossSwitches) {
    const std::vector<float> in = Noise(Samples(1.0), 0.5f, 13);
    const auto render = [&](size_t block) {
        Rig r;
        std::vector<float> y(in.size());
        r.Feed(in, y, 0, Samples(0.2), block);
        r.e.SetParam(btparam::kReverbAlgo, 0.0f);
        r.Feed(in, y, Samples(0.2), Samples(0.4), block);
        r.e.SetParam(btparam::kReverbAlgo, 2.0f);
        r.Feed(in, y, Samples(0.4), in.size(), block);
        return y;
    };
    const std::vector<float> ref = render(1);
    for (size_t block : {48u, 64u}) {
        const std::vector<float> y = render(block);
        EXPECT_EQ(std::memcmp(ref.data(), y.data(), ref.size() * sizeof(float)), 0) << "block " << block;
    }
}
