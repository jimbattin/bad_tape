// daisy/src/dsp/blocks.h: the parameter smoother, the bit-pattern float tests against the float compares they replace.
// daisy/src/dsp/halfband.h: the half-rate reverb's resamplers.
#include "dsp/blocks.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "dsp/halfband.h"
#include "test_util.h"

namespace {

using namespace bttest;

// A reverb whose folded wet is its input: L = R = in / (2 kReverbFold).
struct PassReverb final : btdsp::Reverb {
    void Init(float, const btdsp::ReverbPool&) override {}
    void Reset() override {}
    void SetParam(uint8_t, float) override {}
    void Snap() override {}
    void Process(const float* in, float* outL, float* outR, size_t n) override {
        for (size_t i = 0; i < n; ++i) outL[i] = outR[i] = in[i] * (0.5f / btdsp::kReverbFold);
    }
};

// x through the decimator, PassReverb and the interpolator, in kBlock blocks.
std::vector<float> Resample(const std::vector<float>& x) {
    PassReverb pass;
    btdsp::HalfRateReverb rs;
    rs.Reset();
    std::vector<float> y(x.size());
    for (size_t i = 0; i < x.size(); i += kBlock)
        rs.Process(pass, x.data() + i, y.data() + i, std::min(kBlock, x.size() - i));
    return y;
}

}  // namespace

TEST(Smoother, OneTimeConstant) {
    btdsp::Smoother s;
    s.Init(1000.0f, 0.01f, 0.0f);
    for (int i = 0; i < 10; ++i) s.Process(1.0f);
    EXPECT_NEAR(s.Value(), 1.0f - std::exp(-1.0f), 1e-4f);
}

// Every helper gives the float compare's answer (bit for bit where it returns a float) on every non-NaN value: the
// zeros, denormals, the thresholds and their neighbours, infinities and random values over the whole exponent range.
TEST(BitTests, MatchFloatCompares) {
    const float cs[] = {1e-15f, 1e-6f, 1.0f / 256.0f, 0.1f, 0.5f, 1.0f, 3.0f};
    std::vector<float> xs = {0.0f, std::numeric_limits<float>::denorm_min(), std::numeric_limits<float>::min(),
                             std::numeric_limits<float>::max(), std::numeric_limits<float>::infinity()};
    for (float c : cs) xs.insert(xs.end(), {c, std::nextafter(c, 0.0f), std::nextafter(c, 10.0f)});
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> mant(1.0f, 2.0f);
    std::uniform_int_distribution<int> ex(-140, 127);
    for (int i = 0; i < 20000; ++i) xs.push_back(std::ldexp(mant(rng), ex(rng)));
    const size_t n = xs.size();
    for (size_t i = 0; i < n; ++i) xs.push_back(-xs[i]);
    const auto bits = [](float v) {
        uint32_t b;
        std::memcpy(&b, &v, sizeof b);
        return b;
    };
    const auto softClipRef = [](float x) {
        if (x >= 3.0f) return 1.0f;
        if (x <= -3.0f) return -1.0f;
        return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
    };
    for (float x : xs) {
        SCOPED_TRACE(x);
        EXPECT_EQ(bits(btdsp::Flush(x)), bits(std::fabs(x) < 1e-15f ? 0.0f : x));
        EXPECT_EQ(btdsp::IsZero(x), x == 0.0f);
        EXPECT_EQ(btdsp::IsPositive(x), x > 0.0f);
        for (float c : cs) {
            EXPECT_EQ(btdsp::Below(x, c), x < c);
            EXPECT_EQ(btdsp::Above(x, c), x > c);
        }
        EXPECT_EQ(bits(btdsp::SoftClip(x)), bits(softClipRef(x)));
        EXPECT_EQ(bits(btdsp::Clamp(x, -3.0f, 3.0f)), bits(std::clamp(x, -3.0f, 3.0f)));
        EXPECT_EQ(bits(btdsp::Max(x, 0.5f)), bits(std::max(x, 0.5f)));
        EXPECT_EQ(bits(btdsp::Min(x, 0.5f)), bits(std::min(x, 0.5f)));
    }
}

// A passband tone comes back at its level; a tone above the half rate's Nyquist leaves no alias at 48 kHz - f.
TEST(HalfRateReverb, PassbandAndAliasRejection) {
    const size_t n = Samples(1.0), from = Samples(0.1);
    const std::vector<float> pass = Resample(Sine(n, 5000.0f, 0.5f));
    EXPECT_NEAR(Amplitude(pass, from, n, 5000.0), 0.5, 0.5 * 0.012);
    const std::vector<float> stop = Resample(Sine(n, 40000.0f, 0.5f));
    EXPECT_LT(Amplitude(stop, from, n, 8000.0), 0.5 * std::pow(10.0, -70.0 / 20.0));
}
