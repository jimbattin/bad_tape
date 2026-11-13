// daisy/src/dsp/blocks.h: the parameter smoother. daisy/src/dsp/halfband.h: the half-rate reverb's resamplers.
#include "dsp/blocks.h"

#include <gtest/gtest.h>

#include <cmath>
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

// A passband tone comes back at its level; a tone above the half rate's Nyquist leaves no alias at 48 kHz - f.
TEST(HalfRateReverb, PassbandAndAliasRejection) {
    const size_t n = Samples(1.0), from = Samples(0.1);
    const std::vector<float> pass = Resample(Sine(n, 5000.0f, 0.5f));
    EXPECT_NEAR(Amplitude(pass, from, n, 5000.0), 0.5, 0.5 * 0.012);
    const std::vector<float> stop = Resample(Sine(n, 40000.0f, 0.5f));
    EXPECT_LT(Amplitude(stop, from, n, 8000.0), 0.5 * std::pow(10.0, -70.0 / 20.0));
}
