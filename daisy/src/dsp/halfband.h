// HalfRateReverb: a reverb designed for kReverbRate (48 kHz) run inside the 96 kHz chain. The input is decimated 2:1,
// the reverb runs on the half-rate samples, its L + R is folded to mono (kReverbFold) and interpolated back 1:2.
// Both resamplers are the same 47-tap half-band FIR: a Kaiser (beta 8) windowed sinc, centre tap 0.5, even offsets 0;
// flat to 18 kHz (0.001 dB) and -80 dB from 30 kHz, so nothing above 24 kHz aliases into the reverb's input and the
// interpolator's image above 24 kHz is removed. Latency: 23 input samples (decimator) + 24 (interpolator) + 2 (the
// output FIFO), 49 samples at 96 kHz. Every state advances per sample, so the output does not depend on how the caller
// splits its blocks. No libDaisy, no allocation.
#pragma once

#include <cstddef>

#include "../memory.h"
#include "reverb.h"

namespace btdsp {

// The half-band's odd taps c_j at offsets +-(2j + 1), normalised so sum c_j = 0.25 (DC gain 1 with the 0.5 centre).
// numpy/scipy: k=arange(-23,24); h=0.5*sinc(k/2)*kaiser(47,8.0); c=h[24::2]; c*=0.25/c.sum()
constexpr float kHalfbandC[12] = {3.160600265e-01f,  -9.953366729e-02f, 5.323910908e-02f,  -3.190591831e-02f,
                                  1.951150296e-02f,  -1.168527653e-02f, 6.670786169e-03f,  -3.539435262e-03f,
                                  1.690635467e-03f,  -6.899972485e-04f, 2.146022812e-04f,  -3.236778986e-05f};

class HalfRateReverb {
public:
    static constexpr size_t kMaxBlock = 64;  // the longest Process call

    // Histories zero, the decimator's phase at its start, the output FIFO holding exactly 2 zeros.
    void Reset() {
        for (float& v : x_) v = 0.0f;
        for (float& v : v_) v = 0.0f;
        for (float& v : fifo_) v = 0.0f;
        xp_ = vp_ = 0;
        phase_ = false;
        rd_ = 0;
        wr_ = 2;
    }

    // n <= kMaxBlock samples at the full rate through r (running at half the rate); in must not alias out. The FIFO
    // holds 2 - (pending decimator input) samples before a call and at most n + 2 after its pushes, so a pop never
    // underruns.
    BT_ITCM void Process(Reverb& r, const float* in, float* out, size_t n) {
        constexpr size_t kHalf = kMaxBlock / 2 + 1;
        float d[kHalf], l[kHalf], rr[kHalf];
        size_t m = 0;
        for (size_t i = 0; i < n; ++i) {
            x_[xp_] = x_[xp_ + kTaps] = in[i];
            if (++xp_ == kTaps) xp_ = 0;
            phase_ = !phase_;
            if (phase_) continue;
            const float* w = x_ + xp_;  // w[0] oldest, w[46] newest
            float s = 0.5f * w[23];
            for (int j = 0; j < 12; ++j) s += kHalfbandC[j] * (w[24 + 2 * j] + w[22 - 2 * j]);
            d[m++] = s;
        }
        if (m > 0) r.Process(d, l, rr, m);
        for (size_t j = 0; j < m; ++j) {
            const float f = kReverbFold * (l[j] + rr[j]);
            v_[vp_] = v_[vp_ + kHist] = f;
            if (++vp_ == kHist) vp_ = 0;
            const float* v = v_ + vp_;  // v[0] oldest, v[23] newest
            float s = 0.0f;
            for (int k = 0; k < 12; ++k) s += kHalfbandC[k] * (v[12 + k] + v[11 - k]);
            fifo_[wr_++ & kFifoMask] = 2.0f * s;
            fifo_[wr_++ & kFifoMask] = v[12];
        }
        for (size_t i = 0; i < n; ++i) out[i] = fifo_[rd_++ & kFifoMask];
    }

private:
    static constexpr int kTaps = 47;  // the decimator's window
    static constexpr int kHist = 24;  // the interpolator's window (half-rate samples)
    static constexpr unsigned kFifoMask = 127;

    float x_[2 * kTaps] = {};  // the full-rate input, written twice: x_ + xp_ is the window in order
    float v_[2 * kHist] = {};  // the folded half-rate wet, written twice the same way
    float fifo_[kFifoMask + 1] = {};
    int xp_ = 0, vp_ = 0;
    bool phase_ = false;  // true: one input waits for its pair
    unsigned rd_ = 0, wr_ = 2;
};

}  // namespace btdsp
