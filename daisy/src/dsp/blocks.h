// The DSP building blocks the delay, the reverbs and the EQ share: constants, the glide laws, coefficient helpers,
// exact float tests on the bits (no float compare per sample), flush to zero, dB to gain, a soft clipper, a
// state-variable filter and the 4-pole Butterworth built on it, a one-pole lowpass, parameter smoothers and ramps, a
// smoothstep crossfade, a noise gate, a random source, a quadrature oscillator, and the reverbs' fractional delay line,
// Schroeder allpass and rate-limited glide. No libDaisy, no allocation. BT_ITCM (../memory.h) puts the per-sample and
// per-tick functions in ITCM on the Seed.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../memory.h"

namespace btdsp {

template <class T> inline constexpr T kPiT = T(3.14159265358979323846L);  // pi in T
constexpr float kPi = kPiT<float>, kHalfPi = 1.5707963f, kTwoPi = 6.28318531f;
constexpr float kSampleRate = 96000.0f;            // the firmware's audio rate
constexpr float kReverbRate = kSampleRate / 2.0f;  // the reverbs' rate (engine.h: HalfRateReverb)
constexpr size_t kBlockSize = 32;                  // the firmware's audio block (1/3 ms)
constexpr int kControlInterval = 32;        // samples per control tick at kSampleRate (3 kHz): delay, EQ
constexpr int kReverbControlInterval = 16;  // samples per control tick at kReverbRate (3 kHz): the reverbs
constexpr float kMaxCutoff = 0.45f;   // filter cutoffs are capped at this fraction of fs
constexpr float kUniformPeakPerRms = 1.7320508f;  // sqrt(3): uniform noise in +-1 has RMS 1/sqrt(3)
constexpr float kButterworth2K = 1.41421356f;     // k = 1/Q of a 2-pole Butterworth
constexpr float kButterworth4K[2] = {1.84775907f, 0.76536686f};  // k = 1/Q of a 4-pole Butterworth's two sections
constexpr float kLn1000 = 6.9077553f;  // ln 1000: an RT60 decays by 1000 (60 dB)

// Glide laws (Glide: the delay's read heads, the reverbs' size, predelay, spread and echo delays, and the Canyon's line
// bases): the rate chases (target - x) / (time x fs), with inertia.
constexpr float kGlideS = 0.15f;         // the time a delay-time change glides over
constexpr float kGlideInertiaS = 0.02f;  // inertia of the read speed (no kink at a change)
constexpr float kGlideMaxRate = 0.25f;   // reverb predelay/spread/echo |dDelay/dn| limit, read speed 0.75..1.25
// Reverb SIZE (knob 3's space macros move it while it is turned): a slower glide and at most 5 % read-speed change on
// the longest line (under a semitone), so turning the knob bends the tail's pitch only slightly.
constexpr float kSizeGlideS = 0.5f;
constexpr float kSizeGlideMaxRate = 0.05f;

// The noise gate (tape hiss, BBD noise): a peak follower on the signal the noise rides on.
constexpr float kNoiseGateFull = 0.01f;     // -40 dBFS: full noise at and above
constexpr float kNoiseGateFloor = 1e-6f;    // -120 dBFS: exactly 0 below (~1.3 s of release after the signal stops)
constexpr float kNoiseGateReleaseS = 0.1f;

// The one-pole lowpass y += a (x - y) with its corner at hz.
BT_ITCM inline float OnePoleCoef(float hz, float fs) {
    return std::min(1.0f, 1.0f - std::exp(-2.0f * kPi * hz / fs));
}
// The one-pole coefficient of time constant seconds, updated at rateHz.
inline float TimeCoef(float seconds, float rateHz) { return 1.0f - std::exp(-1.0f / (seconds * rateHz)); }
// 0 -> 0, 1 -> 1, zero slope at both ends.
BT_ITCM inline float SmoothStep(float x) { return x * x * (3.0f - 2.0f * x); }

// Float tests on the bits, as integer compares: a float compare on the Seed is vcmp then vmrs, which stalls the
// pipeline on every call. Each is exact for every non-NaN argument.
BT_ITCM inline std::int32_t Bits(float x) {
    std::int32_t b;
    std::memcpy(&b, &x, sizeof b);
    return b;
}
// a and b have the same bit pattern: equal (never +0 against -0), or the same NaN.
BT_ITCM inline bool SameBits(float a, float b) { return Bits(a) == Bits(b); }
// x == 0 (+0 or -0).
BT_ITCM inline bool IsZero(float x) { return (std::uint32_t(Bits(x)) << 1) == 0u; }
// x > 0: positive floats are the positive integers; +0, -0 and negative floats are not.
BT_ITCM inline bool IsPositive(float x) { return Bits(x) > 0; }
// x < c and x > c for c > 0: below c, every float (negatives and -0 included) has smaller bits.
BT_ITCM inline bool Below(float x, float c) { return Bits(x) < Bits(c); }
BT_ITCM inline bool Above(float x, float c) { return Bits(x) > Bits(c); }
// min and max of non-NaN a and b, on the Seed vminnm and vmaxnm (no compare); they differ from std::min and std::max
// only on +0 against -0, so they replace them where the operands cannot be zeros of opposite signs.
BT_ITCM inline float Min(float a, float b) { return std::fmin(a, b); }
BT_ITCM inline float Max(float a, float b) { return std::fmax(a, b); }
// x clamped to [lo, hi], lo <= hi, x not NaN (as std::clamp, where x and the bounds cannot be zeros of opposite signs).
BT_ITCM inline float Clamp(float x, float lo, float hi) { return std::fmin(std::fmax(x, lo), hi); }

// Underflow guard for recirculating state: magnitudes below the threshold (1e-15, about -300 dBFS) become exact 0, so
// tails end in true silence and host builds (no FZ) never run on denormals. FPSCR.FZ on the Seed only covers
// denormals. Tested on the bits (|v| < 1e-15f exactly, NaN kept).
BT_ITCM inline float Flush(float v) {
    return (std::uint32_t(Bits(v)) & 0x7FFFFFFFu) < std::uint32_t(Bits(1e-15f)) ? 0.0f : v;
}

// The linear gain of db decibels (always_inline: a template's out-of-line copy loses BT_ITCM, ../memory.h).
template <class T> [[gnu::always_inline]] inline T DbToGain(T db) { return std::pow(T(10), db / T(20)); }

// tanh-like: slope 1 at 0, reaches +-1 at +-3 with zero slope. The polynomial is exactly +-1 at +-3, so the clamp
// (no compares) gives the same value as the branches beyond.
BT_ITCM inline float SoftClip(float x) {
    x = Clamp(x, -3.0f, 3.0f);
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// Simper TPT state-variable lowpass (one 2-pole section). Its members are always_inline: they run per sample inside
// BT_ITCM callers.
struct Svf {
    float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f, ic1 = 0.0f, ic2 = 0.0f;
    // f: cutoff as a fraction of fs (< 0.5); k: 1/Q.
    [[gnu::always_inline]] inline void Set(float f, float k) { SetG(std::tan(kPi * f), k); }
    // g = tan(pi f), precomputed.
    [[gnu::always_inline]] inline void SetG(float g, float k) {
        a1 = 1.0f / (1.0f + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }
    // band: the bandpass output (unity gain at the cutoff when k = 1).
    [[gnu::always_inline]] inline float Process(float v, float& band) {
        const float v3 = v - ic2;
        const float v1 = a1 * ic1 + a2 * v3;
        const float v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1 = Flush(2.0f * v1 - ic1);
        ic2 = Flush(2.0f * v2 - ic2);
        band = v1;
        return v2;
    }
    [[gnu::always_inline]] inline float Process(float v) {
        float b;
        return Process(v, b);
    }
    void Clear() { ic1 = ic2 = 0.0f; }
    // The lowpass's steady state at a constant input v (a filter starting warm on a running signal).
    [[gnu::always_inline]] inline void Warm(float v) {
        ic1 = 0.0f;
        ic2 = v;
    }
};

// 4-pole Butterworth lowpass: two Svf sections.
struct Butterworth4 {
    Svf s[2];
    // f: cutoff as a fraction of fs (< 0.5).
    BT_ITCM void Set(float f) {
        s[0].Set(f, kButterworth4K[0]);
        s[1].Set(f, kButterworth4K[1]);
    }
    // always_inline: GCC's inlining of it into the BBD and crush paths flips with unrelated code changes.
    [[gnu::always_inline]] inline float Process(float v) { return s[1].Process(s[0].Process(v)); }
    void Clear() {
        s[0].Clear();
        s[1].Clear();
    }
    // Both sections at their steady state for a constant input v.
    BT_ITCM void Warm(float v) {
        s[0].Warm(v);
        s[1].Warm(v);
    }
};

// The one-pole lowpass y += a (x - y), flushed.
struct OnePole {
    float y = 0.0f;
    BT_ITCM float Process(float x, float a) { return y = Flush(y + a * (x - y)); }
};

// One-pole smoothing of a parameter updated at rateHz, time constant tauS.
class Smoother {
public:
    void Init(float rateHz, float tauS, float value) {
        a_ = TimeCoef(tauS, rateHz);
        y_ = value;
    }
    BT_ITCM float Process(float target) {
        y_ += a_ * (target - y_);
        return y_;
    }
    BT_ITCM float Value() const { return y_; }

private:
    float a_ = 1.0f, y_ = 0.0f;
};

// Linear ramp toward a target at 1/(seconds*rateHz) per call; reaches the target exactly (bypass and enable gains need
// exact 0 and 1).
class Ramp {
public:
    void Init(float rateHz, float seconds, float value) {
        step_ = 1.0f / (seconds * rateHz);
        y_ = value;
    }
    BT_ITCM float Process(float target) {
        if (SameBits(y_, target)) return y_;  // at rest: no float compares
        if (y_ < target) y_ = std::min(y_ + step_, target);
        else if (y_ > target) y_ = std::max(y_ - step_, target);
        return y_;
    }
    BT_ITCM float Value() const { return y_; }

private:
    float step_ = 1.0f, y_ = 0.0f;
};

// A value set at the control rate and ramped linearly across the tick of Interval samples. Its members are
// always_inline: a template's out-of-line copy loses BT_ITCM (../memory.h).
template <int Interval> struct TickRampT {
    float t = 0.0f, v = 0.0f, d = 0.0f;  // target, value, step
    [[gnu::always_inline]] inline void Set(float target) {
        t = target;
        d = (target - v) * (1.0f / float(Interval));
    }
    [[gnu::always_inline]] inline void Snap() {
        v = t;
        d = 0.0f;
    }
    [[gnu::always_inline]] inline float Step() { return v += d; }
};
using TickRamp = TickRampT<kControlInterval>;              // at kSampleRate (tape)
using ReverbTickRamp = TickRampT<kReverbControlInterval>;  // at kReverbRate (the reverbs)

// The smoothstep weight of the new source, 0 -> 1 over len samples; Advance after this sample's Weight, true when it
// ends.
struct Crossfade {
    int pos = 0, len = 1;
    bool on = false;
    BT_ITCM void Start(int length) {
        pos = 0;
        len = length;
        on = true;
    }
    BT_ITCM float Weight() const { return SmoothStep(float(pos) / float(len)); }
    // from crossfaded toward to by Weight().
    BT_ITCM float Mix(float from, float to) const { return from + Weight() * (to - from); }
    BT_ITCM bool Advance() {
        if (++pos < len) return false;
        on = false;
        return true;
    }
};

// Gates noise riding on a signal: a peak follower (instant attack, kNoiseGateReleaseS release) on the signal; the
// noise is at full level from kNoiseGateFull and exactly 0 below kNoiseGateFloor.
struct NoiseGate {
    float env = 0.0f, rel = 0.0f;
    void Init(float fs) {
        env = 0.0f;
        rel = std::exp(-1.0f / (kNoiseGateReleaseS * fs));
    }
    void Clear() { env = 0.0f; }
    BT_ITCM void Follow(float x) {
        env = Max(std::fabs(x), env * rel);
        if (Below(env, kNoiseGateFloor)) env = 0.0f;
    }
    BT_ITCM bool Open() const { return IsPositive(env); }
    BT_ITCM float Level() const { return Min(1.0f, env / kNoiseGateFull); }
};

// xorshift32; s must not be 0.
struct XorShift32 {
    uint32_t s;
    // [0, 1).
    BT_ITCM float Uniform() {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return float(s >> 8) * (1.0f / 16777216.0f);
    }
    // [-1, 1).
    BT_ITCM float Bipolar() { return 2.0f * Uniform() - 1.0f; }
};

// A sine/cosine pair (c, s) rotated by (rc, rs) = (cos w, sin w) per Step.
struct Oscillator {
    float c = 1.0f, s = 0.0f, rc = 1.0f, rs = 0.0f;
    // w <= 2 pi 10 Hz / fs: the series to w^3 is exact in float.
    BT_ITCM void SetRate(float w) {
        rc = 1.0f - 0.5f * w * w;
        rs = w - w * w * w * (1.0f / 6.0f);
    }
    BT_ITCM void Step() {
        const float x = c;
        c = x * rc - s * rs;
        s = s * rc + x * rs;
    }
    // Pulls (c, s) back onto the unit circle (one Newton step); call now and then against rounding drift.
    BT_ITCM void Renormalize() {
        const float k = 0.5f * (3.0f - (c * c + s * s));
        c *= k;
        s *= k;
    }
};

// A circular line of cap floats; w is the next slot written.
struct Line {
    float* buf = nullptr;
    int cap = 0, w = 0;
    BT_ITCM void Write(float v) {
        buf[w] = v;
        if (++w == cap) w = 0;
    }
    // d samples behind the next write, 1 <= d <= cap - 2, linear interpolation. Called before Write in the same
    // sample it delays by exactly d.
    BT_ITCM float Read(float d) const {
        const int di = int(d);
        const float f = d - float(di);
        int i0 = w - di;
        if (i0 < 0) i0 += cap;
        int i1 = i0 - 1;
        if (i1 < 0) i1 += cap;
        return buf[i0] + f * (buf[i1] - buf[i0]);
    }
    // Exactly d samples behind the next write, 1 <= d <= cap - 1, no interpolation (Read at an integer d).
    BT_ITCM float ReadAt(int d) const {
        int i = w - d;
        if (i < 0) i += cap;
        return buf[i];
    }
};

// Schroeder allpass: v = x + g * v[n - d], y = v[n - d] - g * v.
struct Allpass {
    Line line;
    BT_ITCM float Process(float x, float d, float g) { return Feed(x, line.Read(d), g); }
    // The same with z = v[n - d] read by the caller (a moving read head): writes v, returns y.
    BT_ITCM float Feed(float x, float z, float g) {
        const float v = Flush(x + g * z);
        line.Write(v);
        return z - g * v;
    }
};

// Points l at the next cap floats of p and advances p: an algorithm's pool layout at Init.
inline void Carve(float*& p, Line& l, int cap) {
    l.buf = p;
    l.cap = cap;
    l.w = 0;
    p += cap;
}

// l read d behind, d clamped to [lo, cap - 2].
BT_ITCM inline float Tap(const Line& l, float d, float lo) { return l.Read(Clamp(d, lo, float(l.cap - 2))); }

// A value gliding to its target: the rate chases a * (target - x), limited to +-maxRate, with inertia b, and snaps to
// the target within eps. Kept as an error relative to the target, so the approach never stalls on float resolution at
// large values. Process is noclone: a local clone (.constprop) beside an out-of-line COMDAT copy in one translation
// unit is a section type conflict (../memory.h), which the bench image's inlining produces.
struct Glide {
    float t = 0.0f, e = 0.0f, v = 0.0f;  // target, x - target, rate
    [[gnu::noclone]] BT_ITCM float Process(float target, float a, float b, float maxRate, float eps) {
        if (SameBits(target, t) && IsZero(e) && IsZero(v)) return t;  // at rest: no float compares
        if (target != t) {
            e += t - target;
            t = target;
        }
        if (e == 0.0f && v == 0.0f) return t;
        const float vT = Clamp(-e * a, -maxRate, maxRate);
        v += b * (vT - v);
        e += v;
        if (Below(std::fabs(e), eps) && Below(std::fabs(v), eps * 1e-2f)) e = v = 0.0f;  // eps > 0
        return t + e;
    }
    BT_ITCM float Value() const { return t + e; }
    BT_ITCM void Snap(float target) {
        t = target;
        e = v = 0.0f;
    }
};

// The glide laws' coefficients at fs: a (kGlideS), aSize (kSizeGlideS) and the inertia b (kGlideInertiaS).
struct GlideCoefs {
    float a = 0.0f, aSize = 0.0f, b = 0.0f;
    void Init(float fs) {
        a = TimeCoef(kGlideS, fs);
        aSize = TimeCoef(kSizeGlideS, fs);
        b = TimeCoef(kGlideInertiaS, fs);
    }
};

}  // namespace btdsp
