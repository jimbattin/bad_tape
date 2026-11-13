// SW2 down: the delay's bucket-brigade (BBD) character, inside the feedback loop like the bitcrush (crush.h), so every
// repeat passes through it again, or on the chain's input with delay_bbd_scope all (delay.h ColorInput, a second Bbd
// clocked from the delay time). A BBD pedal's path per pass: 2:1 compressor -> anti-alias lowpass -> the chip (a
// sample-and-hold clocked from the delay time, with overload and its own noise) -> reconstruction lowpass -> 1:2
// expander. Per sample:
// - compander (NE570 style): the compressor is the feedback type, its gain kBbdRef / env with env following the signal
//   it hands the chip; the expander's gain is env' / kBbdRef with env' following the signal it gets back, with the same
//   attack and release. The pair cancels except where the chip changes the signal in between (filtering, overload,
//   noise): the BBD's breathing noise floor and its soft attacks. Below kBbdFloor the compressor's gain stops rising,
//   so a dying tail fades out faster (downward expansion);
// - anti-alias and reconstruction lowpasses: 4-pole Butterworth each at toneHz, so the repeats darken pass by pass;
// - the clock: the chip samples at stages x clock^amount / (2 x the delay in samples) of fs (at most fs), the delay
//   set by SetDelay at the control rate. Long delays and color lower it below the filters and the repeats alias: the
//   sample degradation. Each sample is taken at its exact clock instant (linear interpolation between the two fs
//   samples around it) and the output joins the samples with straight lines, one clock period late, read at the fs
//   instants: the 48 kHz grid adds no timing jitter of its own;
// - clock LFO (BbdLfo, driven by delay.h): a triangle swinging the clock by +-depth, so the delay by d / (1 +- depth),
//   which bends the pitch; independent of amount. It runs for every type; Delay fades its depth to 0 off BBD;
// - overload: a soft clipper driven by kBbdDriveDb + driveDb x amount, undone after it (small-signal gain 1);
// - noise: uniform noise at noiseDb x (kBbdNoiseBase + (1 - kBbdNoiseBase) x amount) RMS inside the compander (so the
//   expander makes it breathe with the signal), gated like the tape hiss by a NoiseGate on the input: exactly 0 once
//   the input has stayed below kNoiseGateFloor for the gate's release.
// Amount 0 (knob 6 fully counter-clockwise) keeps every part at its base: the BBD still sounds like one. Every random
// draw is counted per sample, so the output does not depend on the caller's block size. No libDaisy, no allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../memory.h"
#include "blocks.h"

namespace btdsp {

constexpr float kBbdRef = 0.1f;         // the compander's unity level (-20 dBFS, about guitar level)
constexpr float kBbdFloor = 1e-3f;      // compressed-domain envelope below which the compressor's gain stops rising
constexpr float kBbdAttackS = 0.003f;   // the compander envelopes
constexpr float kBbdReleaseS = 0.03f;
constexpr float kBbdDriveDb = 3.0f;     // the chip's overload at amount 0
constexpr float kBbdNoiseBase = 0.1f;   // the noise at amount 0, relative to noiseDb
constexpr uint32_t kBbdSeed = 0x6A09E667u;
constexpr float kBbdModMaxRate = 0.25f;  // the clock LFO's |d offset / dn| limit: read speed 0.75..1.25
constexpr float kBbdModOff = 1e-7f;      // smoothed depths below this send the LFO's offset back to 0

struct BbdSettings {
    float amount = 0.0f;      // 0..1 (knob 6)
    float stages = 16384.0f;  // the rest: defaults = the params.h defaults
    float toneHz = 4000.0f;
    float driveDb = 12.0f;    // added to kBbdDriveDb at amount 1
    float noiseDb = -60.0f;   // at amount 1
    float clock = 0.25f;      // clock multiplier at amount 1
    bool operator==(const BbdSettings& o) const {
        return amount == o.amount && stages == o.stages && toneHz == o.toneHz && driveDb == o.driveDb &&
               noiseDb == o.noiseDb && clock == o.clock;
    }
};

// The clock LFO's settings; defaults = the params.h defaults.
struct BbdModSettings {
    float rateHz = 0.2f;   // triangle rate
    float depth = 0.005f;  // clock swing as a fraction of the clock, 0..0.25
};

// The BBD's clock LFO (Deluxe Memory Man style, with a continuous rate): a triangle tri in [-1, 1] sets the clock to
// (1 + depth x tri) of its set value, so a head d samples back reads d / (1 + depth x tri): Process returns that minus
// d. The offset moves at most kBbdModMaxRate per sample (read speed 0.75..1.25 whatever the settings). The phase
// advances every sample, so the output does not depend on the caller's block size. No libDaisy, no allocation.
class BbdLfo {
public:
    void Init(float sampleRate) {
        invFs_ = 1.0f / sampleRate;
        inc_ = depth_ = 0.0f;
        Reset();
    }
    // The phase at the triangle's zero crossing (offset 0), the offset at 0.
    void Reset() {
        phase_ = 0.25f;
        off_ = 0.0f;
    }
    // Control rate. depth: the smoothed swing; below kBbdModOff the offset returns to exactly 0.
    BT_ITCM void Set(float rateHz, float depth) {
        inc_ = rateHz * invFs_;
        depth_ = depth < kBbdModOff ? 0.0f : depth;
    }
    // Per sample: the read-position offset in samples for a head d samples back.
    BT_ITCM float Process(float d) {
        phase_ += inc_;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        if (depth_ == 0.0f && off_ == 0.0f) return 0.0f;
        float target = 0.0f;
        if (depth_ > 0.0f) {
            const float m = depth_ * (4.0f * std::fabs(phase_ - 0.5f) - 1.0f);
            target = -d * m / (1.0f + m);
        }
        off_ += std::clamp(target - off_, -kBbdModMaxRate, kBbdModMaxRate);
        return off_;
    }
    float Offset() const { return off_; }

private:
    float invFs_ = 0.0f, inc_ = 0.0f, phase_ = 0.25f, depth_ = 0.0f, off_ = 0.0f;  // invFs_: Init
};

class Bbd {
public:
    void Init(float sampleRate) {
        fs_ = sampleRate;
        atk_ = TimeCoef(kBbdAttackS, fs_);
        rel_ = TimeCoef(kBbdReleaseS, fs_);
        gate_.Init(fs_);
        rng_ = XorShift32{kBbdSeed};
        d_ = 2.0f;
        last_.amount = -1.0f;
        last_.toneHz = -1.0f;
        Reset();
        Set(BbdSettings{});
    }

    // Clears the signal state (envelopes, filters, the held sample); keeps the settings.
    void Reset() {
        envC_ = envE_ = h0_ = h1_ = prev_ = phase_ = 0.0f;
        gate_.Clear();
        pre_.Clear();
        post_.Clear();
    }

    // Control rate; returns early when nothing changed.
    BT_ITCM void Set(const BbdSettings& s) {
        if (s == last_) return;
        if (s.toneHz != last_.toneHz) {
            const float f = std::min(s.toneHz / fs_, kMaxCutoff);
            pre_.Set(f);
            post_.Set(f);
        }
        last_ = s;
        const float a = std::clamp(s.amount, 0.0f, 1.0f);
        stages_ = s.stages * std::pow(s.clock, a);
        drive_ = DbToGain(kBbdDriveDb + s.driveDb * a);
        invDrive_ = 1.0f / drive_;
        noiseAmp_ = DbToGain(s.noiseDb) * (kBbdNoiseBase + (1.0f - kBbdNoiseBase) * a) * kUniformPeakPerRms;
        SetDelay(d_);
    }

    // Control rate: the delay in samples, which sets the clock.
    BT_ITCM void SetDelay(float d) {
        d_ = std::max(d, 1.0f);
        inc_ = std::min(1.0f, stages_ / (2.0f * d_));
    }

    // Per sample, in the loop or on the input.
    BT_ITCM float Process(float v) {
        gate_.Follow(v);
        const float g = kBbdRef / std::max(envC_, kBbdFloor);
        const float s = SoftClip(drive_ * pre_.Process(v * g)) * invDrive_;
        envC_ = Follow(envC_, s);
        phase_ += inc_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
            h1_ = h0_;
            h0_ = s + (phase_ / inc_) * (prev_ - s);  // the clock ticked phase_ / inc_ samples ago
        }
        prev_ = s;
        float n = h1_ + phase_ * (h0_ - h1_);  // one clock period ago, between the last two samples
        if (gate_.Open()) n += noiseAmp_ * gate_.Level() * rng_.Bipolar();
        const float p = post_.Process(n);
        envE_ = Follow(envE_, p);
        return p * envE_ * (1.0f / kBbdRef);
    }

private:
    BT_ITCM float Follow(float e, float x) const {
        const float r = std::fabs(x);
        return Flush(e + (r > e ? atk_ : rel_) * (r - e));
    }

    float fs_ = 0.0f;  // Init
    BbdSettings last_;
    float atk_ = 1.0f, rel_ = 1.0f;
    float stages_ = 16384.0f, d_ = 2.0f, inc_ = 1.0f, drive_ = 1.0f, invDrive_ = 1.0f, noiseAmp_ = 0.0f;
    float envC_ = 0.0f, envE_ = 0.0f, h0_ = 0.0f, h1_ = 0.0f, prev_ = 0.0f, phase_ = 0.0f;
    NoiseGate gate_;
    XorShift32 rng_{kBbdSeed};
    Butterworth4 pre_, post_;  // anti-alias, reconstruction
};

}  // namespace btdsp
