// SW2 middle: the delay's tape color. One value, age (0 new .. 1 worn: knob 6 from fully counter-clockwise to fully
// clockwise), scales every part in from nothing; each setting is its effect at age 1:
// - wow (kWowHz) and flutter (kFlutterHz): two sines on the read position whose rates and depths drift to a new
//   random draw every cycle (ReadOffset, interpolated linearly between control ticks);
// - saturation (Record, in the loop): a soft clipper driven by driveDb, the negative half harder (kTapeAsym) so it
//   turns asymmetric with age, undone after it so it only ever compresses (|out| <= |in|, slope 1 at 0);
// - HF loss (Record, in the loop): a Butterworth lowpass falling log-wise from kMaxCutoff * fs to toneHz;
// - LF bump (Playback, output only): a peak of bumpDb at kBumpHz;
// - hiss (Playback, output only): uniform noise at hissDb RMS, gated by a NoiseGate on the wet so it falls with the
//   repeats and is exactly 0 once they are gone;
// - dropouts (Dropout, applied to the wet before it feeds back): above kDropoutAge, random 20..80 ms dips to
//   10..50 % gain, dropoutsPerMin at age 1.
// Every random draw is counted per sample (Tick at the control rate, Playback per sample), so the output does not
// depend on the caller's block size. colorShare scales every part except wow and flutter: the age driving them is
// age x colorShare (delay.h moves them to the chain's input with delay_tape_scope all). At or below kTapeOff the
// stage passes its input bit-exactly (Record and Playback while age x colorShare <= kTapeOff) and runs no filters;
// switching on starts the lowpass and the bump at their steady state for the current input (at an age near 0, where
// the lowpass is near kMaxCutoff and the bump near unity), so it does not click. No libDaisy, no allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../memory.h"
#include "blocks.h"

namespace btdsp {

constexpr float kTapeOff = 1e-4f;  // ages at or below this bypass the stage
constexpr float kWowHz[2] = {0.5f, 2.0f};
constexpr float kFlutterHz[2] = {6.0f, 12.0f};
constexpr float kWobbleDepthMin = 0.6f;  // each cycle's depth multiplier is drawn from [0.6, 1]
constexpr float kTapeDriftS = 0.5f;      // smoothing of the drawn rates and depths
constexpr float kTapeAsym = 1.0f;        // the negative half is driven (1 + kTapeAsym * age) x harder
constexpr float kBumpHz = 100.0f;        // LF bump centre, Q 1
constexpr float kDropoutAge = 0.6f;      // no dropouts below this age; the rate rises linearly to age 1
constexpr float kDropoutMs[2] = {20.0f, 80.0f};
constexpr float kDropoutGain[2] = {0.1f, 0.5f};
constexpr float kDropoutSmoothS = 0.005f;
constexpr uint32_t kTapeSeed = 0x9E3779B9u;
constexpr uint32_t kTapeInputSeed = 0xBB67AE85u;  // the input's tape (delay.h ColorInput), independent of the loop's

struct TapeSettings {
    float age = 0.0f;           // 0..1
    float wowMs = 2.0f;         // the rest at age 1; defaults = the params.h defaults
    float flutterMs = 0.1f;
    float driveDb = 9.0f;
    float toneHz = 2500.0f;
    float bumpDb = 3.0f;
    float hissDb = -60.0f;
    float dropoutsPerMin = 6.0f;
    float colorShare = 1.0f;  // 0..1: the share of age driving every part but wow and flutter (delay_tape_scope)
    bool operator==(const TapeSettings& o) const {
        return age == o.age && wowMs == o.wowMs && flutterMs == o.flutterMs && driveDb == o.driveDb &&
               toneHz == o.toneHz && bumpDb == o.bumpDb && hissDb == o.hissDb && dropoutsPerMin == o.dropoutsPerMin &&
               colorShare == o.colorShare;
    }
};

class Tape {
public:
    // Tick is called once every kControlInterval samples.
    void Init(float sampleRate, uint32_t seed = kTapeSeed) {
        fs_ = sampleRate;
        tickS_ = float(kControlInterval);
        tickRate_ = fs_ / tickS_;
        rng_ = XorShift32{seed};
        lp_ = bump_ = Svf{};
        bump_.Set(kBumpHz / fs_, 1.0f);
        lp_.Set(kMaxCutoff, kButterworth2K);
        wow_ = {0.0f, 1.0f, 1.0f, 0.8f, 0.8f};
        flut_ = {0.0f, 8.0f, 8.0f, 0.8f, 0.8f};
        driftA_ = TimeCoef(kTapeDriftS, tickRate_);
        mod_ = span_ = TickRamp{};
        gate_.Init(fs_);
        drop_.Init(fs_, kDropoutSmoothS, 1.0f);
        dropT_ = 1.0f;
        dropLeft_ = 0;
        drive_ = invDrive_ = driveNeg_ = invDriveNeg_ = 1.0f;
        blend_ = bumpGm1_ = hissAmp_ = dropP_ = wowS_ = flutS_ = 0.0f;
        active_ = colorOn_ = false;
        recCold_ = playCold_ = true;
        last_.age = -1.0f;
        Set(TapeSettings{});
    }

    // Control rate; returns early when nothing changed.
    BT_ITCM void Set(const TapeSettings& s) {
        if (s == last_) return;
        last_ = s;
        const float a = std::clamp(s.age, 0.0f, 1.0f);
        if (a <= kTapeOff) {
            // Skipping the coefficient math keeps a decaying smoothed age cheap.
            active_ = false;
            StopColor();
            return;
        }
        active_ = true;
        wowS_ = a * s.wowMs * fs_ / 1000.0f;
        flutS_ = a * s.flutterMs * fs_ / 1000.0f;
        const float c = a * std::clamp(s.colorShare, 0.0f, 1.0f);
        if (c <= kTapeOff) {
            StopColor();
            return;
        }
        colorOn_ = true;
        drive_ = DbToGain(s.driveDb * c);
        invDrive_ = 1.0f / drive_;
        driveNeg_ = drive_ * (1.0f + kTapeAsym * c);
        invDriveNeg_ = 1.0f / driveNeg_;
        blend_ = c;
        lp_.Set(kMaxCutoff * std::pow((s.toneHz / fs_) / kMaxCutoff, c), kButterworth2K);
        bumpGm1_ = DbToGain(s.bumpDb * c) - 1.0f;
        hissAmp_ = c * DbToGain(s.hissDb) * kUniformPeakPerRms;
        dropP_ = s.dropoutsPerMin / 60.0f * std::clamp((c - kDropoutAge) / (1.0f - kDropoutAge), 0.0f, 1.0f) * tickS_ /
                 fs_;
    }

    BT_ITCM bool Active() const { return active_; }  // wow and flutter live

    // Control rate, after Set.
    BT_ITCM void Tick() {
        Advance(wow_, kWowHz);
        Advance(flut_, kFlutterHz);
        const float next = wowS_ * wow_.depth * std::sin(kTwoPi * wow_.phase) +
                           flutS_ * flut_.depth * std::sin(kTwoPi * flut_.phase);
        mod_.Set(next);
        span_.Set(wowS_ + flutS_);
        if (dropLeft_ > 0) {
            if (--dropLeft_ == 0) dropT_ = 1.0f;
        } else if (dropP_ > 0.0f && rng_.Uniform() < dropP_) {
            const float ms = kDropoutMs[0] + (kDropoutMs[1] - kDropoutMs[0]) * rng_.Uniform();
            dropLeft_ = std::max(1L, std::lround(ms * 1e-3f * tickRate_));
            dropT_ = kDropoutGain[0] + (kDropoutGain[1] - kDropoutGain[0]) * rng_.Uniform();
        }
    }

    // Per sample: the read-position offset in samples, |offset| <= (wowMs + flutterMs) x age x fs / 1000.
    BT_ITCM float ReadOffset() { return mod_.Step(); }

    // Per sample, after ReadOffset: the bound of the offset's magnitude, (wowMs + flutterMs) x age x fs / 1000, ramped
    // across each tick like the offset, so |ReadOffset()| <= WobbleSpan() and a moving age or depth glides the pitch.
    BT_ITCM float WobbleSpan() { return span_.Step(); }

    // Per sample: the playback gain (1 outside a dropout).
    BT_ITCM float Dropout() { return drop_.Process(dropT_); }

    // Per sample, in the loop: saturation and HF loss. Never expands and the lowpass never exceeds unity, so the loop
    // gain never exceeds the feedback.
    BT_ITCM float Record(float v) {
        if (!colorOn_) {
            recCold_ = true;
            return v;
        }
        if (recCold_) {
            lp_.Warm(v);
            recCold_ = false;
        }
        const float sat = v >= 0.0f ? SoftClip(drive_ * v) * invDrive_ : SoftClip(driveNeg_ * v) * invDriveNeg_;
        return lp_.Process(v + blend_ * (sat - v));
    }

    // Per sample, on the output only (never fed back): LF bump and gated hiss.
    BT_ITCM float Playback(float w) {
        if (!colorOn_) {
            playCold_ = true;
            return w;
        }
        if (playCold_) {
            bump_.Warm(w);
            gate_.Clear();
            playCold_ = false;
        }
        float band;
        bump_.Process(w, band);
        gate_.Follow(w);
        float y = w + bumpGm1_ * band;  // k = 1: band is unity at kBumpHz, so the peak boost is exactly the gain
        if (gate_.Open()) y += hissAmp_ * gate_.Level() * rng_.Bipolar();
        return y;
    }

private:
    struct Wobble {
        float phase, rate, rateT, depth, depthT;
    };

    // The color off: Record and Playback pass their input, no dropout runs or starts (wow and flutter stay as set).
    BT_ITCM void StopColor() {
        colorOn_ = false;
        dropP_ = 0.0f;
        dropT_ = 1.0f;
        dropLeft_ = 0;
    }

    BT_ITCM void Advance(Wobble& o, const float (&hz)[2]) {
        o.phase += o.rate / tickRate_;
        if (o.phase >= 1.0f) {
            o.phase -= 1.0f;
            o.rateT = hz[0] + (hz[1] - hz[0]) * rng_.Uniform();
            o.depthT = kWobbleDepthMin + (1.0f - kWobbleDepthMin) * rng_.Uniform();
        }
        o.rate += driftA_ * (o.rateT - o.rate);
        o.depth += driftA_ * (o.depthT - o.depth);
    }

    float fs_ = 0.0f, tickS_ = 0.0f, tickRate_ = 0.0f;  // Init
    TapeSettings last_;
    bool active_ = false, colorOn_ = false;
    bool recCold_ = true, playCold_ = true;  // Record's or Playback's filters stopped while the color was off
    XorShift32 rng_{kTapeSeed};
    Wobble wow_{}, flut_{};
    float driftA_ = 0.0f;
    TickRamp mod_, span_;  // the read offset and its bound, ramped across each tick
    float wowS_ = 0.0f, flutS_ = 0.0f;
    float drive_ = 1.0f, invDrive_ = 1.0f, driveNeg_ = 1.0f, invDriveNeg_ = 1.0f, blend_ = 0.0f;
    float bumpGm1_ = 0.0f, hissAmp_ = 0.0f;
    NoiseGate gate_;
    float dropP_ = 0.0f, dropT_ = 1.0f;
    long dropLeft_ = 0;
    Smoother drop_;
    Svf lp_, bump_;
};

}  // namespace btdsp
