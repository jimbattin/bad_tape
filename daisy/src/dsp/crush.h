// SW2 up: the digital delay's bitcrush color.
// Per sample, in the feedback loop or (delay_crush_scope all, delay.h) on the chain's input: anti-alias lowpass ->
// sample-and-hold -> drive into a soft clipper (or wrap) -> quantiser -> makeup -> 4-pole anti-imaging lowpass
// (Process). After it, never fed back: bit inversion (InvertBits), on the wet output or on the crushed input. Amount
// 0..1 (knob 6, fully counter-clockwise to fully clockwise):
// - hold rate: falls exponentially from fs to rateHz (delay_crush_rate);
// - bit depth: steps 12 -> 8 -> 4 at the amounts in kCrushBitsAmount, with kCrushBitsHyst of hysteresis so a knob
//   resting on a boundary does not chatter;
// - drive: driveDb x amount into the soft clipper ahead of the quantiser, undone after it, so a guitar-level signal
//   fills the coarse grid (grit and body rather than hiss) while the small-signal gain stays 1;
// - the anti-alias cutoff (aaRatio) and the anti-imaging cutoff (toneRatio) are fractions of the hold rate. aaRatio
//   above 0.5 lets aliasing through (crunch); the anti-imaging filter removes the hold's staircase images (fizz).
// The options (0 off .. 1 on, smoothed by the caller, so switching one fades it):
// - open: both filters open geometrically toward kMaxCutoff with open x amount, so fully clockwise the hold is bare:
//   no anti-aliasing (notes above half the hold rate fold down into inharmonic tones) and the staircase images left in;
// - clock: rateHz becomes the hold rate at a kCrushClockRefS delay, scaled by kCrushClockRefS / the delay time (a
//   sampler that sets its delay time with its sample clock), so longer delays alias more and a time change sweeps it;
// - wrap: the driven signal wraps around to the opposite rail past +-1 (two's-complement overflow) instead of
//   clipping, crossfaded by wrap;
// - bits: bit inversion. The wet is read as an 8-bit offset-binary word (0x80 = 0), the bits set in mask (bit 7 = the
//   MSB) are inverted, and the result is mixed in by bits x amount x (env / kInvertFull)^2 (at most 1), env a peak
//   follower on the wet (instant attack, kInvertReleaseS release): loud repeats are inverted fully, quieter ones less,
//   so the inverted bits fall away with the repeats. The word is mid-rise, so the inversion is odd-symmetric and adds
//   no DC; below kInvertDead (half an 8-bit step) the wet is silence and passes as is, so the output is exactly the
//   wet once the repeats are gone. A mask change crossfades over kCrushMaskFadeS.
// At or below kCrushOff the stage passes its input bit-exactly and runs no filters; switching on starts both filters at
// their steady state for the current input (it switches on at an amount near 0, where the cutoffs are near kMaxCutoff
// and settle within a few samples), so it does not click. No libDaisy, no allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../memory.h"
#include "blocks.h"

namespace btdsp {

constexpr float kCrushOff = 1e-4f;  // amounts at or below this bypass the stage
constexpr int kCrushBits[3] = {12, 8, 4};
// The amounts (knob 6 travel) where the depth steps to 8 and to 4 bits.
constexpr float kCrushBitsAmount[2] = {0.45f, 0.78f};
constexpr float kCrushBitsHyst = 0.01f;  // in knob travel, each side of a step
constexpr float kCrushClockRefS = 1.0f;  // clock on: the delay time at which the hold rate is rateHz
constexpr float kCrushMaskFadeS = 0.02f;
constexpr float kInvertFull = 0.1f;          // -20 dBFS: bit inversion at full depth from this wet level up
constexpr float kInvertReleaseS = 0.05f;
constexpr float kInvertDead = 1.0f / 256.0f;  // |wet| below this is silence

struct CrushSettings {
    float amount = 0.0f;      // 0..1
    float driveDb = 0.0f;     // drive at amount 1
    float rateHz = 4000.0f;   // hold rate at amount 1
    float aaRatio = 0.5f;     // anti-alias cutoff / hold rate
    float toneRatio = 0.5f;   // anti-imaging cutoff / hold rate
    float open = 0.0f;        // the options, 0..1
    float clock = 0.0f;
    float wrap = 0.0f;
    float bits = 0.0f;
    float delayS = kCrushClockRefS;  // the delay time, s (clock)
    int mask = 0;                    // the bit inversion mask, 0..255
    bool operator==(const CrushSettings& o) const {
        return amount == o.amount && driveDb == o.driveDb && rateHz == o.rateHz && aaRatio == o.aaRatio &&
               toneRatio == o.toneRatio && open == o.open && clock == o.clock && wrap == o.wrap && bits == o.bits &&
               delayS == o.delayS && mask == o.mask;
    }
};

// v as an 8-bit offset-binary word with the bits of mask inverted, back to -1..1. Mid-rise: word u stands for
// (u - 127.5) / 128, so inverting a bit maps -v to minus the image of v.
BT_ITCM inline float InvertedWord(float v, uint32_t mask) {
    const float u = std::clamp(std::floor(v * 128.0f + 128.0f), 0.0f, 255.0f);
    return (float(uint32_t(u) ^ mask) - 127.5f) * (1.0f / 128.0f);
}

// x wrapped into [-1, 1): two's-complement overflow.
BT_ITCM inline float Wrap(float x) { return x - 2.0f * std::floor(0.5f * (x + 1.0f)); }

class Crush {
public:
    void Init(float sampleRate) {
        fs_ = sampleRate;
        pre_ = Svf{};
        post_ = Butterworth4{};
        hold_ = phase_ = 0.0f;
        cold_ = true;
        step_ = 0;
        env_ = 0.0f;
        rel_ = std::exp(-1.0f / (kInvertReleaseS * fs_));
        fadeLen_ = int(std::lround(kCrushMaskFadeS * fs_));
        last_.amount = -1.0f;
        Set(CrushSettings{});
        SnapMask();
    }

    // The mask set by the last Set takes effect at once, without the crossfade.
    void SnapMask() {
        mask_ = prevMask_ = maskT_;
        fade_ = Crossfade{};
    }

    // Control rate; returns early when nothing changed. The bit step moves with hysteresis from where it is.
    BT_ITCM void Set(const CrushSettings& in) {
        if (in == last_) return;
        last_ = in;
        const float a = std::clamp(in.amount, 0.0f, 1.0f);
        active_ = a > kCrushOff;
        for (;;) {
            if (step_ < 2 && a >= kCrushBitsAmount[step_] + kCrushBitsHyst) ++step_;
            else if (step_ > 0 && a < kCrushBitsAmount[step_ - 1] - kCrushBitsHyst) --step_;
            else break;
        }
        q_ = std::exp2(1.0f - float(kCrushBits[step_]));
        invQ_ = 1.0f / q_;
        drive_ = DbToGain(in.driveDb * a);
        invDrive_ = 1.0f / drive_;
        const float rate = in.rateHz * std::pow(kCrushClockRefS / std::max(in.delayS, 1e-3f), in.clock);
        inc_ = std::min(1.0f, std::pow(rate / fs_, a));
        const float open = in.open * a;
        pre_.Set(Opened(in.aaRatio * inc_, open), kButterworth2K);
        post_.Set(Opened(in.toneRatio * inc_, open));
        wrap_ = in.wrap;
        bits_ = in.bits * a;
        maskT_ = uint32_t(in.mask) & 0xFFu;
    }

    // The current bit depth (tests: the step hysteresis).
    int Bits() const { return kCrushBits[step_]; }
    // Whether the amount is above kCrushOff (below it Process passes its input bit-exactly).
    BT_ITCM bool Active() const { return active_; }

    // In the loop, or on the input (delay_crush_scope all).
    BT_ITCM float Process(float v) {
        if (!active_) {
            cold_ = true;
            hold_ = v;
            phase_ = 0.0f;
            return v;
        }
        if (cold_) {
            pre_.Warm(v);
            post_.Warm(v);
            cold_ = false;
        }
        const float lp = pre_.Process(v);
        phase_ += inc_;
        if (phase_ >= 1.0f) {
            phase_ -= 1.0f;
            const float x = drive_ * lp;
            float s = SoftClip(x);
            if (wrap_ > 0.0f) s += wrap_ * (Wrap(x) - s);
            hold_ = std::floor(s * invQ_ + 0.5f) * q_ * invDrive_;
        }
        return post_.Process(hold_);
    }

    // After Process, never fed back: bit inversion.
    BT_ITCM float InvertBits(float w) {
        const float a = std::fabs(w);
        env_ = std::max(a, Flush(env_ * rel_));
        if (!(bits_ > 0.0f)) return w;
        if (!fade_.on && maskT_ != mask_) {
            prevMask_ = mask_;
            mask_ = maskT_;
            fade_.Start(fadeLen_);
        }
        float x = 1.0f;  // mask_'s weight against prevMask_
        if (fade_.on) {
            x = fade_.Weight();
            fade_.Advance();
        }
        if (a < kInvertDead) return w;
        float b = InvertedWord(w, mask_);
        if (x < 1.0f) {
            const float b0 = InvertedWord(w, prevMask_);
            b = b0 + x * (b - b0);
        }
        const float l = std::min(1.0f, env_ * (1.0f / kInvertFull));
        return w + bits_ * l * l * (b - w);
    }

private:
    // Cutoff f (a fraction of fs, capped at kMaxCutoff) opened geometrically toward kMaxCutoff by open 0..1.
    BT_ITCM static float Opened(float f, float open) {
        f = std::min(f, kMaxCutoff);
        return open > 0.0f ? f * std::pow(kMaxCutoff / f, open) : f;
    }

    float fs_ = 0.0f;  // Init
    CrushSettings last_;
    bool active_ = false;
    int step_ = 0;  // index into kCrushBits
    float inc_ = 1.0f, q_ = 0.0f, invQ_ = 0.0f, drive_ = 1.0f, invDrive_ = 1.0f;
    float wrap_ = 0.0f, bits_ = 0.0f;
    Svf pre_;            // anti-alias
    Butterworth4 post_;  // anti-imaging
    float hold_ = 0.0f, phase_ = 0.0f;
    bool cold_ = true;  // the filters stopped while inactive: Process warms them on the next active sample
    // Bit inversion.
    float env_ = 0.0f, rel_ = 0.0f;  // the wet's peak follower and its release coefficient
    uint32_t mask_ = 0, prevMask_ = 0, maskT_ = 0;  // playing, fading out, wanted
    Crossfade fade_;                                // prevMask_ -> mask_
    int fadeLen_ = 1;
};

}  // namespace btdsp
