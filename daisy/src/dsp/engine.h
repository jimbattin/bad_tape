// The DSP chain (daisy/README.md "Signal flow"): EQ -> delay -> reverb on the left input, or delay -> reverb -> EQ.
// No libDaisy, no allocation: the same code runs in the firmware's audio callback (daisy/src/bad_tape.cpp), the bench
// (daisy/src/bench.cpp), the host render (host/render.cpp) and under the tests.
//
// EQ (eq.h, off by default): eq_position puts it pre, where it feeds the delay, both dry paths and the reverb, or post,
// where it takes the mixed output (tails included). A placement change fades the EQ out, moves it on a chunk boundary
// and fades it back in.
//
// Delay (delay.h): delay_type (SW2) picks its character and delay_color (knob 6, 0 none .. 100 % full) colors it:
// BBD (bbd.h: compander, clocked sample-and-hold, filters, overload, noise; its base character stays at 0; and a
// triangle clock LFO (delay_bbd_mod_rate, delay_bbd_mod_depth) that bends the pitch regardless of color), tape (tape.h:
// wow and flutter, saturation, HF loss, LF bump, gated hiss, dropouts) or digital (crush.h: bitcrush). Each color
// sits in the feedback loop with its scope (delay_crush_scope, delay_tape_scope, delay_bbd_scope) at repeats, or on
// the chain's input after a pre EQ with all (the default: Delay::ColorInput), where the dry, the delay and every reverb
// input take it; tape wow and flutter and the BBD clock LFO stay on the repeats. Tape and BBD glide the pitch on
// a time change; digital crossfades. delay_mode reverse plays every repeat backwards in windows as long as the delay
// time; forward and reverse crossfade at a switch. delay_mode off stops recording new input while the repeats ring out
// in the last direction (the reverb keeps its input).
//
// Mixes: the delay mixed by delay_mix, then the reverb SW3 selects (reverb_algo: plate.h, hall.h or canyon.h) mixed by
// reverb_mix. Both are equal-power: sqrt(1 - t) dry + sqrt(t) wet, t = delay_mix for knob 1 and t = reverb_mix ^
// kReverbMixExp for knob 2 (dry and wet equal at reverb_mix 75 %); the gains (not t) are smoothed; 0 % and 100 % target
// exactly 1 and 0. The reverb runs at kReverbRate (half the chain's rate) inside a half-band decimator and interpolator
// (halfband.h HalfRateReverb); its wet is kReverbFold * (L + R) back at the chain's rate.
//
// Routing (MIDI only) selects the reverb's input with a kRouteRampS crossfade: the delay's 100 % wet, the delay mix
// (default) or the dry input (through the EQ when pre, and the input colors).
//
// SW3 switch: the three algorithms share one pool (reverb_pool.h), which the engine owns: it zeroes it at Init, and an
// SW3 change ramps the reverb's input and wet down over kAlgoFadeSamples, zeroes the pool in slices of
// kPoolClearPerSample floats per sample (kAlgoClearSamples, the wet silent; the old tail is lost), swaps the algorithm
// (Reset, Snap) and ramps both back up (the new algorithm's input starts from 0, so a running signal does not enter it
// as a step). Returning to the running algorithm during the fade-out ramps straight back up with no clear; a change
// during the clear is honoured when it ends. Every algorithm receives its parameters all the time, so the one switched
// in starts at its current settings.
//
// Bypass passes the raw input (never equalised or crushed) at unity, stops recording, mutes the reverb's input and
// lets the repeats and the reverb tail spill over (dry + sqrt(mix) * delay wet + the knob 2 wet gain * reverb wet); the
// tails pass the EQ when it is post; pre, they were equalised on the way in.
//
// Priming: parameters set between Init and the first Process apply without smoothing (reverb_algo too: no fade, no
// clear).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../controls.h"
#include "../memory.h"
#include "../params.h"
#include "delay.h"
#include "eq.h"
#include "halfband.h"
#include "reverb_pool.h"

namespace btdsp {

constexpr float kReverbMixExp = 2.4094208f;  // ln 0.5 / ln 0.75: reverb_mix 75 % gives t = 0.5

class Engine {
public:
    static constexpr int kAlgoFadeSamples = int(0.02f * kSampleRate);  // SW3: the wet's ramp down and back up (20 ms)
    static constexpr size_t kPoolClearPerSample = 64;  // SW3: pool floats zeroed per sample of the clear
    // SW3: the clear's length, ceil((58 076 + 33 604) / kPoolClearPerSample) = 1 433 samples (~15 ms at 96 kHz).
    static constexpr int kAlgoClearSamples =
        int((kReverbPoolFloats + kPoolClearPerSample - 1) / kPoolClearPerSample);

    // Every parameter at its table default. delayMem: btdsp::kDelaySamples floats; reverbMem: the reverb pool
    // (reverb_pool.h); both owned by the caller. The reverbs run at sampleRate / 2.
    void Init(float sampleRate, float* delayMem, const ReverbPool& reverbMem) {
        fs_ = sampleRate;
        for (const btparam::ParamDef& d : btparam::kParams) param_[d.id] = d.def;
        delay_.Init(sampleRate, delayMem);
        plate_.Init(sampleRate * 0.5f, reverbMem);
        hall_.Init(sampleRate * 0.5f, reverbMem);
        canyon_.Init(sampleRate * 0.5f, reverbMem);
        rs_.Reset();
        eq_.Init(sampleRate);
        ClearPool(reverbMem, 0, kReverbPoolFloats);
        pool_ = reverbMem;
        active_ = btctl::Algo::kHall;
        reverb_ = &hall_;
        fade_ = kAlgoFadeSamples;
        clearPos_ = -1;
        primed_ = false;
        reverse_ = false;
    }
    // btparam::Clamp()ed; ids >= kParamCount are ignored.
    void SetParam(uint8_t id, float v) {
        using btparam::Group;
        if (id >= btparam::kParamCount) return;
        param_[id] = btparam::Clamp(btparam::kParams[id], v);
        const Group g = btparam::kParams[id].group;
        if (g == Group::kHall) hall_.SetParam(id, param_[id]);
        else if (g == Group::kPlate) plate_.SetParam(id, param_[id]);
        else if (g == Group::kCanyon) canyon_.SetParam(id, param_[id]);
        else if (g == Group::kEq) eq_.SetParam(id, param_[id]);
    }
    float Param(uint8_t id) const { return id < btparam::kParamCount ? param_[id] : 0.0f; }

    // Any n; in == out allowed.
    BT_ITCM void Process(const float* in, float* out, size_t n) {
        UpdateTargets();
        if (!primed_) Prime();
        for (size_t i0 = 0, k = 0; i0 < n; i0 += k) {
            k = BeginChunk(std::min(kChunk, n - i0));
            const bool clearing = clearPos_ >= 0;
            const bool fadingOut = !clearing && want_ != active_;
            const bool eqPost = eq_.Post();
            constexpr float kFade = float(kAlgoFadeSamples);
            float xs[kChunk], dm[kChunk], mw[kChunk], bs[kChunk], sg[kChunk], rin[kChunk], wet[kChunk];
            for (size_t i = 0; i < k; ++i) {  // in is copied here, so in == out is fine
                const float raw = in[i0 + i];
                const float x = delay_.ColorInput(eqPost ? raw : eq_.Process(raw));
                const float w = delay_.Process(x);
                const float gw = delayMix_.wet.Process(delayMix_.wetT);
                const float d = delayMix_.dry.Process(delayMix_.dryT) * x + gw * w;  // the delay mix
                const float b = by_.Process(byT_);
                // The SW3 switch gain, on the reverb's input (the new algorithm's input starts from 0, not with a
                // step) and on its wet.
                float g = 1.0f;
                if (clearing) g = 0.0f;
                else if (fadingOut) g = float(--fade_) / kFade;
                else if (fade_ < kAlgoFadeSamples) g = float(++fade_) / kFade;
                xs[i] = raw;
                dm[i] = d;
                mw[i] = gw * w;
                bs[i] = b;
                sg[i] = g;
                const float src =
                    toWet_.Process(toWetT_) * w + toMix_.Process(toMixT_) * d + toDry_.Process(toDryT_) * x;
                rin[i] = (1.0f - b) * src * g;  // bypass mutes the reverb's input
            }
            if (clearing) {
                std::fill(wet, wet + k, 0.0f);
                ClearStep(k);
            } else {
                rs_.Process(*reverb_, rin, wet, k);
            }
            for (size_t i = 0; i < k; ++i) {
                const float rw = reverbMix_.wet.Process(reverbMix_.wetT) * wet[i] * sg[i];
                const float y = reverbMix_.dry.Process(reverbMix_.dryT) * dm[i] + rw;
                // bypass: dry at unity (never equalised or crushed) + tails
                out[i0 + i] = eqPost ? eq_.Process((1.0f - bs[i]) * y + bs[i] * (mw[i] + rw)) + bs[i] * xs[i]
                                     : (1.0f - bs[i]) * y + bs[i] * (xs[i] + mw[i] + rw);
            }
        }
    }

private:
    static constexpr size_t kChunk = 48;  // samples per reverb call
    static_assert(kChunk <= HalfRateReverb::kMaxBlock, "a chunk fits the resampler");
    static constexpr float kRouteRampS = 0.02f;  // routing crossfade (the reverb's input)

    // An equal-power mix's smoothed dry and wet gains and their targets.
    struct MixGains {
        Smoother dry, wet;
        float dryT = 1.0f, wetT = 0.0f;
        BT_ITCM void SetTarget(float t) {
            dryT = std::sqrt(1.0f - t);
            wetT = std::sqrt(t);
        }
        void Snap(float fs, float tauS) {
            dry.Init(fs, tauS, dryT);
            wet.Init(fs, tauS, wetT);
        }
    };

    // Once per Process call: the delay's targets and every target the sample loops chase, from the parameters.
    BT_ITCM void UpdateTargets() {
        using namespace btparam;
        const bool bypass = param_[kBypass] > 0.5f;
        const btctl::DelayMode mode = btctl::ToDelayMode(uint8_t(param_[kDelayMode]));
        if (mode == btctl::DelayMode::kReverse) reverse_ = true;
        else if (mode == btctl::DelayMode::kForward) reverse_ = false;  // off keeps the last direction
        DelayTargets dt;
        dt.timeMs = param_[kDelayTime];
        dt.feedback = param_[kDelayFeedback] / 100.0f;
        dt.type = btctl::ToDelayType(uint8_t(param_[kDelayType]));
        dt.color = param_[kDelayColor] / 100.0f;
        dt.crush.driveDb = param_[kDelayCrushDrive];
        dt.crush.rateHz = param_[kDelayCrushRate];
        dt.crush.aaRatio = param_[kDelayCrushAa];
        dt.crush.toneRatio = param_[kDelayCrushTone];
        dt.crush.open = param_[kDelayCrushOpen];
        dt.crush.clock = param_[kDelayCrushClock];
        dt.crush.wrap = param_[kDelayCrushWrap];
        dt.crush.bits = param_[kDelayCrushBits];
        dt.crush.mask = int(param_[kDelayCrushMask]);
        dt.crushScope = param_[kDelayCrushScope];
        dt.tapeScope = param_[kDelayTapeScope];
        dt.bbdScope = param_[kDelayBbdScope];
        dt.tape.wowMs = param_[kDelayTapeWow];
        dt.tape.flutterMs = param_[kDelayTapeFlutter];
        dt.tape.driveDb = param_[kDelayTapeDrive];
        dt.tape.toneHz = param_[kDelayTapeTone];
        dt.tape.bumpDb = param_[kDelayTapeBump];
        dt.tape.hissDb = param_[kDelayTapeHiss];
        dt.tape.dropoutsPerMin = param_[kDelayTapeDropouts];
        dt.bbd.stages = param_[kDelayBbdStages];
        dt.bbd.toneHz = param_[kDelayBbdTone];
        dt.bbd.driveDb = param_[kDelayBbdDrive];
        dt.bbd.noiseDb = param_[kDelayBbdNoise];
        dt.bbd.clock = param_[kDelayBbdClock];
        dt.bbdMod.rateHz = param_[kDelayBbdModRate];
        dt.bbdMod.depth = param_[kDelayBbdModDepth] / 100.0f;
        dt.inputOn = !bypass && mode != btctl::DelayMode::kOff;
        dt.reverse = reverse_;
        delay_.SetTargets(dt);
        delayMix_.SetTarget(param_[kDelayMix] / 100.0f);
        reverbMix_.SetTarget(std::pow(param_[kReverbMix] / 100.0f, kReverbMixExp));
        byT_ = bypass ? 1.0f : 0.0f;
        const btctl::Routing route = btctl::ToRouting(uint8_t(param_[kRouting]));
        toWetT_ = route == btctl::Routing::kWet ? 1.0f : 0.0f;
        toMixT_ = route == btctl::Routing::kMix ? 1.0f : 0.0f;
        toDryT_ = route == btctl::Routing::kDry ? 1.0f : 0.0f;
        want_ = btctl::ToAlgo(uint8_t(param_[kReverbAlgo]));
    }

    // The first Process call: every stage, smoother and ramp at its target.
    void Prime() {
        if (want_ != active_) Activate(want_);  // the pool is still zero: switch at once
        delay_.Snap();
        reverb_->Snap();
        eq_.Snap();
        delayMix_.Snap(fs_, btparam::SmoothSeconds(btparam::kDelayMix));
        reverbMix_.Snap(fs_, btparam::SmoothSeconds(btparam::kReverbMix));
        toWet_.Init(fs_, kRouteRampS, toWetT_);
        toMix_.Init(fs_, kRouteRampS, toMixT_);
        toDry_.Init(fs_, kRouteRampS, toDryT_);
        by_.Init(fs_, kEnableRampS, byT_);
        primed_ = true;
    }

    // A chunk of at most k samples starts: the SW3 clear starts once faded out, and the chunk ends where the clear,
    // the SW3 fade-out or the EQ's fade-out does (each phase ends on a chunk boundary, whatever the caller's blocks);
    // a faded-out EQ moves now. Returns the chunk's length.
    BT_ITCM size_t BeginChunk(size_t k) {
        if (clearPos_ < 0 && want_ != active_ && fade_ == 0) clearPos_ = 0;
        if (clearPos_ >= 0) {
            const size_t left = kReverbPoolFloats - size_t(clearPos_);
            k = std::min(k, (left + kPoolClearPerSample - 1) / kPoolClearPerSample);
        } else if (want_ != active_) {
            k = std::min(k, size_t(fade_));
        }
        const int mv = eq_.SamplesToMove();
        if (mv == 0) eq_.Move();
        else if (mv > 0) k = std::min(k, size_t(mv));
        return k;
    }

    // k samples of the SW3 clear: k x kPoolClearPerSample pool floats zeroed; the last slice swaps the algorithm.
    BT_ITCM void ClearStep(size_t k) {
        const size_t m = std::min(k * kPoolClearPerSample, kReverbPoolFloats - size_t(clearPos_));
        ClearPool(pool_, size_t(clearPos_), m);
        clearPos_ += long(m);
        if (size_t(clearPos_) == kReverbPoolFloats) {
            clearPos_ = -1;
            Activate(want_);
            reverb_->Snap();
        }
    }

    // Makes `a` the running algorithm, its state outside the pool and the resampler's zeroed, so it starts exactly as
    // at power-on (Reset on the concrete class: a call through reverb_ trips GCC's -Wstringop-overflow on its
    // speculative devirtualisation).
    void Activate(btctl::Algo a) {
        active_ = a;
        rs_.Reset();
        if (a == btctl::Algo::kPlate) {
            plate_.Reset();
            reverb_ = &plate_;
        } else if (a == btctl::Algo::kCanyon) {
            canyon_.Reset();
            reverb_ = &canyon_;
        } else {
            hall_.Reset();
            reverb_ = &hall_;
        }
    }

    float param_[btparam::kParamCount];
    float fs_ = 0.0f;  // Init
    bool primed_ = false;
    bool reverse_ = false;
    Delay delay_;
    Plate plate_;
    Hall hall_;
    Canyon canyon_;
    Eq eq_;
    btctl::Algo active_ = btctl::Algo::kHall;
    btctl::Algo want_ = btctl::Algo::kHall;  // reverb_algo as of UpdateTargets
    Reverb* reverb_ = nullptr;
    HalfRateReverb rs_;  // reverb_ at half the rate
    ReverbPool pool_{nullptr, nullptr};
    int fade_ = kAlgoFadeSamples;  // the SW3 switch gain x kAlgoFadeSamples
    long clearPos_ = -1;           // the next pool float the SW3 clear zeroes; -1: no clear
    MixGains delayMix_;            // knob 1
    MixGains reverbMix_;           // knob 2
    float toWetT_ = 0.0f, toMixT_ = 1.0f, toDryT_ = 0.0f, byT_ = 0.0f;
    Ramp toWet_, toMix_, toDry_;  // routing: the reverb-input gains
    Ramp by_;                     // bypass
};

}  // namespace btdsp
