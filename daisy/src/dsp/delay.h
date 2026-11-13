// The delay: one line in caller-provided memory (SDRAM on the Seed), read with
// 4-point Hermite interpolation. Its type (SW2, btctl::DelayType) sets how a time change sounds and which color knob
// 6 drives. Digital: a time change crossfades from the playing read head to a second one at the new time (kXfadeMs,
// smoothstep), so the read speed never changes and the pitch never glides; a change during a fade starts the next
// fade when this one ends. Tape and BBD: a time change glides the read head instead, overdamped over about kGlideS
// with the read speed held to 1 +- kTapeGlideMaxRate, so the pitch bends like a capstan or a BBD clock changing speed;
// the tape's wow and flutter offset the read position. The BBD's clock LFO (bbd.h BbdLfo) offsets the read position
// the same way, on the forward head and the reverse grains, with its depth faded over 150 ms when the type changes. A
// fade running when the type changes to tape or BBD completes first; a change to digital mid-glide crossfades from
// where the head is.
// The loop: input (enable ramp) + feedback * forward wet (tape dropouts) -> tape record (saturation, HF loss) -> DC
// blocker -> bitcrush (crush.h) -> BBD (bbd.h, crossfaded in and out over kXfadeMs when the type changes) -> soft
// limiter (ceiling 1.0) -> line, so feedback above 100 % self-oscillates and stays bounded. SetTargets sets the tape
// age and the crush amount to 0 unless their type is selected. The output is the wet through the crush's bit
// inversion and tape playback (LF bump, hiss), which never feed back. delay_crush_scope moves the crush: repeats keeps
// it in the loop; all sets the loop crush's amount to 0 and gives the amount to a second crush on the chain's input
// (ColorInput, with its own bit inversion); a scope change moves the amount from one to the other over its smoothing.
// delay_tape_scope and delay_bbd_scope move the tape color and the BBD stage onto the input the same way (a second
// Tape and Bbd in ColorInput). Tape wow and flutter stay on the read head as well and also wobble the input (a short
// line in ColorInput); the BBD's clock LFO stays on the read head only.
// Reverse (ReverseGrains): the heard wet crossfades between the forward and reverse reads over kXfadeMs (smoothstep);
// forward skips the grains entirely. The caller decides the direction (the engine keeps the last one while SW1 is
// off).
// Every update counts samples (per sample, or every kControlInterval samples), so the output does not depend on how the
// caller splits its blocks. No libDaisy, no allocation.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

#include "../controls.h"
#include "../memory.h"
#include "../params.h"
#include "bbd.h"
#include "blocks.h"
#include "crush.h"
#include "tape.h"

namespace btdsp {

// 2^19 floats (2 MiB, 5.46 s at 96 kHz): the 2 s head, and reverse grains reading at most 2 + 2 (kRevMaxHop +
// kRevMaxFade) < kDelaySamples back.
constexpr size_t kDelaySamples = size_t(1) << 19;
constexpr size_t kDelayMask = kDelaySamples - 1;
constexpr float kXfadeMs = 20.0f;       // read-head crossfade on a time change, and the direction crossfade
constexpr int kRevMaxFade = 2048;       // reverse: longest grain crossfade, samples (kXfadeMs: 1 920 at 96 kHz)
constexpr long kRevMaxHop = long(kDelaySamples / 2) - 2 * kRevMaxFade;  // reverse: longest window, samples
constexpr float kEnableRampS = 0.005f;  // input enable (SW1 off, bypass) ramp
constexpr float kDcHz = 10.0f;          // feedback-loop DC blocker corner
constexpr float kLimitKnee = 0.5f;      // soft limiter: unity below, ceiling 1.0
constexpr float kTapeGlideMaxRate = 0.5f;  // tape mode: |dDelay/dn| limit, read speed 0.5..1.5
// The input tape's wow line (ColorInput): delays up to 2 x (delay_tape_wow + delay_tape_flutter) at their maxima
// (1 056 samples at 96 kHz), plus the newest sample and the interpolation's second point.
constexpr int kTapeInputLine = 2048;
static_assert(2.0f * (btparam::kParams[btparam::kDelayTapeWow].max + btparam::kParams[btparam::kDelayTapeFlutter].max) *
                          kSampleRate / 1000.0f + 3.0f <= float(kTapeInputLine),
              "the wow line holds the deepest wobble");

// Unity up to kLimitKnee, C1 at the knee, approaching 1.0, which it reaches at kLimitKnee + 3 (1 - kLimitKnee) = 2.0.
BT_ITCM inline float SoftLimit(float v) {
    const float a = std::fabs(v);
    if (a <= kLimitKnee) return v;
    return std::copysign(kLimitKnee + (1.0f - kLimitKnee) * SoftClip((a - kLimitKnee) / (1.0f - kLimitKnee)), v);
}

struct DcBlocker {
    float r = 1.0f, x1 = 0.0f, y1 = 0.0f;
    void Init(float sampleRate) {
        r = 1.0f - 2.0f * kPi * kDcHz / sampleRate;
        x1 = y1 = 0.0f;
    }
    BT_ITCM float Process(float x) {
        const float y = Flush(x - x1 + r * y1);
        x1 = x;
        y1 = y;
        return y;
    }
};

// Everything the delay takes once per engine block. crush.amount, tape.age, tape.colorShare and bbd.amount are
// ignored: SetTargets and the scopes derive them.
struct DelayTargets {
    float timeMs = 350.0f, feedback = 0.35f;  // ms; gain 0..1.1
    btctl::DelayType type = btctl::DelayType::kTape;
    float color = 0.0f;  // knob 6, 0..1
    CrushSettings crush;
    // delay_crush_scope, delay_tape_scope, delay_bbd_scope: 0 in the loop, 1 on the input (ColorInput).
    float crushScope = 0.0f, tapeScope = 0.0f, bbdScope = 0.0f;
    TapeSettings tape;
    BbdSettings bbd;
    BbdModSettings bbdMod;
    bool inputOn = true, reverse = false;
};

// A settings field and the parameter whose smoothMs it smooths with.
template <class S> struct SmoothField {
    float S::*field;
    uint8_t id;
};
// The crush options fade over their own smoothMs; the delay time (clock) glides with the hold rate it sets.
inline constexpr SmoothField<CrushSettings> kCrushSmooth[] = {
    {&CrushSettings::amount, btparam::kDelayColor},      {&CrushSettings::driveDb, btparam::kDelayCrushDrive},
    {&CrushSettings::rateHz, btparam::kDelayCrushRate},  {&CrushSettings::aaRatio, btparam::kDelayCrushAa},
    {&CrushSettings::toneRatio, btparam::kDelayCrushTone}, {&CrushSettings::open, btparam::kDelayCrushOpen},
    {&CrushSettings::clock, btparam::kDelayCrushClock},  {&CrushSettings::wrap, btparam::kDelayCrushWrap},
    {&CrushSettings::bits, btparam::kDelayCrushBits},    {&CrushSettings::delayS, btparam::kDelayCrushRate}};
// dropoutsPerMin is a rate: not smoothed.
inline constexpr SmoothField<TapeSettings> kTapeSmooth[] = {
    {&TapeSettings::age, btparam::kDelayColor},        {&TapeSettings::wowMs, btparam::kDelayTapeWow},
    {&TapeSettings::flutterMs, btparam::kDelayTapeFlutter}, {&TapeSettings::driveDb, btparam::kDelayTapeDrive},
    {&TapeSettings::toneHz, btparam::kDelayTapeTone},  {&TapeSettings::bumpDb, btparam::kDelayTapeBump},
    {&TapeSettings::hissDb, btparam::kDelayTapeHiss}};
inline constexpr SmoothField<BbdSettings> kBbdSmooth[] = {
    {&BbdSettings::amount, btparam::kDelayColor},     {&BbdSettings::stages, btparam::kDelayBbdStages},
    {&BbdSettings::toneHz, btparam::kDelayBbdTone},   {&BbdSettings::driveDb, btparam::kDelayBbdDrive},
    {&BbdSettings::noiseDb, btparam::kDelayBbdNoise}, {&BbdSettings::clock, btparam::kDelayBbdClock}};

// sm[i] at target's field f[i], with f[i]'s parameter's smoothMs at ctlRate.
template <class S, size_t N>
void InitSmoothers(Smoother (&sm)[N], const SmoothField<S> (&f)[N], float ctlRate, const S& target) {
    for (size_t i = 0; i < N; ++i) sm[i].Init(ctlRate, btparam::SmoothSeconds(f[i].id), target.*f[i].field);
}
// target with each field f[i] replaced by sm[i] stepped toward it (always_inline: a template's out-of-line copy loses
// BT_ITCM, ../memory.h).
template <class S, size_t N>
[[gnu::always_inline]] inline S StepSmoothers(Smoother (&sm)[N], const SmoothField<S> (&f)[N], const S& target) {
    S s = target;
    for (size_t i = 0; i < N; ++i) s.*f[i].field = sm[i].Process(target.*f[i].field);
    return s;
}

// The reverse read: the loop still records forward; only the output reads the line backwards, so every repeat plays
// reversed. Two grains read backwards at 1x (delay 2, growing by 2 per sample), one launched every hop (the delay
// time, at most kRevMaxHop samples); at a launch the new grain fades in and the old one out with equal-power gains
// over min(maxFade, hop/2, kRevMaxFade). A shorter hop applies at once, a longer one from the next launch; a launch
// waits for the running fade, so at most two grains play. Wow, flutter and the BBD clock LFO offset the grains too
// (the caller's read mapping), dropouts scale them; they never glide.
class ReverseGrains {
public:
    // maxFade: the longest grain crossfade, samples.
    void Init(int maxFade) {
        maxFade_ = maxFade;
        hop_ = 2;
        Reset();
    }
    // No grain playing; the next Process launches one.
    BT_ITCM void Reset() {
        grain_[0].on = grain_[1].on = false;
        newest_ = 0;
        count_ = 0;
        fading_ = false;
        fade_.c = 1.0f;
        fade_.s = 0.0f;
    }
    // The launch period, samples.
    BT_ITCM void SetHop(long hop) { hop_ = hop; }

    // One reverse sample; read(d): the line d samples back, after the caller's read-position mapping. always_inline:
    // a template's out-of-line copy loses BT_ITCM (memory.h), and this runs every reverse sample.
    template <class R> [[gnu::always_inline]] inline float Process(const R& read) {
        if (count_ > hop_) count_ = hop_;
        if (count_ <= 0 && !fading_) Launch();
        const float gNew = fading_ ? fade_.s : 1.0f;
        const float gOld = fading_ ? fade_.c : 0.0f;
        Grain& gn = grain_[newest_];
        Grain& go = grain_[newest_ ^ 1];
        float w = 0.0f;
        if (gn.on) {
            w += gNew * read(gn.d);
            gn.d += 2.0f;
        }
        if (go.on) {
            w += gOld * read(go.d);
            go.d += 2.0f;
        }
        if (fading_) {
            if (++fadePos_ >= fadeLen_) {
                fading_ = false;
                fade_.c = 0.0f;
                fade_.s = 1.0f;
                go.on = false;
            } else {
                fade_.Step();
            }
        }
        --count_;
        return w;
    }

private:
    struct Grain {
        float d = 2.0f;
        bool on = false;
    };

    // Starts a grain at the newest recorded sample; the previous one fades out over fadeLen_ samples.
    BT_ITCM void Launch() {
        const int k = newest_ ^ 1;
        grain_[k] = Grain{2.0f, true};
        newest_ = k;
        fadeLen_ = std::clamp(std::min(maxFade_, int(hop_ / 2)), 1, kRevMaxFade);
        const float dlt = kHalfPi / float(fadeLen_);
        fade_.c = 1.0f;
        fade_.s = 0.0f;
        fade_.rc = std::cos(dlt);
        fade_.rs = std::sin(dlt);
        fadePos_ = 0;
        fading_ = true;
        count_ = hop_;
    }

    Grain grain_[2];
    int newest_ = 0;  // the grain launched last
    long count_ = 0, hop_ = 2;
    int maxFade_ = 1, fadeLen_ = 1, fadePos_ = 0;
    bool fading_ = false;
    Oscillator fade_;  // (c, s): the old and the new grain's gains, rotated by pi/2 / fadeLen_ per sample
};

class Delay {
public:
    // mem: kDelaySamples floats, cleared here (the SDRAM section is NOLOAD).
    void Init(float sampleRate, float* mem) {
        fs_ = sampleRate;
        mem_ = mem;
        std::memset(mem_, 0, kDelaySamples * sizeof(float));
        wpos_ = 0;
        dc_.Init(fs_);
        crush_.Init(fs_);
        crushIn_.Init(fs_);
        bbd_.Init(fs_);
        bbdIn_.Init(fs_);
        bbdLfo_.Init(fs_);
        tape_.Init(fs_);
        tapeIn_.Init(fs_, kTapeInputSeed);
        std::fill(std::begin(tapeBuf_), std::end(tapeBuf_), 0.0f);
        tapeLine_ = Line{tapeBuf_, kTapeInputLine, 0};
        fadeLen_ = int(std::lround(kXfadeMs * fs_ / 1000.0f));
        grains_.Init(fadeLen_);
        gc_.Init(fs_);
        ctl_ = 1;
        targetD_ = 2.0f;
        fbT_ = inT_ = 0.0f;
        crushT_ = CrushSettings{};
        crushScopeT_ = tapeScopeT_ = bbdScopeT_ = 0.0f;
        tapeT_ = TapeSettings{};
        bbdT_ = BbdSettings{};
        bbdModT_ = BbdModSettings{};
        glideT_ = false;
        bbdOnT_ = 0.0f;
        revT_ = 0.0f;
        Snap();
    }

    // Once per engine Process call: the type glides (tape, BBD) or crossfades (digital) a time change and switches
    // the BBD stage in or out; color drives the selected type's color (the others' amounts are 0, the BBD's amount
    // is color whatever the type, so it is set when the stage fades in).
    BT_ITCM void SetTargets(const DelayTargets& t) {
        targetD_ = std::clamp(t.timeMs * fs_ / 1000.0f, 2.0f, float(kDelaySamples - 3));
        grains_.SetHop(std::min(long(std::lround(targetD_)), kRevMaxHop));
        fbT_ = t.feedback;
        glideT_ = t.type != btctl::DelayType::kDigital;
        bbdOnT_ = t.type == btctl::DelayType::kBbd ? 1.0f : 0.0f;
        crushT_ = t.crush;
        crushT_.amount = t.type == btctl::DelayType::kDigital ? t.color : 0.0f;
        crushT_.delayS = targetD_ / fs_;
        crushScopeT_ = t.crushScope;
        tapeScopeT_ = t.tapeScope;
        bbdScopeT_ = t.bbdScope;
        tapeT_ = t.tape;
        tapeT_.age = t.type == btctl::DelayType::kTape ? t.color : 0.0f;
        bbdT_ = t.bbd;
        bbdT_.amount = t.color;
        bbdModT_ = t.bbdMod;
        inT_ = t.inputOn ? 1.0f : 0.0f;
        revT_ = t.reverse ? 1.0f : 0.0f;
    }

    // Jumps every smoothed value to its target.
    void Snap() {
        head_[0] = head_[1] = targetD_;
        active_ = 0;
        fade_ = Crossfade{};
        glide_.Snap(targetD_);
        fb_.Init(fs_, btparam::SmoothSeconds(btparam::kDelayFeedback), fbT_);
        const float ctlRate = fs_ / kControlInterval;
        InitSmoothers(crushS_, kCrushSmooth, ctlRate, crushT_);
        crushScopeS_.Init(ctlRate, btparam::SmoothSeconds(btparam::kDelayCrushScope), crushScopeT_);
        SetCrushes(crushT_, crushScopeT_);
        crush_.SnapMask();
        crushIn_.SnapMask();
        InitSmoothers(tapeS_, kTapeSmooth, ctlRate, tapeT_);
        SetTapes(tapeT_, tapeScopeT_);
        tapeScopeR_.Init(fs_, btparam::SmoothSeconds(btparam::kDelayTapeScope), tapeScopeT_);
        InitSmoothers(bbdS_, kBbdSmooth, ctlRate, bbdT_);
        bbd_.Set(bbdT_);
        bbd_.SetDelay(targetD_);
        bbd_.Reset();
        bbdIn_.Set(bbdT_);
        bbdIn_.SetDelay(targetD_);
        bbdIn_.Reset();
        bbdInLive_ = false;
        bbdScopeR_.Init(fs_, btparam::SmoothSeconds(btparam::kDelayBbdScope), bbdScopeT_);
        bbdOn_.Init(fs_, kXfadeMs * 1e-3f, bbdOnT_);
        bbdModS_.Init(ctlRate, btparam::SmoothSeconds(btparam::kDelayBbdModDepth), bbdOnT_ * bbdModT_.depth);
        bbdLfo_.Reset();
        bbdLfo_.Set(bbdModT_.rateHz, bbdModS_.Value());
        bbdLive_ = false;
        in_.Init(fs_, kEnableRampS, inT_);
        rev_.Init(fs_, kXfadeMs * 1e-3f, revT_);
        grains_.Reset();
    }

    // One sample in, the wet (read) sample out.
    BT_ITCM float Process(float x) {
        const float tsc = tapeScopeR_.Process(tapeScopeT_);
        const float bsc = bbdScopeR_.Process(bbdScopeT_);
        const float fb = fb_.Process(fbT_);
        if (--ctl_ == 0) {
            ctl_ = kControlInterval;
            SetCrushes(StepSmoothers(crushS_, kCrushSmooth, crushT_), crushScopeS_.Process(crushScopeT_));
            SetTapes(StepSmoothers(tapeS_, kTapeSmooth, tapeT_), tsc);
            tape_.Tick();
            tapeIn_.Tick();
            bbdLfo_.Set(bbdModT_.rateHz, bbdModS_.Process(bbdOnT_ * bbdModT_.depth));
            if (bbdOnT_ > 0.0f || bbdOn_.Value() > 0.0f) {  // the smoothers resume from where they stopped
                const BbdSettings bs = StepSmoothers(bbdS_, kBbdSmooth, bbdT_);
                bbd_.Set(bs);
                bbd_.SetDelay(head_[active_] + bbdLfo_.Offset());
                bbdIn_.Set(bs);
                bbdIn_.SetDelay(head_[active_]);  // the input chip is clocked like the delay's, without the clock LFO
            }
        }
        const float g = in_.Process(inT_);
        // Every sample regardless of mode, so the tape's state advances by sample count.
        const bool tapeOn = tape_.Active();
        const float mod = tape_.ReadOffset();
        const float drop = tape_.Dropout();
        // The glide owns the active head except during a digital crossfade; every other place that sets the head
        // re-snaps it.
        if (!fade_.on) {
            if (glideT_) {
                head_[active_] = glide_.Process(targetD_, gc_.a, gc_.b, kTapeGlideMaxRate, 1e-3f);
            } else if (targetD_ != head_[active_]) {
                head_[active_ ^ 1] = targetD_;
                fade_.Start(fadeLen_);
            }
        }
        const float boff = bbdLfo_.Process(head_[active_]);  // every sample: the LFO counts samples
        const bool moved = tapeOn || boff != 0.0f;
        const float off = (tapeOn ? mod : 0.0f) + boff;
        const auto pos = [&](float h) { return moved ? std::clamp(h + off, 2.0f, float(kDelaySamples - 3)) : h; };
        float wF = Read(pos(head_[active_]));
        if (fade_.on) {
            wF += fade_.Weight() * (Read(pos(head_[active_ ^ 1])) - wF);
            if (fade_.Advance()) {
                active_ ^= 1;
                glide_.Snap(head_[active_]);
            }
        }
        if (tapeOn) wF *= drop;
        float w = wF;
        if (revT_ > 0.0f || rev_.Value() > 0.0f) {  // a branch, not a zero gain: forward stays bit-identical
            float wR = grains_.Process([&](float d) { return Read(pos(d)); });
            if (tapeOn) wR *= drop;
            const float r = rev_.Process(revT_);
            w = wF + SmoothStep(r) * (wR - wF);
            if (r == 0.0f && revT_ == 0.0f) grains_.Reset();
        }
        float v = crush_.Process(dc_.Process(tape_.Record(g * x + fb * wF)));
        const float on = bbdOn_.Process(bbdOnT_) * (1.0f - bsc);
        v = SoftLimit(ThroughBbd(bbd_, bbdLive_, on, v));
        mem_[wpos_] = v;
        wpos_ = (wpos_ + 1) & kDelayMask;
        return tape_.Playback(crush_.InvertBits(w));
    }

    // The chain's input (after a pre EQ) through the input colors: tape (delay_tape_scope all: wow and flutter on a
    // short line, dropouts, saturation, HF loss, bump, hiss), bitcrush (delay_crush_scope all, with its bit inversion)
    // and BBD (delay_bbd_scope all, crossfaded with the BBD stage); x exactly while all three are off. Once per sample,
    // before Process, whatever the scopes: the wobble ramps count samples (an idle tape or crush runs no filters and
    // starts them warm). Settings move on Process's control ticks.
    BT_ITCM float ColorInput(float x) {
        const float off = tapeIn_.ReadOffset();
        const float span = tapeIn_.WobbleSpan();
        const float drop = tapeIn_.Dropout();
        tapeLine_.Write(x);
        // After Write, delay 1 is x itself: no wobble reads x exactly.
        float v = tapeIn_.Active() ? Tap(tapeLine_, 1.0f + span + off, 1.0f) * drop : x;
        v = tapeIn_.Playback(tapeIn_.Record(v));
        const float c = crushIn_.InvertBits(crushIn_.Process(v));
        if (crushIn_.Active()) v = c;
        return ThroughBbd(bbdIn_, bbdInLive_, bbdOn_.Value() * bbdScopeR_.Value(), v);
    }

private:
    // Both crushes from cs, the amount split by the scope (all: 0 in the loop .. 1 on the input).
    BT_ITCM void SetCrushes(CrushSettings cs, float all) {
        const float a = cs.amount;
        cs.amount = a * (1.0f - all);
        crush_.Set(cs);
        cs.amount = a * all;
        crushIn_.Set(cs);
    }

    // Both tapes from ts, split by the scope (all as above): the loop's keeps wow and flutter at the full age and
    // colors with the share 1 - all; the input's runs at age x all.
    BT_ITCM void SetTapes(TapeSettings ts, float all) {
        ts.colorShare = 1.0f - all;
        tape_.Set(ts);
        ts.colorShare = 1.0f;
        ts.age *= all;
        tapeIn_.Set(ts);
    }

    // v crossfaded by on (0..1) into stage b's output. b runs only while on > 0 and is reset when on falls to 0, so the
    // next switch-in starts from silence (live: b ran since its last reset).
    BT_ITCM static float ThroughBbd(Bbd& b, bool& live, float on, float v) {
        if (on > 0.0f) {
            live = true;
            return v + on * (b.Process(v) - v);
        }
        if (live) {
            b.Reset();
            live = false;
        }
        return v;
    }

    // d samples behind the slot about to be written, d >= 2 (so the read never touches that slot).
    BT_ITCM float Read(float d) const {
        const size_t di = size_t(d);
        const float f = d - float(di);
        const float x0 = mem_[(wpos_ - (di - 1)) & kDelayMask];
        const float x1 = mem_[(wpos_ - di) & kDelayMask];
        const float x2 = mem_[(wpos_ - (di + 1)) & kDelayMask];
        const float x3 = mem_[(wpos_ - (di + 2)) & kDelayMask];
        const float c1 = 0.5f * (x2 - x0);
        const float c2 = x0 - 2.5f * x1 + 2.0f * x2 - 0.5f * x3;
        const float c3 = 0.5f * (x3 - x0) + 1.5f * (x1 - x2);
        return ((c3 * f + c2) * f + c1) * f + x1;
    }

    float fs_ = 0.0f;  // Init
    float* mem_ = nullptr;
    size_t wpos_ = 0;
    float targetD_ = 2.0f, fbT_ = 0.0f, inT_ = 0.0f;
    CrushSettings crushT_;
    float crushScopeT_ = 0.0f, tapeScopeT_ = 0.0f, bbdScopeT_ = 0.0f;  // delay_*_scope: 0 in the loop, 1 on the input
    Ramp tapeScopeR_, bbdScopeR_;  // per sample, over each scope's smoothing
    TapeSettings tapeT_;
    float head_[2] = {2.0f, 2.0f};
    int active_ = 0, fadeLen_ = 1, ctl_ = 1;
    Crossfade fade_;  // digital: from head_[active_] to the other head
    Glide glide_;     // the active head while not fading
    GlideCoefs gc_;
    Smoother fb_;
    Smoother crushS_[std::size(kCrushSmooth)];  // at the control rate
    Smoother crushScopeS_;                      // at the control rate
    Smoother tapeS_[std::size(kTapeSmooth)];
    Ramp in_;
    ReverseGrains grains_;
    float revT_ = 0.0f;  // 1 reverse, 0 forward
    Ramp rev_;
    DcBlocker dc_;
    Crush crush_;
    Crush crushIn_;  // delay_crush_scope all (ColorInput)
    Tape tape_;
    Tape tapeIn_;                   // delay_tape_scope all (ColorInput)
    Line tapeLine_;                 // its wow line, in tapeBuf_
    float tapeBuf_[kTapeInputLine];
    // The BBD stage (SW2 down).
    Bbd bbd_;
    BbdSettings bbdT_;
    Smoother bbdS_[std::size(kBbdSmooth)];  // at the control rate
    Ramp bbdOn_;                            // the BBD stage's crossfade in and out
    BbdLfo bbdLfo_;
    BbdModSettings bbdModT_;
    Smoother bbdModS_;  // the LFO depth, target bbdOnT_ x depth, at the control rate
    float bbdOnT_ = 0.0f;
    bool bbdLive_ = false;
    Bbd bbdIn_;  // delay_bbd_scope all (ColorInput)
    bool bbdInLive_ = false;
    bool glideT_ = false;  // time changes glide (tape, BBD) or crossfade (digital)
};

}  // namespace btdsp
