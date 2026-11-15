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
// off). The SOS head reverses on its own grains over the SOS line, turning around where the head plays (overdub: at 2,
// one pass newer), then in windows of delay_sos_length capped at SosRevMaxHop (301: 60.05 s; a longer length gets
// 60.05 s windows and a shorter first grain, 53 ms at 120 s), the head's delay (501, wobbling) or half the loop
// (overdub). The SOS line records forward, and the 501 feedback and the overdub keep take the forward head.
// Sound on sound (delay_sos_mode, delay_sos): a second line (SosLine, caller-provided; 44 MiB of SDRAM on the Seed)
// records what the loop writes, every sample in every mode, and one more head reads it into the wet, through the
// same output-only colors as the repeats (bit inversion, tape bump and hiss). delay_sos fades the head in and out
// over kXfadeMs. 301: the head is delay_sos_length back and plays once. 501: the head is at delay_sos_ratio x the
// delay time, wobbles and glides like the main head, and feeds back into the loop at the delay feedback. Overdub: the
// head is delay_sos_length back and the line writes what the loop writes plus delay_sos_feedback x what the head
// plays (soft limited), so switching on loops the last delay_sos_length and keeps layering. A length, ratio or mode
// change crossfades the head (a 501 time change on tape or BBD glides it). SW1 off and bypass stop the input, and the
// SOS head keeps playing in the last direction. With SOS off and faded out the output is bit-identical to a delay
// without it. Live, the head runs out of line (ProcessSos); its ramps, smoothers and 501 glide step on the control
// tick (SosTick), its gains ramp linearly across each tick.
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

// The sound-on-sound line (Delay::Init's SosLine on the Seed): 11 x 2^20 floats (44 MiB, 120.15 s at 96 kHz).
constexpr size_t kSosSamples = size_t(11) << 20;
static_assert(btparam::kParams[btparam::kDelaySosLength].max * kSampleRate + 3.0f <= float(kSosSamples),
              "the SOS line holds the longest length");
// The longest SOS reverse window (301, 501) on a line of n samples: a grain reads at most 2 + 2 hop + 2 kRevMaxFade
// <= n - 3 back. The Seed's line: 5 765 117 samples, 60.05 s at 96 kHz.
constexpr long SosRevMaxHop(size_t n) { return std::max(1L, long((n - 5) / 2) - kRevMaxFade); }
static_assert(SosRevMaxHop(kSosSamples) >= long(60.0f * kSampleRate), "301 reverses windows of at least 60 s");
// Caller-owned memory for the SOS line: any length >= 8 samples; lengths and heads clamp to it.
struct SosLine {
    float* mem;
    size_t samples;
};

// 4-point Hermite through x0..x3 (x1 at f = 0, x2 at f = 1; x0 is the newer neighbour).
BT_ITCM inline float Hermite(float x0, float x1, float x2, float x3, float f) {
    const float c1 = 0.5f * (x2 - x0);
    const float c2 = x0 - 2.5f * x1 + 2.0f * x2 - 0.5f * x3;
    const float c3 = 0.5f * (x3 - x0) + 1.5f * (x1 - x2);
    return ((c3 * f + c2) * f + c1) * f + x1;
}

// Unity up to kLimitKnee, C1 at the knee, approaching 1.0, which it reaches at kLimitKnee + 3 (1 - kLimitKnee) = 2.0.
BT_ITCM inline float SoftLimit(float v) {
    const float a = std::fabs(v);
    if (!Above(a, kLimitKnee)) return v;
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

// Everything the delay takes once per engine block. crush.amount, crush.delayS, tape.age, tape.colorShare and
// bbd.amount are ignored: SetTargets and the scopes derive them.
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
    // Sound on sound: delay_sos_mode, delay_sos, delay_sos_length (s), delay_sos_ratio (x the delay time),
    // delay_sos_feedback and delay_sos_level (0..1).
    btctl::SosMode sosMode = btctl::SosMode::kOff;
    bool sosOn = false;
    float sosLengthS = 15.0f, sosRatio = 2.5f, sosFeedback = 0.9f, sosLevel = 1.0f;
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
// (the caller's read mapping), dropouts scale them; they never glide. Reset(d) starts the next grain d back for the
// rest of a window (until 2 + 2 hop, at least 1 sample); later grains start at 2.
class ReverseGrains {
public:
    // maxFade: the longest grain crossfade, samples.
    void Init(int maxFade) {
        maxFade_ = maxFade;
        hop_ = 2;
        Reset();
    }
    // No grain playing; the next Process launches one, d samples back.
    BT_ITCM void Reset(float d = 2.0f) {
        grain_[0].on = grain_[1].on = false;
        newest_ = 0;
        count_ = 0;
        fading_ = false;
        fade_.c = 1.0f;
        fade_.s = 0.0f;
        first_ = d;
    }
    // Both grains are off only between a Reset and the next Process.
    BT_ITCM bool Idle() const { return !grain_[0].on && !grain_[1].on; }
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
        grain_[k] = Grain{first_, true};
        newest_ = k;
        fadeLen_ = std::clamp(std::min(maxFade_, int(hop_ / 2)), 1, kRevMaxFade);
        const float dlt = kHalfPi / float(fadeLen_);
        fade_.c = 1.0f;
        fade_.s = 0.0f;
        fade_.rc = std::cos(dlt);
        fade_.rs = std::sin(dlt);
        fadePos_ = 0;
        fading_ = true;
        count_ = std::max(1L, hop_ - long((first_ - 2.0f) * 0.5f));
        first_ = 2.0f;
    }

    Grain grain_[2];
    int newest_ = 0;  // the grain launched last
    long count_ = 0, hop_ = 2;
    float first_ = 2.0f;  // the next launch's depth (Reset)
    int maxFade_ = 1, fadeLen_ = 1, fadePos_ = 0;
    bool fading_ = false;
    Oscillator fade_;  // (c, s): the old and the new grain's gains, rotated by pi/2 / fadeLen_ per sample
};

class Delay {
public:
    // mem: kDelaySamples floats; sos: the SOS line. Both cleared here (the SDRAM sections are NOLOAD).
    void Init(float sampleRate, float* mem, SosLine sos) {
        fs_ = sampleRate;
        mem_ = mem;
        std::memset(mem_, 0, kDelaySamples * sizeof(float));
        wpos_ = 0;
        sos_ = sos.mem;
        sosN_ = sos.samples;
        sosMax_ = float(sosN_ - 3);
        sosRevMaxHop_ = SosRevMaxHop(sosN_);
        std::memset(sos_, 0, sosN_ * sizeof(float));
        sosW_ = 0;
        sosMode_ = btctl::SosMode::kOff;
        sosOnT_ = m501T_ = mOdT_ = 0.0f;
        sosLevelT_ = 1.0f;
        sosFbT_ = 0.9f;
        sosTargetD_ = 2.0f;
        sosGlides_ = false;
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
        sosGrains_.Init(fadeLen_);
        gc_.Init(fs_);
        sosGc_.Init(fs_ / kControlInterval);
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
        // SOS: mode off fades the head out with the old mode's behaviour (sosMode_ and the weights stay).
        const bool armed = t.sosMode != btctl::SosMode::kOff;
        sosOnT_ = armed && t.sosOn ? 1.0f : 0.0f;
        if (armed) {
            if (t.sosMode != sosMode_) {
                sosJump_ = true;
                sosMode_ = t.sosMode;
            }
            m501T_ = sosMode_ == btctl::SosMode::k501 ? 1.0f : 0.0f;
            mOdT_ = sosMode_ == btctl::SosMode::kOverdub ? 1.0f : 0.0f;
        }
        sosTargetD_ = sosMode_ == btctl::SosMode::k501
                          ? std::clamp(t.sosRatio * targetD_, 2.0f, sosMax_)
                          : std::clamp(float(std::lround(t.sosLengthS * fs_)), 2.0f, sosMax_);
        // SOS reverse windows: overdub half the loop (a grain spans one pass and the next starts at the same loop
        // position one pass newer); 301 and 501 the head's delay, at most sosRevMaxHop_ (301 above 60.05 s reverses
        // 60.05 s windows).
        const long sosD = std::lround(sosTargetD_);
        sosGrains_.SetHop(sosMode_ == btctl::SosMode::kOverdub ? std::max(1L, sosD / 2)
                                                                : std::min(sosD, sosRevMaxHop_));
        sosGlides_ = glideT_ && sosMode_ == btctl::SosMode::k501;
        sosLevelT_ = t.sosLevel;
        sosFbT_ = t.sosFeedback;
        if (!sosLive_) SnapSosHeads();  // idle: nothing has moved them since
        sosLive_ = sosLive_ || sosOnT_ > 0.0f;
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
        sosGrains_.Reset();
        SnapSosHeads();
        sosOn_.Init(ctlRate, kXfadeMs * 1e-3f, sosOnT_);
        m501_.Init(ctlRate, kXfadeMs * 1e-3f, m501T_);
        mOd_.Init(ctlRate, kXfadeMs * 1e-3f, mOdT_);
        sosLevel_.Init(ctlRate, btparam::SmoothSeconds(btparam::kDelaySosLevel), sosLevelT_);
        sosFb_.Init(ctlRate, btparam::SmoothSeconds(btparam::kDelaySosFeedback), sosFbT_);
        SetSosGains();
        sosOut_.Snap();
        sosFbIn_.Snap();
        sosKeep_.Snap();
        sosLive_ = sosOnT_ > 0.0f;
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
            } else if (!SameBits(targetD_, head_[active_])) {  // heads >= 2: same bits is equality
                head_[active_ ^ 1] = targetD_;
                fade_.Start(fadeLen_);
            }
        }
        const float boff = bbdLfo_.Process(head_[active_]);  // every sample: the LFO counts samples
        const bool moved = tapeOn || !IsZero(boff);
        const float off = (tapeOn ? mod : 0.0f) + boff;
        const auto pos = [&](float h) { return moved ? Clamp(h + off, 2.0f, float(kDelaySamples - 3)) : h; };
        float wF = Read(pos(head_[active_]));
        if (fade_.on) {
            wF = fade_.Mix(wF, Read(pos(head_[active_ ^ 1])));
            if (fade_.Advance()) {
                active_ ^= 1;
                glide_.Snap(head_[active_]);
            }
        }
        if (tapeOn) wF *= drop;
        float w = wF, r = 0.0f;  // r: the direction ramp, 0 forward .. 1 reverse
        if (IsPositive(revT_) || IsPositive(rev_.Value())) {  // a branch, not a zero gain: forward stays bit-identical
            float wR = grains_.Process([&](float d) { return Read(pos(d)); });
            if (tapeOn) wR *= drop;
            r = rev_.Process(revT_);
            w = wF + SmoothStep(r) * (wR - wF);
            if (IsZero(r) && IsZero(revT_)) {
                grains_.Reset();
                sosGrains_.Reset();
            }
        }
        // SOS: idle, the loop is exactly the delay's own (a branch, not a zero gain, so the output stays bit-identical)
        // and the line just records; live, out of line (inlined, its registers cost the idle path ~25 ‰ on the bench).
        if (sosLive_) return ProcessSos(g * x, fb, wF, w, bsc, moved, off, r);
        StoreSos(WriteLoop(g * x + fb * wF, bsc));
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
        if (IsPositive(on)) {
            live = true;
            return v + on * (b.Process(v) - v);
        }
        if (live) {
            b.Reset();
            live = false;
        }
        return v;
    }

    // The loop from its input in: tape record, DC blocker, crush, BBD, soft limiter, written to the line; returns the
    // written sample. always_inline: the idle path stays the delay's own.
    [[gnu::always_inline]] inline float WriteLoop(float in, float bsc) {
        float v = crush_.Process(dc_.Process(tape_.Record(in)));
        const float on = bbdOn_.Process(bbdOnT_) * (1.0f - bsc);
        v = SoftLimit(ThroughBbd(bbd_, bbdLive_, on, v));
        mem_[wpos_] = v;
        wpos_ = (wpos_ + 1) & kDelayMask;
        return v;
    }

    // Writes v at the SOS line's write slot and advances it. always_inline: the idle path stays the delay's own.
    [[gnu::always_inline]] inline void StoreSos(float v) {
        sos_[sosW_] = v;
        sosW_ = sosW_ + 1 == sosN_ ? 0 : sosW_ + 1;
    }

    // The rest of Process while SOS is live: gx the loop's input (g x), fb the feedback, wF the main forward read, w
    // the main wet, bsc the BBD scope, moved and off the main head's wobble (tape wow and flutter, the BBD clock LFO),
    // applied to a 501 head, r the direction ramp (0 forward .. 1 reverse): the SOS wet crossfades between its forward
    // head and its reverse grains like the main head. Returns the wet. Per sample only the head, its read, three linear
    // gain steps and, in reverse, the grains; the rest is SosTick's, on Process's control ticks.
    [[gnu::noinline]] BT_ITCM float ProcessSos(float gx, float fb, float wF, float w, float bsc, bool moved,
                                               float off, float r) {
        if (ctl_ == kControlInterval) SosTick();  // Process ticked this sample
        const float gOut = sosOut_.Step(), gFb = sosFbIn_.Step(), gKeep = sosKeep_.Step();
        sosHead_[sosActive_] += sosStep_;
        const auto sp = [&](int k) {
            return sosWob_[k] && moved ? Clamp(sosHead_[k] + off, 2.0f, sosMax_) : sosHead_[k];
        };
        float s = ReadSos(sp(sosActive_));
        if (sosFade_.on) {
            s = sosFade_.Mix(s, ReadSos(sp(sosActive_ ^ 1)));
            if (sosFade_.Advance()) {
                sosActive_ ^= 1;
                sosGlide_.Snap(sosHead_[sosActive_]);
                sosStep_ = 0.0f;
            }
        }
        float heard = s;  // the SOS wet: the forward head, or crossfaded to the reverse grains by SW1's ramp
        if (IsPositive(r)) {
            // Fresh grains turn around where the head plays (overdub: at 2, the same loop position one pass newer,
            // which keeps the grains on the loop's grid).
            if (sosGrains_.Idle()) sosGrains_.Reset(sosMode_ == btctl::SosMode::kOverdub ? 2.0f : sosHead_[sosActive_]);
            const bool wob = moved && sosMode_ == btctl::SosMode::k501;  // 501's grains wobble like its head
            const float sR = sosGrains_.Process(
                [&](float d) { return ReadSos(Min(wob ? Max(d + off, 2.0f) : d, sosMax_)); });
            heard = s + SmoothStep(r) * (sR - s);
        }
        const float v = WriteLoop(gx + fb * (wF + gFb * s), bsc);  // 501: the forward head feeds the loop
        StoreSos(IsPositive(gKeep) ? SoftLimit(v + gKeep * s) : v);  // overdub: the line keeps its forward loop
        return tape_.Playback(crush_.InvertBits(w + gOut * heard));
    }

    // Once per control tick while live: the on/off, mode and level ramps, the gains' next targets (reached linearly over
    // the tick), and the head: a 501 glide step on tape or BBD (spread over the tick's samples), or a crossfade to its
    // target (a mode change always crossfades). Faded out, it ends the live state (the gains are exactly 0).
    [[gnu::noinline]] BT_ITCM void SosTick() {
        sosOut_.Snap();
        sosFbIn_.Snap();
        sosKeep_.Snap();
        if (sosOnT_ == 0.0f && sosOn_.Value() == 0.0f) {
            sosLive_ = false;
            sosGrains_.Reset();  // the next reverse switch-on turns around fresh
            sosStep_ = 0.0f;
            return;
        }
        sosOn_.Process(sosOnT_);
        m501_.Process(m501T_);
        mOd_.Process(mOdT_);
        sosLevel_.Process(sosLevelT_);
        sosFb_.Process(sosFbT_);
        SetSosGains();
        sosStep_ = 0.0f;
        if (sosFade_.on) return;
        if (sosJump_ || !sosGlides_) {
            if (sosTargetD_ != sosHead_[sosActive_]) {
                const int o = sosActive_ ^ 1;
                sosHead_[o] = sosTargetD_;
                sosWob_[o] = sosMode_ == btctl::SosMode::k501;
                sosFade_.Start(fadeLen_);
            }
            sosJump_ = false;
        } else {
            const float next = sosGlide_.Process(sosTargetD_, sosGc_.a, sosGc_.b,
                                                 kTapeGlideMaxRate * float(kControlInterval), 1e-3f);
            sosStep_ = (next - sosHead_[sosActive_]) * (1.0f / float(kControlInterval));
        }
    }

    // The SOS gains' targets from the ramps and smoothers: the wet (on x level), the 501 feedback (on x 501) and the
    // overdub's keep (on x overdub x delay_sos_feedback).
    BT_ITCM void SetSosGains() {
        const float on = SmoothStep(sosOn_.Value());
        sosOut_.Set(on * sosLevel_.Value());
        sosFbIn_.Set(on * m501_.Value());
        sosKeep_.Set(on * mOd_.Value() * sosFb_.Value());
    }

    // Both SOS heads at their target, no fade, no glide.
    BT_ITCM void SnapSosHeads() {
        sosHead_[0] = sosHead_[1] = sosTargetD_;
        sosActive_ = 0;
        sosFade_ = Crossfade{};
        sosGlide_.Snap(sosTargetD_);
        sosStep_ = 0.0f;
        sosWob_[0] = sosWob_[1] = sosMode_ == btctl::SosMode::k501;
        sosJump_ = false;
    }

    // d samples behind the slot about to be written, d >= 2 (so the read never touches that slot).
    BT_ITCM float Read(float d) const {
        const size_t di = size_t(d);
        const float f = d - float(di);
        const float x0 = mem_[(wpos_ - (di - 1)) & kDelayMask];
        const float x1 = mem_[(wpos_ - di) & kDelayMask];
        const float x2 = mem_[(wpos_ - (di + 1)) & kDelayMask];
        const float x3 = mem_[(wpos_ - (di + 2)) & kDelayMask];
        return Hermite(x0, x1, x2, x3, f);
    }

    // The SOS line d samples behind the slot about to be written, 2 <= d <= sosN_ - 3; exactly a sample at whole d.
    BT_ITCM float ReadSos(float d) const {
        const size_t di = size_t(d);
        const float f = d - float(di);
        const size_t i1 = sosW_ >= di ? sosW_ - di : sosW_ + sosN_ - di;
        if (IsZero(f)) return sos_[i1];  // a whole delay (301, overdub): the sample itself, as Hermite gives it
        const size_t i0 = i1 + 1 == sosN_ ? 0 : i1 + 1;
        const size_t i2 = i1 == 0 ? sosN_ - 1 : i1 - 1;
        const size_t i3 = i2 == 0 ? sosN_ - 1 : i2 - 1;
        return Hermite(sos_[i0], sos_[i1], sos_[i2], sos_[i3], f);
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
    // Sound on sound.
    float* sos_ = nullptr;
    size_t sosN_ = 8, sosW_ = 0;  // length, the slot about to be written
    float sosMax_ = 5.0f;         // sosN_ - 3: the deepest SOS read
    btctl::SosMode sosMode_ = btctl::SosMode::kOff;  // the last mode other than off
    float sosOnT_ = 0.0f, m501T_ = 0.0f, mOdT_ = 0.0f, sosLevelT_ = 1.0f, sosFbT_ = 0.9f, sosTargetD_ = 2.0f;
    ReverseGrains sosGrains_;  // the SOS head's reverse read (SW1 down)
    long sosRevMaxHop_ = 1;    // SosRevMaxHop(sosN_)
    bool sosJump_ = false;    // a mode change: the head crossfades even where it would glide
    bool sosGlides_ = false;  // 501 on tape or BBD
    bool sosLive_ = false;    // the head is on or fading: Process runs ProcessSos
    float sosHead_[2] = {2.0f, 2.0f};
    bool sosWob_[2] = {false, false};  // the head takes wow, flutter and the BBD clock LFO (501)
    int sosActive_ = 0;
    Crossfade sosFade_;  // from sosHead_[sosActive_] to the other head
    Glide sosGlide_;     // the active head while gliding, at the control rate
    GlideCoefs sosGc_;   // the glide laws at the control rate
    float sosStep_ = 0.0f;       // the active head's per-sample move over this tick (its glide step / kControlInterval)
    Ramp sosOn_, m501_, mOd_;    // delay_sos on, and the 501 and overdub weights, over kXfadeMs at the control rate
    Smoother sosLevel_, sosFb_;  // over their parameters' smoothing, at the control rate
    TickRamp sosOut_, sosFbIn_, sosKeep_;  // SetSosGains' gains, linear across each tick
};

}  // namespace btdsp
