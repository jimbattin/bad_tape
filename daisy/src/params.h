// The parameter table: the single source of truth for every parameter of the pedal. The
// firmware, the MIDI CC/NRPN/SysEx protocol (midi.h, link.h), the host render (host/render.cpp), docs/MIDI.md
// (host/gen_midi_doc.cpp, `make midi-doc`) and daisy/tools/usb_log.py (via SysEx DUMP) all read it. Host-portable
// (no libDaisy, no allocation), covered by tests/params_test.cpp.
//
// Ids are append-only: MIDI (NRPN numbers, SysEx ids), the docs and the tools depend on them. Ranges and defaults are
// our own values and may be retuned without renumbering.
//
// Values travel as 14-bit codes (0..16383) along the parameter's taper (ToCode/FromCode) and arrive as 7-bit CC values
// (FromCc). Enums are integers min..max; a code is the value's position on the 0..1 scale (ToNorm) x kCodeMax,
// and FromCode rounds back to the integer.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>

#include "controls.h"

namespace btparam {

// kAudio: an offset exponential like an A-taper pot, min + (max - min) * (32^n - 1) / 31; 15 % of the range at n = 0.5.
enum class Taper : uint8_t { kLinear = 0, kLog = 1, kEnum = 2, kAudio = 3 };
enum class Control : uint8_t { kNone = 0, kSw1 = 1, kSw2 = 2, kSw3 = 3, kFs1 = 4, kFs2 = 5 };
enum class Group : uint8_t { kGlobal = 0, kDelay = 1, kHall = 2, kPlate = 3, kCanyon = 4, kEq = 5, kKnobs = 6 };

struct ParamDef {
    uint8_t id;        // == index in kParams
    const char* name;  // snake_case, unique: render --set and usb_log.py use it
    const char* unit;  // engineering unit; an enum lists its values, e.g. "0 off, 1 on"
    float min, max, def;
    Taper taper;
    float smoothMs;  // one-pole time constant DSP stages apply (btdsp::Smoother); 0 = the stage handles changes itself
    Control control;
    uint8_t cc;  // 0 = no CC
    Group group;
};

enum ParamId : uint8_t {
    kBypass, kDelayMode, kRouting, kReverbAlgo, kDelayMix, kReverbMix, kDelayTime, kDelayFeedback, kDelayColor,
    kHallRtMid, kHallBassMult, kHallCrossover, kHallRtHfCut, kHallHfCutoff, kHallSize, kHallShape, kHallSpread,
    kHallDiffusion, kHallPredelay, kHallEchoLevel1, kHallEchoLevel2, kHallEchoLevel3, kHallEchoLevel4,
    kHallEchoDelay1, kHallEchoDelay2, kHallEchoDelay3, kHallEchoDelay4, kHallSpin, kHallWander, kHallOutputShelf,
    kPlateMidDecay, kPlateLfDecay, kPlateCrossover, kPlateTrebleDecay, kPlateHfBandwidth, kPlatePredelay,
    kPlateDiffusion, kPlateDefinition, kPlateDepth, kPlateChorus, kPlateSize, kPlateDecayOpt, kPlateStopDecay,
    kPlateLfStopDecay, kPlateMidStopDecay, kPlateStopDelay,
    kCanyonFeedback, kCanyonFeedbackInvert, kCanyonMasterDelay, kCanyonMasterRate, kCanyonMasterDepth,
    kCanyonGlideRate, kCanyonDelayGliding, kCanyonMix, kCanyonInTrim, kCanyonOutTrim,
    kCanyonDelay1, kCanyonDelay2, kCanyonDelay3, kCanyonDelay4, kCanyonDelay5, kCanyonDelay6,
    kCanyonRate1, kCanyonRate2, kCanyonRate3, kCanyonRate4, kCanyonRate5, kCanyonRate6,
    kCanyonDepth1, kCanyonDepth2, kCanyonDepth3, kCanyonDepth4, kCanyonDepth5, kCanyonDepth6,
    kEqEnable, kEqPosition, kEqLowGain, kEqLowFreq, kEqMidGain1, kEqMidFreq1, kEqMidQ1, kEqHighGain, kEqHighFreq,
    kEqMidFreq2, kEqMidQ2, kEqMidGain2,
    kDelayCrushDrive, kDelayCrushRate, kDelayCrushAa, kDelayCrushTone,
    kDelayTapeWow, kDelayTapeFlutter, kDelayTapeDrive, kDelayTapeTone, kDelayTapeBump, kDelayTapeHiss,
    kDelayTapeDropouts,
    kHallSpace, kPlateSpace, kCanyonSpace,
    kDelayType, kDelayBbdStages, kDelayBbdTone, kDelayBbdDrive, kDelayBbdNoise, kDelayBbdClock,
    kDelayBbdModRate, kDelayBbdModDepth,
    kKnob1Primary, kKnob2Primary, kKnob3Primary, kKnob4Primary, kKnob5Primary, kKnob6Primary,
    kKnob1Secondary, kKnob2Secondary, kKnob3Secondary, kKnob4Secondary, kKnob5Secondary, kKnob6Secondary,
    kDelayCrushOpen, kDelayCrushClock, kDelayCrushWrap, kDelayCrushBits, kDelayCrushMask, kDelayCrushScope,
    kDelayTapeScope, kDelayBbdScope,
    kEqMidGain3, kEqMidFreq3, kEqMidQ3,
    kDelaySosMode, kDelaySos, kDelaySosLength, kDelaySosRatio, kDelaySosFeedback, kDelaySosLevel,
    kParamCount
};
constexpr uint8_t kNoParam = 255;  // an assignment's "none"
static_assert(kParamCount <= kNoParam, "kNoParam is never a parameter id, and uint8_t id loops end");

// The table's short names (taper, control, group); detail only.
namespace detail {
constexpr Taper L = Taper::kLinear, G = Taper::kLog, E = Taper::kEnum, A = Taper::kAudio;
constexpr Control N_ = Control::kNone, S1 = Control::kSw1, S2 = Control::kSw2, S3 = Control::kSw3, F1 = Control::kFs1,
                  F2 = Control::kFs2;
constexpr Group Gl = Group::kGlobal, De = Group::kDelay, Ha = Group::kHall, Pl = Group::kPlate, Ca = Group::kCanyon,
                Eq = Group::kEq, Kn = Group::kKnobs;

// clang-format off
inline constexpr ParamDef kParams[] = {
    //id                     name                      unit             min   max    def     taper ms control cc group
    {kBypass,               "bypass",                 "0 off, 1 on",   0,    1,     0,      E, 0,  F1, 25, Gl},
    {kDelayMode,            "delay_mode",             "0 reverse, 1 off, 2 forward", 0, 2, 2, E, 0, S1, 22, De},
    // The reverb's input (btctl::Routing): MIDI only since SW2 selects the delay type; the default is the delay mix.
    {kRouting,              "routing",                "0 wet, 1 mix, 2 dry", 0, 2,  1,      E, 0,  N_, 0,  Gl},
    {kReverbAlgo,           "reverb_algo",            "0 plate, 1 hall, 2 canyon", 0, 2, 1, E, 0,  S3, 24, Gl},
    {kDelayMix,             "delay_mix",              "%",             0,    100,   50,     L, 20, N_, 14, De},
    {kReverbMix,            "reverb_mix",             "%",             0,    100,   65,     L, 20, N_, 15, Gl},
    {kDelayTime,            "delay_time",             "ms",            10,   2000,  350,    A, 0,  N_, 17, De},
    {kDelayFeedback,        "delay_feedback",         "%",             0,    110,   35,     L, 20, N_, 18, De},
    {kDelayColor,           "delay_color",            "%",             0,    100,   0,      L, 20, N_, 19, De},

    {kHallRtMid,            "hall_rt_mid",            "s",             0.2f, 20,    2.0f,   G, 20, N_, 0,  Ha},
    {kHallBassMult,         "hall_bass_mult",         "x",             0.2f, 4,     1.2f,   G, 20, N_, 0,  Ha},
    {kHallCrossover,        "hall_crossover",         "Hz",            50,   4000,  600,    G, 20, N_, 0,  Ha},
    {kHallRtHfCut,          "hall_rt_hf_cut",         "Hz",            500,  20000, 5000,   G, 20, N_, 0,  Ha},
    {kHallHfCutoff,         "hall_hf_cutoff",         "Hz",            120,  20000, 8000,   G, 20, N_, 0,  Ha},
    {kHallSize,             "hall_size",              "m",             1,    39,    30,     L, 0,  N_, 0,  Ha},
    {kHallShape,            "hall_shape",             "%",             0,    100,   50,     L, 20, N_, 0,  Ha},
    {kHallSpread,           "hall_spread",            "%",             0,    100,   50,     L, 20, N_, 0,  Ha},
    {kHallDiffusion,        "hall_diffusion",         "%",             0,    100,   70,     L, 20, N_, 0,  Ha},
    {kHallPredelay,         "hall_predelay",          "ms",            0,    500,   20,     L, 0,  N_, 0,  Ha},
    {kHallEchoLevel1,       "hall_echo_level_1",      "dB (-60 = off)", -60, 0,     -60,    L, 20, N_, 0,  Ha},
    {kHallEchoLevel2,       "hall_echo_level_2",      "dB (-60 = off)", -60, 0,     -60,    L, 20, N_, 0,  Ha},
    {kHallEchoLevel3,       "hall_echo_level_3",      "dB (-60 = off)", -60, 0,     -60,    L, 20, N_, 0,  Ha},
    {kHallEchoLevel4,       "hall_echo_level_4",      "dB (-60 = off)", -60, 0,     -60,    L, 20, N_, 0,  Ha},
    {kHallEchoDelay1,       "hall_echo_delay_1",      "ms",            0,    400,   15,     L, 0,  N_, 0,  Ha},
    {kHallEchoDelay2,       "hall_echo_delay_2",      "ms",            0,    400,   30,     L, 0,  N_, 0,  Ha},
    {kHallEchoDelay3,       "hall_echo_delay_3",      "ms",            0,    400,   45,     L, 0,  N_, 0,  Ha},
    {kHallEchoDelay4,       "hall_echo_delay_4",      "ms",            0,    400,   60,     L, 0,  N_, 0,  Ha},
    {kHallSpin,             "hall_spin",              "%",             0,    100,   50,     L, 20, N_, 0,  Ha},
    {kHallWander,           "hall_wander",            "%",             0,    100,   40,     L, 20, N_, 0,  Ha},
    {kHallOutputShelf,      "hall_output_shelf",      "dB",            -12,  12,    0,      L, 20, N_, 0,  Ha},

    {kPlateMidDecay,        "plate_mid_decay",        "s",             0.2f, 20,    1.8f,   G, 20, N_, 0,  Pl},
    {kPlateLfDecay,         "plate_lf_decay",         "s",             0.2f, 20,    1.8f,   G, 20, N_, 0,  Pl},
    {kPlateCrossover,       "plate_crossover",        "Hz",            100,  2000,  500,    G, 20, N_, 0,  Pl},
    {kPlateTrebleDecay,     "plate_treble_decay",     "Hz",            1000, 16000, 16000,  G, 20, N_, 0,  Pl},
    {kPlateHfBandwidth,     "plate_hf_bandwidth",     "Hz",            2000, 16000, 10000,  G, 20, N_, 0,  Pl},
    {kPlatePredelay,        "plate_predelay",         "ms",            0,    200,   10,     L, 0,  N_, 0,  Pl},
    {kPlateDiffusion,       "plate_diffusion",        "%",             0,    100,   80,     L, 20, N_, 0,  Pl},
    {kPlateDefinition,      "plate_definition",       "%",             0,    100,   30,     L, 20, N_, 0,  Pl},
    {kPlateDepth,           "plate_depth",            "%",             0,    100,   50,     L, 20, N_, 0,  Pl},
    {kPlateChorus,          "plate_chorus",           "%",             0,    100,   30,     L, 20, N_, 0,  Pl},
    {kPlateSize,            "plate_size",             "%",             0,    100,   70,     L, 0,  N_, 0,  Pl},
    {kPlateDecayOpt,        "plate_decay_opt",        "0 off, 1 on",   0,    1,     1,      E, 0,  N_, 0,  Pl},
    {kPlateStopDecay,       "plate_stop_decay",       "0 off, 1 on",   0,    1,     0,      E, 0,  N_, 0,  Pl},
    {kPlateLfStopDecay,     "plate_lf_stop_decay",    "s",             0.1f, 10,    0.5f,   G, 20, N_, 0,  Pl},
    {kPlateMidStopDecay,    "plate_mid_stop_decay",   "s",             0.1f, 10,    0.5f,   G, 20, N_, 0,  Pl},
    {kPlateStopDelay,       "plate_stop_delay",       "ms",            0,    1000,  200,    L, 0,  N_, 0,  Pl},

    {kCanyonFeedback,       "canyon_feedback",        "%",             0,    100,   95,     L, 20, N_, 0,  Ca},
    {kCanyonFeedbackInvert, "canyon_feedback_invert", "0 off, 1 on",   0,    1,     0,      E, 0,  N_, 0,  Ca},
    {kCanyonMasterDelay,    "canyon_master_delay",    "%",             0,    100,   75,     L, 0,  N_, 0,  Ca},
    {kCanyonMasterRate,     "canyon_master_rate",     "%",             0,    100,   15,     L, 20, N_, 0,  Ca},
    {kCanyonMasterDepth,    "canyon_master_depth",    "%",             0,    100,   50,     L, 20, N_, 0,  Ca},
    {kCanyonGlideRate,      "canyon_glide_rate",      "%",             0,    100,   20,     L, 0,  N_, 0,  Ca},
    {kCanyonDelayGliding,   "canyon_delay_gliding",   "0 off, 1 on",   0,    1,     1,      E, 0,  N_, 0,  Ca},
    {kCanyonMix,            "canyon_mix",             "%",             0,    100,   100,    L, 20, N_, 0,  Ca},
    {kCanyonInTrim,         "canyon_in_trim",         "dB",            -24,  24,    0,      L, 20, N_, 0,  Ca},
    {kCanyonOutTrim,        "canyon_out_trim",        "dB",            -24,  24,    0,      L, 20, N_, 0,  Ca},
    {kCanyonDelay1,         "canyon_delay_1",         "ms",            0,    200,   40,     L, 0,  N_, 0,  Ca},
    {kCanyonDelay2,         "canyon_delay_2",         "ms",            0,    200,   55,     L, 0,  N_, 0,  Ca},
    {kCanyonDelay3,         "canyon_delay_3",         "ms",            0,    200,   70,     L, 0,  N_, 0,  Ca},
    {kCanyonDelay4,         "canyon_delay_4",         "ms",            0,    200,   95,     L, 0,  N_, 0,  Ca},
    {kCanyonDelay5,         "canyon_delay_5",         "ms",            0,    200,   130,    L, 0,  N_, 0,  Ca},
    {kCanyonDelay6,         "canyon_delay_6",         "ms",            0,    200,   170,    L, 0,  N_, 0,  Ca},
    {kCanyonRate1,          "canyon_rate_1",          "",              0,    100,   6,      L, 20, N_, 0,  Ca},
    {kCanyonRate2,          "canyon_rate_2",          "",              0,    100,   8,      L, 20, N_, 0,  Ca},
    {kCanyonRate3,          "canyon_rate_3",          "",              0,    100,   10,     L, 20, N_, 0,  Ca},
    {kCanyonRate4,          "canyon_rate_4",          "",              0,    100,   12,     L, 20, N_, 0,  Ca},
    {kCanyonRate5,          "canyon_rate_5",          "",              0,    100,   14,     L, 20, N_, 0,  Ca},
    {kCanyonRate6,          "canyon_rate_6",          "",              0,    100,   16,     L, 20, N_, 0,  Ca},
    {kCanyonDepth1,         "canyon_depth_1",         "ms",            0,    40,    5,      L, 20, N_, 0,  Ca},
    {kCanyonDepth2,         "canyon_depth_2",         "ms",            0,    40,    6,      L, 20, N_, 0,  Ca},
    {kCanyonDepth3,         "canyon_depth_3",         "ms",            0,    40,    7,      L, 20, N_, 0,  Ca},
    {kCanyonDepth4,         "canyon_depth_4",         "ms",            0,    40,    8,      L, 20, N_, 0,  Ca},
    {kCanyonDepth5,         "canyon_depth_5",         "ms",            0,    40,    9,      L, 20, N_, 0,  Ca},
    {kCanyonDepth6,         "canyon_depth_6",         "ms",            0,    40,    10,     L, 20, N_, 0,  Ca},

    // The EQ (dsp/eq.h, docs/EQ.md): a five-band parametric (low shelf, three bells, high shelf), MIDI and editor only,
    // off by default. eq_position puts it before the delay (pre: it feeds the delay, both dry paths and the reverb) or
    // after the reverb mix (post). Frequencies are each section's centre (bells) or half-gain frequency (shelves).
    // The third bell is at the end of the table.
    {kEqEnable,             "eq_enable",              "0 off, 1 on",   0,    1,     0,      E, 0,  N_, 0,  Eq},
    {kEqPosition,           "eq_position",            "0 pre, 1 post", 0,    1,     0,      E, 0,  N_, 0,  Eq},
    {kEqLowGain,            "eq_low_gain",            "dB",            -12,  12,    0,      L, 20, N_, 0,  Eq},
    {kEqLowFreq,            "eq_low_freq",            "Hz",            40,   600,   100,    G, 20, N_, 0,  Eq},
    {kEqMidGain1,           "eq_mid_gain_1",          "dB",            -12,  12,    0,      L, 20, N_, 0,  Eq},
    {kEqMidFreq1,           "eq_mid_freq_1",          "Hz",            40,   800,   250,    G, 20, N_, 0,  Eq},
    {kEqMidQ1,              "eq_mid_q_1",             "Q",             0.4f, 8,     1,      G, 20, N_, 0,  Eq},
    {kEqHighGain,           "eq_high_gain",           "dB",            -12,  12,    0,      L, 20, N_, 0,  Eq},
    {kEqHighFreq,           "eq_high_freq",           "Hz",            1500, 16000, 6000,   G, 20, N_, 0,  Eq},
    {kEqMidFreq2,           "eq_mid_freq_2",          "Hz",            200,  5000,  1000,   G, 20, N_, 0,  Eq},
    {kEqMidQ2,              "eq_mid_q_2",             "Q",             0.4f, 8,     1,      G, 20, N_, 0,  Eq},
    {kEqMidGain2,           "eq_mid_gain_2",          "dB",            -12,  12,    0,      L, 20, N_, 0,  Eq},

    // The bitcrush color (SW2 up, knob 6, dsp/crush.h): drive and hold rate at knob 6 fully clockwise (drive scales
    // in from fully counter-clockwise); the anti-alias and anti-imaging cutoffs as fractions of the current hold rate.
    {kDelayCrushDrive,      "delay_crush_drive",      "dB",            0,    24,    12,     L, 20, N_, 0,  De},
    {kDelayCrushRate,       "delay_crush_rate",       "Hz",            1000, 16000, 1000,   G, 20, N_, 0,  De},
    {kDelayCrushAa,         "delay_crush_aa",         "x hold rate",   0.25f, 1,    0.4f,   L, 20, N_, 0,  De},
    {kDelayCrushTone,       "delay_crush_tone",       "x hold rate",   0.1f, 0.5f,  0.4f,   L, 20, N_, 0,  De},

    // The tape color (SW2 middle, knob 6, dsp/tape.h): each value is the effect at full tape age (knob 6 fully
    // clockwise); age scales them all in from nothing fully counter-clockwise.
    {kDelayTapeWow,         "delay_tape_wow",         "ms",            0,    5,     2,      L, 20, N_, 0,  De},
    {kDelayTapeFlutter,     "delay_tape_flutter",     "ms",            0,    0.5f,  0.1f,   L, 20, N_, 0,  De},
    {kDelayTapeDrive,       "delay_tape_drive",       "dB",            0,    18,    9,      L, 20, N_, 0,  De},
    {kDelayTapeTone,        "delay_tape_tone",        "Hz",            1000, 12000, 2500,   G, 20, N_, 0,  De},
    {kDelayTapeBump,        "delay_tape_bump",        "dB",            0,    6,     3,      L, 20, N_, 0,  De},
    {kDelayTapeHiss,        "delay_tape_hiss",        "dBFS RMS",      -90,  -40,   -60,    L, 20, N_, 0,  De},
    {kDelayTapeDropouts,    "delay_tape_dropouts",    "per minute",    0,    30,    6,      L, 0,  N_, 0,  De},

    // Space on the Hall (knob 3 by default): a macro (the 480L's linked REV mode) that writes hall_rt_mid, hall_size,
    // hall_shape and hall_spread along LinkValue() (Links, below); the DSP itself ignores it.
    {kHallSpace,            "hall_space",             "%",             0,    100,   50,     L, 0,  N_, 16, Ha},
    // Space on the Plate and the Canyon: macros like hall_space, writing their algorithm's size, decay and spread
    // along LinkValue() (Links, below); the DSP ignores them too.
    {kPlateSpace,           "plate_space",            "%",             0,    100,   50,     L, 0,  N_, 20, Pl},
    {kCanyonSpace,          "canyon_space",           "%",             0,    100,   50,     L, 0,  N_, 21, Ca},

    // SW2: the delay's character (btctl::DelayType).
    {kDelayType,            "delay_type",             "0 bbd, 1 tape, 2 digital", 0, 2, 1,  E, 0,  S2, 23, De},
    // The bucket brigade (SW2 down, knob 6, dsp/bbd.h): its clock is delay_bbd_stages / (2 x the delay time); knob 6
    // fully clockwise multiplies the clock by delay_bbd_clock, raises the drive by delay_bbd_drive and the noise to
    // delay_bbd_noise (a tenth of it fully counter-clockwise). The filters stay at delay_bbd_tone.
    {kDelayBbdStages,       "delay_bbd_stages",       "stages",        1024, 32768, 16384,  G, 20, N_, 0,  De},
    {kDelayBbdTone,         "delay_bbd_tone",         "Hz",            1000, 12000, 4000,   G, 20, N_, 0,  De},
    {kDelayBbdDrive,        "delay_bbd_drive",        "dB",            0,    24,    12,     L, 20, N_, 0,  De},
    {kDelayBbdNoise,        "delay_bbd_noise",        "dBFS RMS",      -90,  -40,   -60,    L, 20, N_, 0,  De},
    {kDelayBbdClock,        "delay_bbd_clock",        "x clock",       0.05f, 1,    0.25f,  G, 20, N_, 0,  De},
    // The BBD's clock LFO (SW2 down, dsp/bbd.h BbdLfo), after the Deluxe Memory Man: a triangle at delay_bbd_mod_rate
    // swings the clock by +-delay_bbd_mod_depth, so the delay by d / (1 +- depth) and the repeats bend sharp and flat.
    // It doesn't depend on knob 6; the depth's 150 ms smoothing also fades it in and out on an SW2 change.
    {kDelayBbdModRate,      "delay_bbd_mod_rate",     "Hz",            0.1f, 10,    0.2f,   G, 0,  N_, 0,  De},
    {kDelayBbdModDepth,     "delay_bbd_mod_depth",    "% clock",       0,    25,    0.5f,   L, 150, N_, 0,  De},

    // The knob slots (param_store.h): what knob N drives without FS2 (primary) and with FS2 held (secondary). An
    // assignment holds a parameter id; kNoParam (255) or any id that is not assignable (IsAssignable) means none, and
    // any of the three *_space ids means the active algorithm's Space. Factory secondaries: knob 4 (delay time) drives
    // the sound-on-sound length with FS2 held; the others none.
    {kKnob1Primary,         "knob_1_primary",         "param id (255 = none)", 0, kNoParam, kDelayMix,      E, 0, N_, 0, Kn},
    {kKnob2Primary,         "knob_2_primary",         "param id (255 = none)", 0, kNoParam, kReverbMix,     E, 0, N_, 0, Kn},
    {kKnob3Primary,         "knob_3_primary",         "param id (255 = none)", 0, kNoParam, kHallSpace,     E, 0, N_, 0, Kn},
    {kKnob4Primary,         "knob_4_primary",         "param id (255 = none)", 0, kNoParam, kDelayTime,     E, 0, N_, 0, Kn},
    {kKnob5Primary,         "knob_5_primary",         "param id (255 = none)", 0, kNoParam, kDelayFeedback, E, 0, N_, 0, Kn},
    {kKnob6Primary,         "knob_6_primary",         "param id (255 = none)", 0, kNoParam, kDelayColor,    E, 0, N_, 0, Kn},
    {kKnob1Secondary,       "knob_1_secondary",       "param id (255 = none)", 0, kNoParam, kNoParam,       E, 0, N_, 0, Kn},
    {kKnob2Secondary,       "knob_2_secondary",       "param id (255 = none)", 0, kNoParam, kNoParam,       E, 0, N_, 0, Kn},
    {kKnob3Secondary,       "knob_3_secondary",       "param id (255 = none)", 0, kNoParam, kNoParam,       E, 0, N_, 0, Kn},
    {kKnob4Secondary,       "knob_4_secondary",       "param id (255 = none)", 0, kNoParam, kDelaySosLength, E, 0, N_, 0, Kn},
    {kKnob5Secondary,       "knob_5_secondary",       "param id (255 = none)", 0, kNoParam, kNoParam,       E, 0, N_, 0, Kn},
    {kKnob6Secondary,       "knob_6_secondary",       "param id (255 = none)", 0, kNoParam, kNoParam,       E, 0, N_, 0, Kn},

    // The digital color's options (SW2 up, knob 6, dsp/crush.h): what knob 6 does besides the hold rate, bit depth
    // and drive. open: the anti-alias and anti-imaging filters open as knob 6 turns, to a bare sample-and-hold fully
    // clockwise. clock: delay_crush_rate is the hold rate at a 1 s delay, scaled by 1 s / delay_time. wrap: overload
    // wraps to the opposite rail instead of clipping. bits: bit inversion after the crush, never fed back (on the wet
    // output with delay_crush_scope repeats, on the crushed input with all): the bits of delay_crush_mask (bit 7 = the
    // MSB) inverted in an 8-bit word, mixed in with knob 6.
    {kDelayCrushOpen,       "delay_crush_open",       "0 off, 1 on",   0,    1,     1,      E, 20, N_, 0,  De},
    {kDelayCrushClock,      "delay_crush_clock",      "0 off, 1 on",   0,    1,     0,      E, 20, N_, 0,  De},
    {kDelayCrushWrap,       "delay_crush_wrap",       "0 off, 1 on",   0,    1,     1,      E, 20, N_, 0,  De},
    {kDelayCrushBits,       "delay_crush_bits",       "0 off, 1 on",   0,    1,     0,      E, 20, N_, 0,  De},
    {kDelayCrushMask,       "delay_crush_mask",       "bit mask (bit 7 = MSB)", 0, 255, 12, E, 0, N_, 0, De},
    // Where the digital color (SW2 up, knob 6) sits: repeats, in the feedback loop, so each repeat is crushed again;
    // all, on the chain's input (after a pre EQ), so the dry, the repeats (once: the loop stays clean) and every reverb
    // input are crushed. Bypass passes the raw input either way.
    {kDelayCrushScope,      "delay_crush_scope",      "0 repeats, 1 all", 0, 1,     1,      E, 20, N_, 0,  De},
    // Where the tape color (SW2 middle, knob 6) sits: repeats, in the feedback loop (bump and hiss on the wet output),
    // so each repeat is colored again; all, on the chain's input (after a pre EQ), so the dry, the repeats (once: the
    // loop stays clean) and every reverb input are colored. Wow and flutter stay on the repeats either way and also
    // wobble the input in all. Bypass passes the raw input either way.
    {kDelayTapeScope,       "delay_tape_scope",       "0 repeats, 1 all", 0, 1,     1,      E, 20, N_, 0,  De},
    // The same for the bucket brigade (SW2 down, knob 6); its clock LFO stays on the repeats.
    {kDelayBbdScope,        "delay_bbd_scope",        "0 repeats, 1 all", 0, 1,     1,      E, 20, N_, 0,  De},

    // The EQ's third bell (see the EQ block above).
    {kEqMidGain3,           "eq_mid_gain_3",          "dB",            -12,  12,    0,      L, 20, N_, 0,  Eq},
    {kEqMidFreq3,           "eq_mid_freq_3",          "Hz",            800,  12000, 3000,   G, 20, N_, 0,  Eq},
    {kEqMidQ3,              "eq_mid_q_3",             "Q",             0.4f, 8,     1,      G, 20, N_, 0,  Eq},

    // Sound on sound (dsp/delay.h): a second, long line beside the delay's, read by one more head into the delay's
    // wet. delay_sos_mode picks what that head is (btctl::SosMode); delay_sos switches it on and off (FS2 tap, CC 26).
    // 301: a head delay_sos_length back, played once. 501: a head at delay_sos_ratio x the delay time, fed back into
    // the delay loop at the delay feedback. overdub: the line keeps delay_sos_feedback of what it played
    // delay_sos_length ago under the new input. delay_sos_level is the head's output level.
    {kDelaySosMode,         "delay_sos_mode",         "0 off, 1 301, 2 501, 3 overdub", 0, 3, 1, E, 0, N_, 0, De},
    {kDelaySos,             "delay_sos",              "0 off, 1 on",   0,    1,     0,      E, 0,  F2, 26, De},
    {kDelaySosLength,       "delay_sos_length",       "s",             10,   120,   15,     G, 0,  N_, 0,  De},
    {kDelaySosRatio,        "delay_sos_ratio",        "x delay time",  1.25f, 4,    2.5f,   L, 0,  N_, 0,  De},
    {kDelaySosFeedback,     "delay_sos_feedback",     "%",             0,    100,   90,     L, 20, N_, 0,  De},
    {kDelaySosLevel,        "delay_sos_level",        "%",             0,    100,   100,    L, 20, N_, 0,  De},
};
// clang-format on
}  // namespace detail
using detail::kParams;

constexpr bool IdsMatch() {
    for (size_t i = 0; i < std::size(kParams); ++i)
        if (kParams[i].id != i) return false;
    return true;
}
static_assert(std::size(kParams) == kParamCount, "kParams and ParamId disagree");
static_assert(IdsMatch(), "kParams[i].id must equal i");

constexpr uint16_t kCodeMax = 16383;  // 14-bit NRPN / SysEx code

// v limited to [min, max]; enums rounded to the nearest integer.
inline float Clamp(const ParamDef& d, float v) {
    v = v < d.min ? d.min : v > d.max ? d.max : v;
    return d.taper == Taper::kEnum ? std::round(v) : v;
}

// The parameter's DSP smoothing time constant (smoothMs) in seconds.
constexpr float SmoothSeconds(uint8_t id) { return kParams[id].smoothMs * 1e-3f; }

// v (clamped first) as 0..1 along the taper.
inline float ToNorm(const ParamDef& d, float v) {
    v = Clamp(d, v);
    if (d.taper == Taper::kLog) return std::log(v / d.min) / std::log(d.max / d.min);
    if (d.taper == Taper::kAudio) return std::log2(1.0f + 31.0f * (v - d.min) / (d.max - d.min)) / 5.0f;
    return (v - d.min) / (d.max - d.min);
}

// 0..1 (clamped) along the taper; exactly min at 0 and max at 1.
inline float FromNorm(const ParamDef& d, float n) {
    if (!(n > 0.0f)) return d.min;
    if (n >= 1.0f) return d.max;
    switch (d.taper) {
        case Taper::kLog: return d.min * std::pow(d.max / d.min, n);
        case Taper::kAudio: return d.min + (d.max - d.min) * (std::exp2(5.0f * n) - 1.0f) / 31.0f;
        case Taper::kEnum: return d.min + std::round(n * (d.max - d.min));
        default: return d.min + n * (d.max - d.min);
    }
}

inline uint16_t ToCode(const ParamDef& d, float v) { return uint16_t(std::lround(ToNorm(d, v) * kCodeMax)); }
inline float FromCode(const ParamDef& d, uint16_t c) {
    return FromNorm(d, float(c < kCodeMax ? c : kCodeMax) / float(kCodeMax));
}
// 7-bit CC; for a two-value enum 0-63 / 64-127 give 0 / 1.
inline float FromCc(const ParamDef& d, uint8_t c) { return FromNorm(d, float(c < 127 ? c : 127) / 127.0f); }

inline int FindByName(const char* name) {
    for (const ParamDef& d : kParams)
        if (std::strcmp(d.name, name) == 0) return d.id;
    return -1;
}
inline int FindByCc(uint8_t cc) {
    if (cc == 0) return -1;
    for (const ParamDef& d : kParams)
        if (d.cc == cc) return d.id;
    return -1;
}

// The parameter toggle t (0-2: SW1-SW3) sets.
inline int ToggleParam(int t) { return t == 0 ? kDelayMode : t == 1 ? kDelayType : kReverbAlgo; }

constexpr int kKnobs = 6, kSlots = 12;  // slot s: 0-5 primary of knob s+1, 6-11 FS2 secondary of knob s-5
static_assert(kKnob1Secondary == kKnob1Primary + kKnobs && kKnob6Secondary == kKnob1Primary + kSlots - 1,
              "slots are contiguous");
// The assignment parameter of slot s.
constexpr uint8_t SlotAssignId(int slot) { return uint8_t(kKnob1Primary + slot); }
// One of the three algorithms' Space macros.
inline bool IsSpace(uint8_t id) { return id == kHallSpace || id == kPlateSpace || id == kCanyonSpace; }
// The Space macro of algorithm a.
inline uint8_t SpaceParam(btctl::Algo a) {
    return a == btctl::Algo::kPlate ? kPlateSpace : a == btctl::Algo::kCanyon ? kCanyonSpace : kHallSpace;
}
// Not on a switch or footswitch: the parameter has a saved power-up value (saved_store.h).
constexpr bool IsSaveable(uint8_t id) { return id < kParamCount && kParams[id].control == Control::kNone; }
// The number of saveable parameters (SAVED END's count; the longest SAVE).
constexpr int kSaveableCount = [] {
    int n = 0;
    for (uint8_t id = 0; id < kParamCount; ++id) n += IsSaveable(id);
    return n;
}();
// Saveable and not an assignment.
constexpr bool IsAssignable(uint8_t id) { return IsSaveable(id) && kParams[id].group != Group::kKnobs; }
// The parameter an assignment value drives while `active` is the reverb algorithm, or kNoParam.
inline uint8_t ResolveSlot(float assignment, btctl::Algo active) {
    const long v = std::lround(assignment);
    if (v < 0 || v >= kParamCount || !IsAssignable(uint8_t(v))) return kNoParam;
    return IsSpace(uint8_t(v)) ? SpaceParam(active) : uint8_t(v);
}

// Links: a macro parameter (source) whose change writes other parameters (targets) along a law. The store applies
// them (param_store.h): a target written over MIDI is held until its source moves more than kPickup of its travel.
// The laws are LinkValue(), documented by each entry's law, with n = source / 100.
// Predelay, the Canyon's master rate and its per-line settings are left out: a predelay glide bends the pitch, the
// rate is the breathing character, and the masters already scale the lines.
struct Link {
    uint8_t source, target;
    const char* law;  // the target as a function of n, with its unit (docs/MIDI.md prints it)
};
// The Hall's four first, in this order (tests/param_store_test.cpp indexes them).
inline constexpr Link kLinks[] = {
    {kHallSpace, kHallRtMid, "0.4 * 25^n s"},
    {kHallSpace, kHallSize, "39 - 36 (1 - n)^2 m"},
    {kHallSpace, kHallShape, "35 + 30 n %"},
    {kHallSpace, kHallSpread, "20 + 60 n %"},
    {kPlateSpace, kPlateSize, "180 n - 80 n^2 %"},
    {kPlateSpace, kPlateMidDecay, "0.36 * 25^n s"},
    {kPlateSpace, kPlateLfDecay, "0.36 * 25^n s"},
    {kPlateSpace, kPlateDepth, "20 + 60 n %"},
    {kCanyonSpace, kCanyonMasterDelay, "30 + 110 n - 40 n^2 %"},
    {kCanyonSpace, kCanyonFeedback, "100 - 40 (1 - n)^3 %"},
    {kCanyonSpace, kCanyonMasterDepth, "20 + 60 n %"}};

// The source id parameter is a target of, or -1.
inline int LinkSource(uint8_t id) {
    for (const Link& l : kLinks)
        if (l.target == id) return l.source;
    return -1;
}
inline bool IsLinkSource(uint8_t id) {
    for (const Link& l : kLinks)
        if (l.source == id) return true;
    return false;
}

// The value link l gives its target when its source is at sourceValue (Clamp()ed to the target's range).
inline float LinkValue(const Link& l, float sourceValue) {
    const float n = ToNorm(kParams[l.source], sourceValue);
    float v = kParams[l.target].def;
    switch (l.target) {
        case kHallSize: v = 39.0f - 36.0f * (1.0f - n) * (1.0f - n); break;
        case kHallRtMid: v = 0.4f * std::pow(25.0f, n); break;
        case kHallSpread: v = 20.0f + 60.0f * n; break;
        case kHallShape: v = 35.0f + 30.0f * n; break;
        case kPlateSize: v = 180.0f * n - 80.0f * n * n; break;
        case kPlateMidDecay:
        case kPlateLfDecay: v = 0.36f * std::pow(25.0f, n); break;
        case kPlateDepth: v = 20.0f + 60.0f * n; break;
        case kCanyonMasterDelay: v = 30.0f + 110.0f * n - 40.0f * n * n; break;
        case kCanyonFeedback: v = 100.0f - 40.0f * (1.0f - n) * (1.0f - n) * (1.0f - n); break;
        case kCanyonMasterDepth: v = 20.0f + 60.0f * n; break;
        default: break;
    }
    return Clamp(kParams[l.target], v);
}

}  // namespace btparam
