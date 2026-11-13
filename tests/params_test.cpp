// daisy/src/params.h: the table's invariants (ids, names, ranges, controls, CCs), the delay_time taper, and the
// code/CC round trips the MIDI and SysEx protocols rely on.
#include "params.h"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <set>
#include <string>

using namespace btparam;

TEST(Table, IdsNamesRanges) {
    std::set<std::string> names;
    for (size_t i = 0; i < kParamCount; ++i) {
        const ParamDef& d = kParams[i];
        SCOPED_TRACE(d.name);
        EXPECT_EQ(d.id, i);
        EXPECT_TRUE(names.insert(d.name).second) << "duplicate name";
        EXPECT_FALSE(std::string(d.name).empty());
        for (const char* c = d.name; *c; ++c) {
            EXPECT_TRUE((*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9') || *c == '_');
        }
        EXPECT_LT(d.min, d.max);
        EXPECT_GE(d.def, d.min);
        EXPECT_LE(d.def, d.max);
        if (d.taper == Taper::kLog) {
            EXPECT_GT(d.min, 0.0f);
        }
        if (d.taper == Taper::kEnum) {
            EXPECT_EQ(d.min, std::round(d.min));
            EXPECT_EQ(d.max, std::round(d.max));
            EXPECT_EQ(d.def, std::round(d.def));
        }
    }
}

TEST(Table, ControlsAndCcs) {
    std::set<int> ccs;
    std::map<Control, int> uses;
    for (const ParamDef& d : kParams) {
        SCOPED_TRACE(d.name);
        if (d.cc != 0) {
            EXPECT_TRUE(ccs.insert(d.cc).second) << "duplicate CC " << int(d.cc);
            EXPECT_GE(d.cc, 14);
            EXPECT_LE(d.cc, 31);
        }
        if (d.control != Control::kNone) {
            EXPECT_NE(d.cc, 0) << "controlled parameter without a CC";
            ++uses[d.control];
        }
    }
    for (Control c : {Control::kSw1, Control::kSw2, Control::kSw3, Control::kFs1})
        EXPECT_EQ(uses[c], 1) << "control " << int(c);
}

TEST(DelayTime, EndsAndMidpoint) {
    const ParamDef& d = kParams[kDelayTime];
    EXPECT_EQ(FromNorm(d, 0.0f), 10.0f);
    EXPECT_EQ(FromNorm(d, 1.0f), 2000.0f);
    EXPECT_NEAR(FromNorm(d, 0.5f), 308.94f, 0.01f);  // audio taper: 15 % of the range at mid-travel
}

TEST(DelayTime, ClampsOutOfRangeKnob) {
    const ParamDef& d = kParams[kDelayTime];
    EXPECT_EQ(FromNorm(d, -0.1f), 10.0f);
    EXPECT_EQ(FromNorm(d, 1.2f), 2000.0f);
}

TEST(DelayTime, StrictlyIncreasing) {
    const ParamDef& d = kParams[kDelayTime];
    float prev = FromNorm(d, 0.0f);
    for (int i = 1; i <= 100; ++i) {
        const float t = FromNorm(d, i / 100.0f);
        EXPECT_GT(t, prev) << "at n = " << i / 100.0f;
        prev = t;
    }
}

// What NRPN and SysEx SET/GET rely on: a code decodes to a value that encodes back to the same code (continuous), and
// every enum value survives the trip.
TEST(Code, RoundTrip) {
    for (const ParamDef& d : kParams) {
        SCOPED_TRACE(d.name);
        if (d.taper == Taper::kEnum) {
            for (int v = int(d.min); v <= int(d.max); ++v) EXPECT_EQ(FromCode(d, ToCode(d, float(v))), float(v));
            continue;
        }
        int bad = 0;
        for (int c = 0; c <= kCodeMax; ++c)
            if (ToCode(d, FromCode(d, uint16_t(c))) != c && ++bad <= 3) ADD_FAILURE() << "code " << c;
        EXPECT_EQ(bad, 0);
    }
}

TEST(Code, EndsAndClamp) {
    for (const ParamDef& d : kParams) {
        SCOPED_TRACE(d.name);
        EXPECT_EQ(FromCode(d, 0), d.min);
        EXPECT_EQ(FromCode(d, kCodeMax), d.max);
        EXPECT_EQ(FromCode(d, 0xFFFF), d.max);
        EXPECT_EQ(ToCode(d, d.max + 1000.0f), kCodeMax);
        EXPECT_EQ(ToCode(d, d.min - 1000.0f), 0);
    }
}

TEST(Cc, Ends) {
    for (const ParamDef& d : kParams) {
        SCOPED_TRACE(d.name);
        EXPECT_EQ(FromCc(d, 0), d.min);
        EXPECT_EQ(FromCc(d, 127), d.max);
    }
}

TEST(Cc, TwoValueEnumSplitsAt64) {
    const ParamDef& d = kParams[kBypass];
    EXPECT_EQ(FromCc(d, 63), 0.0f);
    EXPECT_EQ(FromCc(d, 64), 1.0f);
}

TEST(Lookup, NameCcSlotToggle) {
    EXPECT_EQ(FindByName("delay_time"), kDelayTime);
    EXPECT_EQ(FindByName("nope"), -1);
    EXPECT_EQ(FindByCc(17), kDelayTime);
    EXPECT_EQ(FindByCc(0), -1);
    EXPECT_EQ(FindByCc(99), -1);
    EXPECT_EQ(FindByCc(20), kPlateSpace);
    EXPECT_EQ(ResolveSlot(kHallSpace, btctl::Algo::kPlate), kPlateSpace);
    EXPECT_EQ(ResolveSlot(kCanyonSpace, btctl::Algo::kHall), kHallSpace);
    EXPECT_EQ(ResolveSlot(kDelayTime, btctl::Algo::kHall), kDelayTime);
    EXPECT_EQ(ResolveSlot(kBypass, btctl::Algo::kHall), kNoParam);
    EXPECT_EQ(ResolveSlot(kKnob2Primary, btctl::Algo::kHall), kNoParam);
    EXPECT_EQ(ResolveSlot(kNoParam, btctl::Algo::kHall), kNoParam);
    EXPECT_EQ(ToggleParam(1), kDelayType);
    EXPECT_EQ(ToggleParam(2), kReverbAlgo);
}

TEST(Lookup, FactoryLayout) {
    const uint8_t expected[kSlots] = {kDelayMix, kReverbMix, kHallSpace, kDelayTime, kDelayFeedback, kDelayColor,
                                      kNoParam,  kNoParam,   kNoParam,   kNoParam,   kNoParam,       kNoParam};
    for (int s = 0; s < kSlots; ++s)
        EXPECT_EQ(ResolveSlot(kParams[SlotAssignId(s)].def, btctl::Algo::kHall), expected[s]) << "slot " << s;
}

// The pedal boots with every parameter at its default and the knob then moves the source: the source's default must
// give each target its default, or a render and the pedal at default settings disagree.
TEST(Links, SourceDefaultGivesTargetDefaults) {
    for (const Link& l : kLinks) {
        SCOPED_TRACE(kParams[l.target].name);
        EXPECT_NEAR(LinkValue(l, kParams[l.source].def), kParams[l.target].def, 1e-4f * kParams[l.target].max);
    }
}

// Every macro runs small/short -> large/long: no target ever falls as its source rises.
TEST(Links, LawsRiseWithSource) {
    for (const Link& l : kLinks) {
        SCOPED_TRACE(kParams[l.target].name);
        for (int v = 1; v <= 100; ++v) EXPECT_GE(LinkValue(l, float(v)), LinkValue(l, float(v - 1))) << v;
        EXPECT_GT(LinkValue(l, 100.0f), LinkValue(l, 0.0f));
    }
}
