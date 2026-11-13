// daisy/src/param_store.h: knob/MIDI arbitration (pickup), toggle precedence, Space following SW3, links, knob
// remapping and the FS2 secondaries.
#include "param_store.h"

#include <gtest/gtest.h>

using namespace btparam;

namespace {

constexpr uint8_t kMid = 1, kDown = 0;

struct Rig {
    Store s;
    float knobs[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    uint8_t toggles[3] = {2, kMid, kMid};  // forward, tape, hall
    bool shift = false;
    Rig() {
        s.Init();
        s.Update(knobs, toggles, shift);
    }
    uint16_t Update() { return s.Update(knobs, toggles, shift); }
};

}  // namespace

TEST(Store, KnobSetsItsParameter) {
    Rig r;
    EXPECT_NEAR(r.s.Value(kDelayTime), 308.94f, 0.01f);
    EXPECT_EQ(r.s.Value(kDelayMode), 2.0f);
    EXPECT_EQ(r.s.Value(kReverbAlgo), 1.0f);
}

TEST(Store, MidiHoldsUntilKnobMovesPastPickup) {
    Rig r;
    EXPECT_EQ(r.s.SetFromMidi(kDelayTime, 500.0f, r.knobs), 500.0f);
    r.knobs[3] = 0.529f;
    EXPECT_EQ(r.Update(), 0);
    EXPECT_EQ(r.s.Value(kDelayTime), 500.0f);
    r.knobs[3] = 0.531f;
    EXPECT_EQ(r.Update(), 1u << 3);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayTime), FromNorm(kParams[kDelayTime], 0.531f));
    r.knobs[3] = 0.5f;  // released: the knob tracks again
    EXPECT_EQ(r.Update(), 0);
    EXPECT_NEAR(r.s.Value(kDelayTime), 308.94f, 0.01f);
}

TEST(Store, MidiWriteIsClamped) {
    Rig r;
    EXPECT_EQ(r.s.SetFromMidi(kDelayTime, 5000.0f, r.knobs), 2000.0f);
    EXPECT_EQ(r.s.SetFromMidi(kRouting, 1.4f, r.knobs), 1.0f);
}

TEST(Store, ToggleChangeOverridesMidiUnchangedKeepsIt) {
    Rig r;
    r.s.SetFromMidi(kDelayType, 0.0f, r.knobs);
    r.s.SetFromMidi(kRouting, 0.0f, r.knobs);
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayType), 0.0f);  // SW2 unchanged: MIDI value stays
    r.toggles[1] = 2;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayType), 2.0f);  // SW2 moved: it wins
    EXPECT_EQ(r.s.Value(kRouting), 0.0f);    // and leaves the routing alone (MIDI only)
}

TEST(Store, Sw3RetargetsKnob3) {
    Rig r;
    const float hall = r.s.Value(kHallSpace), rt = r.s.Value(kHallRtMid);
    r.toggles[2] = kDown;  // plate
    r.knobs[2] = 0.9f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kPlateSpace), 90.0f);
    EXPECT_EQ(r.s.Value(kPlateMidDecay), LinkValue({kPlateSpace, kPlateMidDecay, ""}, 90.0f));
    EXPECT_EQ(r.s.Value(kHallSpace), hall);
    EXPECT_EQ(r.s.Value(kHallRtMid), rt);
}

TEST(Store, HeldKnob3ParameterSurvivesActivation) {
    Rig r;  // hall active
    r.s.SetFromMidi(kPlateSpace, 70.0f, r.knobs);
    r.knobs[2] = 0.8f;  // moves knob 3 while hall is active: hall follows, plate stays held
    r.Update();
    EXPECT_EQ(r.s.Value(kPlateSpace), 70.0f);
    r.toggles[2] = kDown;  // activate plate: re-anchored at 0.8
    EXPECT_EQ(r.Update(), 0);
    EXPECT_EQ(r.s.Value(kPlateSpace), 70.0f);
    r.knobs[2] = 0.825f;
    EXPECT_EQ(r.Update(), 0);
    EXPECT_EQ(r.s.Value(kPlateSpace), 70.0f);
    r.knobs[2] = 0.835f;
    EXPECT_EQ(r.Update(), 1u << 2);
    EXPECT_FLOAT_EQ(r.s.Value(kPlateSpace), FromNorm(kParams[kPlateSpace], 0.835f));
}

TEST(Store, BypassToggles) {
    Rig r;
    EXPECT_EQ(r.s.Value(kBypass), 0.0f);
    r.s.ToggleBypass();
    EXPECT_EQ(r.s.Value(kBypass), 1.0f);
    r.s.ToggleBypass();
    EXPECT_EQ(r.s.Value(kBypass), 0.0f);
}

TEST(Store, Knob3DrivesHallSpaceTargets) {
    Rig r;  // hall active
    r.knobs[2] = 0.8f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kHallSpace), 80.0f);
    for (const Link& l : kLinks) {
        if (l.source == kHallSpace) {
            EXPECT_EQ(r.s.Value(l.target), LinkValue(l, 80.0f)) << kParams[l.target].name;
        }
    }
}

TEST(Store, Knob3DrivesPlateAndCanyonSpaceTargets) {
    Rig r;
    r.toggles[2] = kDown;  // plate
    r.knobs[2] = 0.8f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kPlateSpace), 80.0f);
    for (const Link& l : kLinks) {
        if (l.source == kPlateSpace) {
            EXPECT_EQ(r.s.Value(l.target), LinkValue(l, 80.0f)) << kParams[l.target].name;
        }
    }
    r.toggles[2] = 2;  // canyon
    r.knobs[2] = 0.3f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kCanyonSpace), 30.0f);
    for (const Link& l : kLinks) {
        if (l.source == kCanyonSpace) {
            EXPECT_EQ(r.s.Value(l.target), LinkValue(l, 30.0f)) << kParams[l.target].name;
        } else if (l.source == kPlateSpace) {
            EXPECT_EQ(r.s.Value(l.target), LinkValue(l, 80.0f)) << kParams[l.target].name;
        }
    }
}

TEST(Store, MidiTargetHeldUntilSourceMovesPastPickup) {
    Rig r;  // knob 3 at 0.5
    r.s.SetFromMidi(kHallRtMid, 7.0f, r.knobs);
    r.knobs[2] = 0.529f;
    r.Update();
    EXPECT_EQ(r.s.Value(kHallRtMid), 7.0f);  // held
    EXPECT_EQ(r.s.Value(kHallSize), LinkValue(kLinks[1], r.s.Value(kHallSpace)));  // the others follow
    r.knobs[2] = 0.531f;
    r.Update();
    EXPECT_EQ(r.s.Value(kHallRtMid), LinkValue(kLinks[0], r.s.Value(kHallSpace)));  // taken back
}

TEST(Store, MidiSourceWriteRelinksAndLaterTargetWriteWins) {
    Rig r;
    r.s.SetFromMidi(kHallSpace, 100.0f, r.knobs);
    EXPECT_EQ(r.s.Value(kHallRtMid), 10.0f);
    EXPECT_EQ(r.s.Value(kHallSize), 39.0f);
    r.s.SetFromMidi(kHallRtMid, 3.0f, r.knobs);
    EXPECT_EQ(r.s.Value(kHallRtMid), 3.0f);
    r.Update();  // knob 3 unmoved: neither the source nor the target changes
    EXPECT_EQ(r.s.Value(kHallSpace), 100.0f);
    EXPECT_EQ(r.s.Value(kHallRtMid), 3.0f);
}

TEST(Store, SavedLinkTargetHoldsUntilKnob3MovesPastPickup) {
    Store s;
    s.Init();
    s.Load(kHallRtMid, 7.0f);
    s.Load(kHallPredelay, 100.0f);
    float knobs[6] = {0.5f, 0.5f, 0.8f, 0.5f, 0.5f, 0.5f};
    const uint8_t toggles[3] = {2, kMid, kMid};  // hall
    s.Update(knobs, toggles, false);  // anchors at the knob position this first Update reads
    EXPECT_EQ(s.Value(kHallRtMid), 7.0f);
    EXPECT_EQ(s.Value(kHallSize), LinkValue(kLinks[1], s.Value(kHallSpace)));  // an unsaved target follows
    knobs[2] = 0.82f;
    s.Update(knobs, toggles, false);
    EXPECT_EQ(s.Value(kHallRtMid), 7.0f);
    knobs[2] = 0.84f;
    s.Update(knobs, toggles, false);
    EXPECT_EQ(s.Value(kHallRtMid), LinkValue(kLinks[0], s.Value(kHallSpace)));
    EXPECT_EQ(s.Value(kHallPredelay), 100.0f);  // a parameter on no slot and no link keeps its saved value
}

TEST(Store, Fs2TurnLeavesPrimaryUntilKnobMovesAgain) {
    Rig r;
    r.s.SetFromMidi(kKnob1Secondary, kHallPredelay, r.knobs);
    r.Update();
    r.shift = true;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.knobs[0] = 0.3f;
    EXPECT_EQ(r.Update(), 1u << 6);
    EXPECT_FLOAT_EQ(r.s.Value(kHallPredelay), FromNorm(kParams[kHallPredelay], 0.3f));
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.shift = false;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.knobs[0] = 0.32f;
    EXPECT_EQ(r.Update(), 0);
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.knobs[0] = 0.34f;
    EXPECT_EQ(r.Update(), 1u << 0);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.34f));
    EXPECT_FLOAT_EQ(r.s.Value(kHallPredelay), FromNorm(kParams[kHallPredelay], 0.3f));
}

TEST(Store, Fs2WithoutMovementKeepsWysiwyg) {
    Rig r;
    r.shift = true;
    r.Update();
    r.knobs[0] = 0.52f;
    r.Update();
    r.shift = false;
    EXPECT_EQ(r.Update(), 0);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.52f));
    r.knobs[0] = 0.51f;
    EXPECT_EQ(r.Update(), 0);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.51f));
}

TEST(Store, Fs2SecondaryJumpsOnlyPastPickup) {
    Rig r;
    r.s.SetFromMidi(kKnob1Secondary, kHallPredelay, r.knobs);
    r.Update();
    const float before = r.s.Value(kHallPredelay);
    r.shift = true;
    r.Update();
    r.knobs[0] = 0.52f;
    EXPECT_EQ(r.Update(), 0);
    EXPECT_EQ(r.s.Value(kHallPredelay), before);
    r.knobs[0] = 0.54f;
    EXPECT_EQ(r.Update(), 1u << 6);
    EXPECT_FLOAT_EQ(r.s.Value(kHallPredelay), FromNorm(kParams[kHallPredelay], 0.54f));
}

TEST(Store, Fs2KnobWithNoSecondaryStillFreezesPrimary) {
    Rig r;
    r.shift = true;
    r.Update();
    r.knobs[0] = 0.3f;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.shift = false;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.knobs[0] = 0.32f;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.knobs[0] = 0.34f;
    EXPECT_EQ(r.Update(), 1u << 0);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.34f));
}

TEST(Store, RemapHoldsNewParameterUntilKnobMoves) {
    Rig r;
    r.s.SetFromMidi(kKnob1Primary, kHallDiffusion, r.knobs);
    r.Update();
    EXPECT_EQ(r.s.SlotParam(0), kHallDiffusion);
    EXPECT_EQ(r.s.Value(kHallDiffusion), 70.0f);
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
    r.knobs[0] = 0.54f;
    EXPECT_EQ(r.Update(), 1u << 0);
    EXPECT_FLOAT_EQ(r.s.Value(kHallDiffusion), FromNorm(kParams[kHallDiffusion], 0.54f));
    r.knobs[0] = 0.9f;
    r.Update();
    EXPECT_EQ(r.s.Value(kDelayMix), 50.0f);
}

TEST(Store, DuplicatePrimariesLastMovedWins) {
    Rig r;
    r.knobs[0] = 0.2f;
    r.knobs[3] = 0.8f;
    r.s.SetFromMidi(kKnob4Primary, kDelayMix, r.knobs);
    r.Update();
    const float v = r.s.Value(kDelayMix);
    EXPECT_FLOAT_EQ(v, FromNorm(kParams[kDelayMix], 0.2f));
    for (int i = 0; i < 5; ++i) {
        r.Update();
        EXPECT_EQ(r.s.Value(kDelayMix), v);
    }
    r.knobs[3] = 0.85f;
    EXPECT_EQ(r.Update(), 1u << 3);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.85f));
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.85f));
    r.knobs[0] = 0.25f;
    EXPECT_EQ(r.Update(), 1u << 0);
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.25f));
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kDelayMix), FromNorm(kParams[kDelayMix], 0.25f));
}

TEST(Store, LinkTargetOnSlotIsNotWrittenByMacro) {
    Rig r;
    r.s.SetFromMidi(kKnob5Secondary, kHallRtMid, r.knobs);
    r.Update();
    r.shift = true;
    r.Update();
    r.knobs[4] = 0.8f;
    r.Update();
    const float rt = r.s.Value(kHallRtMid);
    EXPECT_FLOAT_EQ(rt, FromNorm(kParams[kHallRtMid], 0.8f));
    r.shift = false;
    r.Update();
    r.knobs[2] = 0.8f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kHallSpace), 80.0f);
    EXPECT_EQ(r.s.Value(kHallRtMid), rt);
    EXPECT_EQ(r.s.Value(kHallSize), LinkValue(kLinks[1], 80.0f));
}

TEST(Store, SpaceOnAnotherKnobFollowsSw3) {
    Rig r;
    r.s.SetFromMidi(kKnob1Primary, kHallSpace, r.knobs);
    r.s.SetFromMidi(kKnob3Primary, kNoParam, r.knobs);
    r.Update();
    r.knobs[0] = 0.8f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kHallSpace), 80.0f);
    r.knobs[2] = 0.1f;  // knob 3 drives nothing now
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kHallSpace), 80.0f);
    r.toggles[2] = kDown;  // plate
    r.Update();
    r.knobs[0] = 0.3f;
    r.Update();
    EXPECT_FLOAT_EQ(r.s.Value(kPlateSpace), 30.0f);
    EXPECT_FLOAT_EQ(r.s.Value(kHallSpace), 80.0f);
}

TEST(Store, NoneAssignmentDoesNothing) {
    Rig r;
    r.s.SetFromMidi(kKnob2Primary, kNoParam, r.knobs);
    r.Update();
    float before[kParamCount];
    for (uint8_t id = 0; id < kParamCount; ++id) before[id] = r.s.Value(id);
    r.knobs[1] = 0.9f;
    EXPECT_EQ(r.Update(), 0);
    for (uint8_t id = 0; id < kParamCount; ++id) EXPECT_EQ(r.s.Value(id), before[id]) << kParams[id].name;
}
