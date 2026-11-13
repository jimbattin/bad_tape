// daisy/src/controls.h: the toggle position mapping and hysteresis, the FS2 shift debounce.
#include "controls.h"

#include <gtest/gtest.h>

using namespace btctl;

TEST(Positions, MapToSwitchMeanings) {
    EXPECT_EQ(ToDelayMode(2), DelayMode::kForward);
    EXPECT_EQ(ToDelayMode(1), DelayMode::kOff);
    EXPECT_EQ(ToDelayMode(0), DelayMode::kReverse);
    EXPECT_EQ(ToDelayType(0), DelayType::kBbd);
    EXPECT_EQ(ToDelayType(1), DelayType::kTape);
    EXPECT_EQ(ToDelayType(2), DelayType::kDigital);
    EXPECT_EQ(ToAlgo(0), Algo::kPlate);
    EXPECT_EQ(ToAlgo(1), Algo::kHall);
    EXPECT_EQ(ToAlgo(2), Algo::kCanyon);
}

TEST(Positions, OutOfRangeIsMiddle) {
    EXPECT_EQ(ToDelayMode(3), DelayMode::kOff);
    EXPECT_EQ(ToDelayType(3), DelayType::kTape);
    EXPECT_EQ(ToAlgo(3), Algo::kHall);
}

TEST(ToggleFilter, UpToDownNeverReportsMiddle) {
    ToggleFilter f;
    f.Init(2);
    for (int i = 0; i < 10; ++i) EXPECT_EQ(f.Process(1), 2);
    for (int i = 0; i < kToggleStableBlocks - 1; ++i) EXPECT_EQ(f.Process(0), 2);
    EXPECT_EQ(f.Process(0), 0);
}

TEST(ToggleFilter, AcceptsExactlyAtStableCount) {
    ToggleFilter f;
    f.Init(0);
    for (int i = 0; i < kToggleStableBlocks - 1; ++i) EXPECT_EQ(f.Process(1), 0);
    EXPECT_EQ(f.Process(1), 1);
}

TEST(ToggleFilter, ReturnToCurrentRestartsCount) {
    ToggleFilter f;
    f.Init(2);
    for (int i = 0; i < kToggleStableBlocks - 1; ++i) f.Process(0);
    EXPECT_EQ(f.Process(2), 2);
    for (int i = 0; i < kToggleStableBlocks - 1; ++i) EXPECT_EQ(f.Process(0), 2);
}

TEST(ShiftFilter, PressAcceptedAfterPressBlocks) {
    ShiftFilter f;
    for (int i = 0; i < kShiftPressBlocks - 1; ++i) EXPECT_FALSE(f.Process(true));
    EXPECT_TRUE(f.Process(true));
}

TEST(ShiftFilter, ReleaseAcceptedAfterReleaseBlocks) {
    ShiftFilter f;
    for (int i = 0; i < kShiftPressBlocks; ++i) f.Process(true);
    for (int i = 0; i < kShiftReleaseBlocks - 1; ++i) EXPECT_TRUE(f.Process(false));
    EXPECT_FALSE(f.Process(false));
}

TEST(ShiftFilter, BounceRestartsCount) {
    ShiftFilter f;
    for (int b = 0; b < 500;) {  // before a press is accepted: 5 pressed reads, 1 released
        for (int i = 0; i < 5; ++i, ++b) EXPECT_FALSE(f.Process(true));
        EXPECT_FALSE(f.Process(false));
        ++b;
    }
    for (int i = 0; i < kShiftPressBlocks; ++i) f.Process(true);
    for (int b = 0; b < 500;) {  // held: 10 released reads, 1 pressed
        for (int i = 0; i < 10; ++i, ++b) EXPECT_TRUE(f.Process(false));
        EXPECT_TRUE(f.Process(true));
        ++b;
    }
}
