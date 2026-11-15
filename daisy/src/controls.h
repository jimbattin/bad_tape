// Toggle meanings, toggle hysteresis and the FS2 shift debounce. Host-portable (no libDaisy, no allocation), covered
// by tests/controls_test.cpp. Knob laws to engineering units live in params.h (FromNorm); parameter smoothing in
// dsp/blocks.h.
// Toggle positions (board.h): 0 down, 1 middle, 2 up; the enums' values are those positions.
#pragma once

#include <cstdint>

namespace btctl {

// SW1: Up forward, Middle off, Down reverse.
enum class DelayMode : uint8_t { kReverse = 0, kOff = 1, kForward = 2 };
// SW2: the delay's character, Down bucket brigade, Middle tape, Up digital. Knob 6 (delay_color) colors the one
// selected, from none fully counter-clockwise to full fully clockwise; the BBD keeps its character at 0.
enum class DelayType : uint8_t { kBbd = 0, kTape = 1, kDigital = 2 };
// The routing parameter (MIDI only; no switch), what feeds the reverb: the delay's 100 % wet, the delay mix
// (default), or the dry input (through the EQ when it is placed pre).
enum class Routing : uint8_t { kWet = 0, kMix = 1, kDry = 2 };
// delay_sos_mode (no switch): what the sound-on-sound head is (dsp/delay.h).
enum class SosMode : uint8_t { kOff = 0, k301 = 1, k501 = 2, kOverdub = 3 };
// SW3: Down Plate, Middle Hall, Up Canyon.
enum class Algo : uint8_t { kPlate = 0, kHall = 1, kCanyon = 2 };

// Positions above 2 map to the middle value.
inline DelayMode ToDelayMode(uint8_t pos) { return pos <= 2 ? DelayMode(pos) : DelayMode::kOff; }
inline DelayType ToDelayType(uint8_t pos) { return pos <= 2 ? DelayType(pos) : DelayType::kTape; }
inline Routing ToRouting(uint8_t pos) { return pos <= 2 ? Routing(pos) : Routing::kMix; }
inline SosMode ToSosMode(uint8_t v) { return v <= 3 ? SosMode(v) : SosMode::kOff; }
inline Algo ToAlgo(uint8_t pos) { return pos <= 2 ? Algo(pos) : Algo::kHall; }

// Log names.
inline const char* Name(DelayMode m) {
    return m == DelayMode::kReverse ? "rev" : m == DelayMode::kForward ? "fwd" : "off";
}
inline const char* Name(DelayType t) {
    return t == DelayType::kBbd ? "bbd" : t == DelayType::kDigital ? "digital" : "tape";
}
inline const char* Name(Routing r) { return r == Routing::kWet ? "wet" : r == Routing::kDry ? "dry" : "mix"; }
inline const char* Name(SosMode m) {
    return m == SosMode::k301 ? "301" : m == SosMode::k501 ? "501" : m == SosMode::kOverdub ? "overdub" : "off";
}
inline const char* Name(Algo a) { return a == Algo::kPlate ? "plate" : a == Algo::kCanyon ? "canyon" : "hall"; }

// A toggle accepts a new position only after it has read that position for kToggleStableBlocks consecutive blocks,
// so a lever moving up <-> down never reports the middle it passes through.
constexpr int kToggleStableBlocks = 90;  // 30 ms at the 3 kHz block rate
class ToggleFilter {
public:
    void Init(uint8_t pos) {
        pos_ = candidate_ = pos;
        count_ = 0;
    }
    // Once per block with the raw position; returns the accepted position.
    uint8_t Process(uint8_t raw) {
        if (raw == pos_) {
            count_ = 0;
            return pos_;
        }
        if (raw != candidate_) {
            candidate_ = raw;
            count_ = 1;
        } else {
            ++count_;
        }
        if (count_ >= kToggleStableBlocks) {
            pos_ = raw;
            count_ = 0;
        }
        return pos_;
    }

private:
    uint8_t pos_ = 1, candidate_ = 1;
    int count_ = 0;
};

// FS2 as the knobs' shift (param_store.h). The raw contact, read once per block, counts as pressed after
// kShiftPressBlocks consecutive pressed reads and as released after kShiftReleaseBlocks consecutive released reads.
// Contact bounce on either edge never flips it. A press registers before a hand can turn a knob far. A release
// waits until the knob that was being turned has settled.
constexpr int kShiftPressBlocks = 15;    // 5 ms at the 3 kHz block rate
constexpr int kShiftReleaseBlocks = 90;  // 30 ms
class ShiftFilter {
public:
    // Once per block with the raw contact; returns the accepted state.
    bool Process(bool raw) {
        if (raw == held_) {
            count_ = 0;
            return held_;
        }
        if (++count_ >= (held_ ? kShiftReleaseBlocks : kShiftPressBlocks)) {
            held_ = raw;
            count_ = 0;
        }
        return held_;
    }

private:
    bool held_ = false;
    int count_ = 0;
};

}  // namespace btctl
