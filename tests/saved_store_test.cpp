// daisy/src/saved_store.h: the saved power-up values' payload and its two-slot store.
#include "saved_store.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

using namespace btsave;
using namespace btparam;

namespace {

std::vector<uint8_t> Slot(uint32_t seq, const Saved& s) {
    uint8_t payload[kPayloadBytes];
    Encode(s, payload);
    std::vector<uint8_t> slot(kHeaderBytes + kPayloadBytes);
    Build(slot.data(), seq, payload);
    return slot;
}

}  // namespace

TEST(SavedStore, EncodeDecodeRoundTrip) {
    Saved s;
    for (uint8_t id : {uint8_t(kHallRtMid), uint8_t(kEqLowGain), uint8_t(kDelayBbdModDepth)}) {
        s.has[id] = true;
        s.value[id] = kParams[id].min + 0.37f * (kParams[id].max - kParams[id].min);
    }
    uint8_t payload[kPayloadBytes];
    Encode(s, payload);
    Saved d;
    Decode(payload, kPayloadBytes, d);
    EXPECT_EQ(d.Count(), 3);
    for (uint8_t id = 0; id < kParamCount; ++id) {
        EXPECT_EQ(d.has[id], s.has[id]) << kParams[id].name;
        if (s.has[id]) {
            EXPECT_EQ(d.value[id], s.value[id]) << kParams[id].name;
        }
    }
}

TEST(SavedStore, DecodeDropsSwitchParametersAndNonFiniteValues) {
    Saved s;
    s.has[kDelayType] = true;  // on SW2: not saveable
    s.value[kDelayType] = 2.0f;
    s.has[kHallSize] = true;
    s.value[kHallSize] = NAN;
    s.has[kHallPredelay] = true;
    s.value[kHallPredelay] = 5000.0f;
    uint8_t payload[kPayloadBytes];
    Encode(s, payload);
    Saved d;
    Decode(payload, kPayloadBytes, d);
    EXPECT_FALSE(d.has[kDelayType]);
    EXPECT_FALSE(d.has[kHallSize]);
    EXPECT_TRUE(d.has[kHallPredelay]);
    EXPECT_EQ(d.value[kHallPredelay], 500.0f);  // clamped to the range
    EXPECT_EQ(d.Count(), 1);
}

TEST(SavedStore, FlippedPayloadByteFailsValid) {
    Saved s;
    s.has[kHallPredelay] = true;
    s.value[kHallPredelay] = 100.0f;
    std::vector<uint8_t> slot = Slot(1, s);
    uint32_t seq = 0;
    EXPECT_TRUE(Valid(slot.data(), seq));
    EXPECT_EQ(seq, 1u);
    slot[kHeaderBytes + kBitmapBytes + 4 * kHallPredelay] ^= 0x01;
    EXPECT_FALSE(Valid(slot.data(), seq));
}

TEST(SavedStore, NewestPicksHigherSeqOrTheOnlyValidSlot) {
    const Saved s;
    const std::vector<uint8_t> a = Slot(4, s), b = Slot(5, s);
    uint32_t seq = 0;
    EXPECT_EQ(Newest(a.data(), b.data(), seq), 1);
    EXPECT_EQ(seq, 5u);
    EXPECT_EQ(Newest(b.data(), a.data(), seq), 0);
    EXPECT_EQ(seq, 5u);
    std::vector<uint8_t> bad = b;
    bad[kHeaderBytes + 100] ^= 0xFF;
    EXPECT_EQ(Newest(a.data(), bad.data(), seq), 0);
    EXPECT_EQ(seq, 4u);
    EXPECT_EQ(Newest(bad.data(), a.data(), seq), 1);
    EXPECT_EQ(seq, 4u);
    const std::vector<uint8_t> erased(kHeaderBytes + kPayloadBytes, 0xFF);
    seq = 7;
    EXPECT_EQ(Newest(erased.data(), erased.data(), seq), -1);
    EXPECT_EQ(seq, 7u);
}

// The 528-byte payload of firmware whose link carried an id as one 7-bit byte: read, with a knob assignment of 127
// (that firmware's "none") moved to kNoParam, since 127 is a parameter id now.
TEST(SavedStore, LegacyPayloadDecodesAndKnobNoneMoves) {
    std::vector<uint8_t> payload(kLegacyPayloadBytes, 0);
    const auto put = [&](uint8_t id, float v) {
        payload[id / 8] = uint8_t(payload[id / 8] | (1u << (id % 8)));
        uint32_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        PutLe32(payload.data() + 16 + 4 * id, bits);
    };
    put(kHallPredelay, 100.0f);
    put(kKnob1Secondary, 127.0f);
    put(kKnob2Secondary, float(kDelayMix));
    std::vector<uint8_t> slot(kHeaderBytes + kLegacyPayloadBytes);
    PutLe32(slot.data(), kMagic);
    PutLe32(slot.data() + 4, 3);
    PutLe32(slot.data() + 8, kLegacyPayloadBytes);
    PutLe32(slot.data() + 12, Crc32(payload.data(), kLegacyPayloadBytes));
    std::memcpy(slot.data() + kHeaderBytes, payload.data(), kLegacyPayloadBytes);
    uint32_t seq = 0;
    ASSERT_TRUE(Valid(slot.data(), seq));
    EXPECT_EQ(seq, 3u);
    EXPECT_EQ(PayloadLength(slot.data()), kLegacyPayloadBytes);
    Saved d;
    Decode(slot.data() + kHeaderBytes, PayloadLength(slot.data()), d);
    EXPECT_EQ(d.value[kHallPredelay], 100.0f);
    EXPECT_EQ(d.value[kKnob1Secondary], float(kNoParam));
    EXPECT_EQ(d.value[kKnob2Secondary], float(kDelayMix));
    EXPECT_EQ(d.Count(), 3);
}
