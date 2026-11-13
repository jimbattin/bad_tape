// daisy/src/link.h: the wire contract host tools parse (Identity Reply, PedalCommand) and the host-message reassembly.
#include "link.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

using namespace btlink;

namespace {

// The MIDI bytes of a TxBuffer's USB-MIDI packets (CIN 4: 3 bytes; CIN 5/6/7: 1/2/3 bytes, SysEx end).
std::vector<uint8_t> Unpack(const TxBuffer& t) {
    std::vector<uint8_t> out;
    for (size_t i = 0; i + 4 <= t.size(); i += 4) {
        const uint8_t cin = t.data()[i] & 0x0F;
        const size_t n = cin == 4 ? 3 : cin >= 5 && cin <= 7 ? cin - 4 : 0;
        for (size_t j = 0; j < n; ++j) out.push_back(t.data()[i + 1 + j]);
    }
    return out;
}

std::vector<uint8_t> IdentityPrefix(uint8_t board) {
    return {0xF0, 0x7E, 0x7F, 0x06, 0x02, 0x7D, 0x62, 0x00, board, 0x00, 0x00, 0x00, 0x00, 0x00};
}

}  // namespace

TEST(Identity, ReplyBytes) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendIdentity(1, "abc1234+dirty"));
    std::vector<uint8_t> want = IdentityPrefix(1);
    for (char c : std::string("abc1234+dirty")) want.push_back(uint8_t(c));
    want.push_back(0xF7);
    EXPECT_EQ(Unpack(t), want);
    EXPECT_EQ(t.data()[t.size() - 4] & 0x0F, 5);  // 28 bytes: the last packet carries F7 alone
}

TEST(Identity, EmptyBuildIdIsUnknown) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendIdentity(1, ""));
    std::vector<uint8_t> want = IdentityPrefix(1);
    for (char c : std::string("unknown")) want.push_back(uint8_t(c));
    want.push_back(0xF7);
    EXPECT_EQ(Unpack(t), want);
}

TEST(PedalCommand, OwnFrameOnly) {
    const uint8_t own[] = {0xF0, 0x7D, 0x62, 0x02, 0x06, 0xF7};
    const uint8_t version01[] = {0xF0, 0x7D, 0x62, 0x01, 0x06, 0xF7};
    const uint8_t otherDevice[] = {0xF0, 0x7D, 0x70, 0x02, 0x06, 0xF7};
    const uint8_t otherHeader[] = {0xF0, 0x7D, 0x43, 0x53, 0x08, 0xF7};
    EXPECT_EQ(PedalCommand(own, sizeof own), 0x06);
    EXPECT_EQ(PedalCommand(version01, sizeof version01), -1);
    EXPECT_EQ(PedalCommand(otherDevice, sizeof otherDevice), -1);
    EXPECT_EQ(PedalCommand(otherHeader, sizeof otherHeader), -1);
}

TEST(SysExAssembler, SkipsRealtimeInsideMessage) {
    SysExAssembler<64> a;
    const uint8_t in[] = {0xF0, 0x7D, 0x62, 0xF8, 0x02, 0x06, 0xF7};
    bool done = false;
    for (uint8_t b : in) done = a.Feed(b);
    ASSERT_TRUE(done);
    const std::vector<uint8_t> want = {0xF0, 0x7D, 0x62, 0x02, 0x06, 0xF7};
    EXPECT_EQ(std::vector<uint8_t>(a.data(), a.data() + a.size()), want);
}

TEST(SysExAssembler, StatusByteDropsMessage) {
    SysExAssembler<64> a;
    const uint8_t in[] = {0xF0, 0x7D, 0x62, 0x90, 0x01, 0x06, 0xF7};
    for (uint8_t b : in) EXPECT_FALSE(a.Feed(b));
}

TEST(Log, TruncatedToMaxLog) {
    TxBuffer t;
    const std::string text(200, 'x');
    ASSERT_TRUE(t.AppendLog(text.c_str()));
    const std::vector<uint8_t> bytes = Unpack(t);
    ASSERT_EQ(bytes.size(), 5 + kMaxLog + 1);
    EXPECT_EQ(bytes[4], kLog);
    EXPECT_EQ(bytes.back(), 0xF7);
}

TEST(Params, ValueBytes) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendValue(5, 8192));
    EXPECT_EQ(Unpack(t), (std::vector<uint8_t>{0xF0, 0x7D, 0x62, 0x02, 0x13, 0x00, 0x05, 0x40, 0x00, 0xF7}));
}

TEST(Params, DumpEndBytes) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendDumpEnd(300));
    EXPECT_EQ(Unpack(t), (std::vector<uint8_t>{0xF0, 0x7D, 0x62, 0x02, 0x15, 0x02, 0x2C, 0xF7}));
}

TEST(Saved, DecodeIds) {
    uint8_t ids[2];
    const uint8_t two[] = {0x00, 0x09, 0x00, 0x4A};
    ASSERT_EQ(DecodeIds(two, sizeof two, ids), 2u);
    EXPECT_EQ(ids[0], 9);
    EXPECT_EQ(ids[1], 74);
    const uint8_t odd[] = {0x00, 0x09, 0x00};
    EXPECT_EQ(DecodeIds(odd, sizeof odd, ids), 0u);
    const uint8_t past[] = {0x01, 0x7F};  // id 255
    EXPECT_EQ(DecodeIds(past, sizeof past, ids), 0u);
}

// Every entry fits one message; the text is name|unit|min|max with min and max as %g prints them (usb_log.py parses
// it).
TEST(Params, ParamInfoForEveryEntry) {
    for (const btparam::ParamDef& d : btparam::kParams) {
        SCOPED_TRACE(d.name);
        TxBuffer t;
        ASSERT_TRUE(t.AppendParamInfo(d, 1234));
        const std::vector<uint8_t> b = Unpack(t);
        ASSERT_GE(b.size(), 6u + 10u);
        EXPECT_EQ(b[4], kParam);
        EXPECT_EQ(b[5] << 7 | b[6], d.id);
        EXPECT_EQ(b[13], 1234 >> 7);
        EXPECT_EQ(b[14], 1234 & 0x7F);
        char want[128];
        snprintf(want, sizeof want, "%s|%s|%g|%g", d.name, d.unit, double(d.min), double(d.max));
        EXPECT_EQ(std::string(b.begin() + 15, b.end() - 1), want);
    }
}

TEST(FormatNumber, MatchesPrintfG) {
    for (float v : {0.0f, 1.0f, -100.0f, 0.2f, 0.707f, 141.42136f, 20000.0f, -0.5f, 1234.5f, 99.99999f, 0.001f}) {
        char got[16], want[32];
        FormatNumber(got, sizeof got, v);
        snprintf(want, sizeof want, "%g", double(v));
        EXPECT_STREQ(got, want) << v;
    }
}

TEST(Saved, SavedBytes) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendSaved(9, true, 0x1234));
    EXPECT_EQ(Unpack(t), (std::vector<uint8_t>{0xF0, 0x7D, 0x62, 0x02, 0x16, 0x00, 0x09, 0x01, 0x24, 0x34, 0xF7}));
}

TEST(Saved, StoreBytes) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendStore(btlink::kStoreFailed));
    EXPECT_EQ(Unpack(t), (std::vector<uint8_t>{0xF0, 0x7D, 0x62, 0x02, 0x17, 0x01, 0xF7}));
}

TEST(Saved, SavedEndBytes) {
    TxBuffer t;
    ASSERT_TRUE(t.AppendSavedEnd(96));
    EXPECT_EQ(Unpack(t), (std::vector<uint8_t>{0xF0, 0x7D, 0x62, 0x02, 0x18, 0x00, 0x60, 0xF7}));
}

TEST(Watch, DueOnlyPastTheDeadbandOrAtTheEnds) {
    const btparam::ParamDef& time = btparam::kParams[btparam::kDelayTime];
    EXPECT_FALSE(WatchDue(time, 100, 100));
    EXPECT_FALSE(WatchDue(time, 100, 107));  // knob jitter
    EXPECT_TRUE(WatchDue(time, 100, 108));
    EXPECT_TRUE(WatchDue(time, 108, 100));
    EXPECT_TRUE(WatchDue(time, kNoCode, 100));
    EXPECT_TRUE(WatchDue(time, 5, 0));  // the ends are always exact
    EXPECT_TRUE(WatchDue(time, 16380, btparam::kCodeMax));
    EXPECT_TRUE(WatchDue(btparam::kParams[btparam::kDelayMode], 8191, 8192));  // an enum: any change
}
