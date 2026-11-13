// daisy/src/midi.h: CC extraction from a raw MIDI stream (running status, other messages, realtime, SysEx) and NRPN
// decoding.
#include "midi.h"

#include <gtest/gtest.h>

#include <initializer_list>
#include <tuple>
#include <vector>

using namespace btmidi;

namespace {

using CcT = std::tuple<int, int, int>;  // channel, number, value

std::vector<CcT> Parse(std::initializer_list<uint8_t> bytes) {
    CcParser p;
    std::vector<CcT> out;
    Cc c;
    for (uint8_t b : bytes)
        if (p.Feed(b, c)) out.emplace_back(c.channel, c.number, c.value);
    return out;
}

// Feeds CCs (channel, number, value) to d; returns the (number, code) pairs it emits.
std::vector<std::pair<int, int>> Nrpn(NrpnDecoder& d, std::initializer_list<CcT> ccs) {
    std::vector<std::pair<int, int>> out;
    for (const CcT& t : ccs) {
        uint16_t n, code;
        if (d.Feed(Cc{uint8_t(std::get<0>(t)), uint8_t(std::get<1>(t)), uint8_t(std::get<2>(t))}, n, code))
            out.emplace_back(n, code);
    }
    return out;
}

}  // namespace

TEST(CcParser, RunningStatus) {
    EXPECT_EQ(Parse({0xB2, 0x07, 0x64, 0x07, 0x65}), (std::vector<CcT>{{2, 7, 100}, {2, 7, 101}}));
}

TEST(CcParser, NoteDataIsNotCc) {
    EXPECT_EQ(Parse({0x90, 0x3C, 0x64, 0xB0, 0x0E, 0x40}), (std::vector<CcT>{{0, 14, 64}}));
}

TEST(CcParser, ProgramChangeTakesOneByte) {
    EXPECT_EQ(Parse({0xC0, 0x05, 0xB0, 0x0E, 0x40}), (std::vector<CcT>{{0, 14, 64}}));
    EXPECT_TRUE(Parse({0xC0, 0x05, 0x0E, 0x40}).empty());  // running status of a program change
}

TEST(CcParser, RealtimeInsideMessage) { EXPECT_EQ(Parse({0xB0, 0xF8, 0x0E, 0x40}), (std::vector<CcT>{{0, 14, 64}})); }

TEST(CcParser, SysExEndsRunningStatus) {
    EXPECT_EQ(Parse({0xB0, 0x0E, 0x40, 0xF0, 0x7D, 0x0E, 0x40, 0xF7, 0x0E, 0x40}), (std::vector<CcT>{{0, 14, 64}}));
}

TEST(CcParser, SystemCommonSwallowsItsData) {
    EXPECT_TRUE(Parse({0xB0, 0xF2, 0x0E, 0x40, 0x0E, 0x40}).empty());   // song position: 2 bytes, then no status
    EXPECT_TRUE(Parse({0xB0, 0xF3, 0x0E, 0x0E, 0x40}).empty());
}

TEST(Nrpn, DataMsbThenLsb) {
    NrpnDecoder d;
    EXPECT_EQ(Nrpn(d, {{0, 99, 0}, {0, 98, 5}, {0, 6, 64}}), (std::vector<std::pair<int, int>>{{5, 8192}}));
    EXPECT_EQ(Nrpn(d, {{0, 38, 1}}), (std::vector<std::pair<int, int>>{{5, 8193}}));
}

TEST(Nrpn, MsbSelectsIdsPast127) {
    NrpnDecoder d;
    EXPECT_EQ(Nrpn(d, {{0, 99, 1}, {0, 98, 0x48}, {0, 6, 64}}), (std::vector<std::pair<int, int>>{{200, 8192}}));
}

TEST(Nrpn, NeedsBothNumberBytes) {
    NrpnDecoder d;
    EXPECT_TRUE(Nrpn(d, {{0, 98, 5}, {0, 6, 64}}).empty());
}

TEST(Nrpn, RpnSelectIgnoresDataEntry) {
    NrpnDecoder d;
    EXPECT_TRUE(Nrpn(d, {{0, 99, 0}, {0, 98, 5}, {0, 101, 0}, {0, 100, 0}, {0, 6, 64}}).empty());
}

TEST(Nrpn, NullDeselects) {
    NrpnDecoder d;
    EXPECT_TRUE(Nrpn(d, {{0, 99, 0}, {0, 98, 5}, {0, 99, 127}, {0, 98, 127}, {0, 6, 64}, {0, 38, 1}}).empty());
}

TEST(Nrpn, ChannelsAreIndependent) {
    NrpnDecoder d;
    EXPECT_TRUE(Nrpn(d, {{0, 99, 0}, {0, 98, 5}, {1, 6, 64}}).empty());
    EXPECT_EQ(Nrpn(d, {{1, 99, 0}, {1, 98, 7}, {0, 6, 1}, {1, 6, 2}}),
              (std::vector<std::pair<int, int>>{{5, 128}, {7, 256}}));
}
