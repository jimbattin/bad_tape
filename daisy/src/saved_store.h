// The saved power-up values: the editor's SAVE (link.h) persists a parameter's current value to QSPI flash, and the
// pedal applies it at power-up instead of the params.h default (param_store.h Load). Only parameters not on a switch
// or footswitch are saveable (those set their own value); no saved value means "use the factory default". A parameter
// on a primary knob slot is saveable, but the knob replaces its saved value at power-up.
//
// The store is a pair of 4 KB slots at QSPI 0x903FE000 / 0x903FF000 past the BOOT_SRAM image the Daisy bootloader
// keeps at 0x90040000 (at most 512 KB), each holding a 16-byte header (magic, sequence number, payload length, CRC-32
// of the payload; uint32 little-endian) and the payload (Encode). Every write is the 1056-byte payload for 256 ids;
// the 528-byte payload for 128 ids written by firmware whose link carried an id as one 7-bit byte is still read
// (Decode). QSPIHandle::Erase erases 4 KB sectors. Every write goes to the slot not holding the newest payload, so a
// failed write leaves the previous payload readable; at startup the valid slot with the higher sequence number wins.
// Host-portable (no libDaisy), so tests/saved_store_test.cpp covers it.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "params.h"

namespace btsave {

constexpr uint32_t kQspiBase = 0x90000000;   // memory-mapped QSPI
constexpr uint32_t kHeaderBytes = 16;        // magic, seq, length, crc32

constexpr uint32_t kMagic = 0x56535442;                    // "BTSV"
constexpr uint32_t kSlotOffset[2] = {0x3FE000, 0x3FF000};  // QSPI chip offsets of the two slots
constexpr uint32_t kSlotBytes = 0x1000;                    // a slot, erased as a whole
constexpr uint32_t kIds = 256;                               // params.h: ids are a uint8_t
constexpr uint32_t kBitmapBytes = kIds / 8;                  // 32
constexpr uint32_t kPayloadBytes = kBitmapBytes + 4 * kIds;  // 1056: the saved bitmap, then a float32 LE per id
// The first payload (ids one 7-bit byte): still read at power-up; the next SAVE/CLEAR rewrites it in this format.
constexpr uint32_t kLegacyIds = 128;
constexpr uint32_t kLegacyPayloadBytes = kLegacyIds / 8 + 4 * kLegacyIds;  // 528
constexpr float kLegacyParamCount = 126.0f;  // the last legacy table's size: a knob assignment >= this meant none
static_assert(kHeaderBytes + kPayloadBytes <= kSlotBytes, "a slot holds header and payload");
static_assert(kSlotOffset[1] >= kSlotOffset[0] + kSlotBytes, "the slots do not overlap");
static_assert(kSlotOffset[0] >= 0x40000 + 512 * 1024, "the slots lie past the largest BOOT_SRAM image");
static_assert(kSlotOffset[1] + kSlotBytes <= 0x400000, "the slots end inside MPU region 4 (firmware.h)");
static_assert(btparam::kParamCount <= kIds, "the payload holds every id");

// CRC-32 (reflected, polynomial 0xEDB88320, init and final xor 0xFFFFFFFF).
inline uint32_t Crc32(const uint8_t* p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        c ^= p[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

inline uint32_t GetLe32(const uint8_t* p) {
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void PutLe32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}

// A slot with kMagic, length kPayloadBytes or kLegacyPayloadBytes and the payload's CRC; seq is its sequence number.
inline bool Valid(const uint8_t* slot, uint32_t& seq) {
    const uint32_t len = GetLe32(slot + 8);
    if (GetLe32(slot) != kMagic || (len != kPayloadBytes && len != kLegacyPayloadBytes)) return false;
    if (GetLe32(slot + 12) != Crc32(slot + kHeaderBytes, len)) return false;
    seq = GetLe32(slot + 4);
    return true;
}
inline uint32_t PayloadLength(const uint8_t* slot) { return GetLe32(slot + 8); }  // of a Valid slot

// The slot (0 or 1) holding the newest valid payload, with its sequence number in seq: the higher sequence number
// when both are valid; -1 for none (seq untouched).
inline int Newest(const uint8_t* a, const uint8_t* b, uint32_t& seq) {
    uint32_t sa = 0, sb = 0;
    const bool va = Valid(a, sa), vb = Valid(b, sb);
    const int s = va && vb ? (sb > sa ? 1 : 0) : va ? 0 : vb ? 1 : -1;
    if (s >= 0) seq = s == 1 ? sb : sa;
    return s;
}

// Header and kPayloadBytes payload bytes into out (kHeaderBytes + kPayloadBytes bytes).
inline void Build(uint8_t* out, uint32_t seq, const uint8_t* payload) {
    PutLe32(out, kMagic);
    PutLe32(out + 4, seq);
    PutLe32(out + 8, kPayloadBytes);
    PutLe32(out + 12, Crc32(payload, kPayloadBytes));
    std::memcpy(out + kHeaderBytes, payload, kPayloadBytes);
}

// The saved values: has[id] says value[id] is the id's power-up value.
struct Saved {
    bool has[btparam::kParamCount] = {};
    float value[btparam::kParamCount] = {};

    int Count() const {
        int n = 0;
        for (bool h : has) n += h;
        return n;
    }
};

// s into out (kPayloadBytes): bit id % 8 of byte id / 8 is set iff id is saved; the float32 bits at kBitmapBytes +
// 4 * id. Bytes for ids >= kParamCount are zero.
inline void Encode(const Saved& s, uint8_t* out) {
    std::memset(out, 0, kPayloadBytes);
    for (uint8_t id = 0; id < btparam::kParamCount; ++id) {
        if (!s.has[id]) continue;
        out[id / 8] = uint8_t(out[id / 8] | (1u << (id % 8)));
        uint32_t bits;
        std::memcpy(&bits, &s.value[id], sizeof bits);
        PutLe32(out + kBitmapBytes + 4 * id, bits);
    }
}

// The payload p of length bytes (kPayloadBytes, or kLegacyPayloadBytes: the same layout for 128 ids, its floats at
// 16 + 4 * id) into s. An id is kept only if its bit is set, it is saveable and its value is finite; the value is
// Clamp()ed. Everything else is dropped. A legacy knob assignment >= kLegacyParamCount meant none (no legacy table had
// such an id) and becomes kNoParam, since ids from 126 on are parameters now.
inline void Decode(const uint8_t* p, uint32_t length, Saved& s) {
    s = Saved{};
    const bool legacy = length == kLegacyPayloadBytes;
    const uint32_t ids = legacy ? kLegacyIds : kIds;
    for (uint32_t id = 0; id < ids && id < btparam::kParamCount; ++id) {
        if (!(p[id / 8] & (1u << (id % 8))) || !btparam::IsSaveable(uint8_t(id))) continue;
        const uint32_t bits = GetLe32(p + ids / 8 + 4 * id);
        float v;
        std::memcpy(&v, &bits, sizeof v);
        if (!std::isfinite(v)) continue;
        if (legacy && btparam::kParams[id].group == btparam::Group::kKnobs && v >= kLegacyParamCount)
            v = btparam::kNoParam;
        s.has[id] = true;
        s.value[id] = btparam::Clamp(btparam::kParams[id], v);
    }
}

}  // namespace btsave
