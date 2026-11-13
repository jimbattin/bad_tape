// The pedal's link over the Seed's USB MIDI port: SysEx `F0 7D 62 02 <cmd> <payload> F7` (manufacturer 0x7D =
// non-commercial, device 0x62, version 0x02), packed as USB-MIDI event packets. Ids and counts travel as two 7-bit
// bytes like codes (hi = v >> 7, lo = v & 0x7F); version-01 frames (the previous format, one byte per id) are other
// devices' frames and ignored. Host-portable (no libDaisy), so the host tests (tests/link_test.cpp) cover it. Read the
// log with daisy/tools/usb_log.py. Host tools identify (Identity Request/Reply), restart (DFU) and flash the pedal
// with these messages.
//
// Host -> pedal:
//   01 HELLO                          (no payload) the pedal answers one LOG line, the boot line
//                                     `bad_tape <board> sysclk N MHz <build id>` (the bench: `bad_tape_bench ...`;
//                                     the latency image: `bad_tape_latency <board> seed <version> <build id>`).
//   06 DFU                            (no payload) the pedal logs `dfu reset` and restarts into the Daisy bootloader's
//                                     DFU mode (no timeout), ready for daisy/tools/flash_when_ready.sh.
//   02 GET <id hi> <id lo>            the pedal answers 13 VALUE for parameter id (params.h).
//   03 SET <id hi> <id lo> <hi> <lo>  sets parameter id from code hi << 7 | lo (params.h FromCode) as a MIDI write
//                                     (held against its knob slot, param_store.h) and answers 13 VALUE with the stored
//                                     value's code (an enum's canonical code).
//   04 DUMP                           (no payload) the pedal answers one 14 PARAM per id, then 15 DUMP END. A DUMP
//                                     during a dump restarts it.
//   05 WATCH <on>                     on = 1: the pedal streams 13 VALUE for every parameter whose code changed for any
//                                     reason (knob, switch, FS1, link, MIDI), checked every 20 ms (WatchDue: a few
//                                     codes of knob jitter are absorbed). Turning it on records the current codes as
//                                     sent. Off (0) is the boot default. A wrong length or on > 1 logs
//                                     `link: bad watch message`.
//   07 SAVE <id hi> <id lo> ...       (1..the saveable count ids; a longer frame is dropped without a reply) saves
//                                     each id's current value as its power-up value, in one flash write
//                                     (saved_store.h); answered by 16 SAVED per id (only on success), then 17 STORE.
//   08 CLEAR <id hi> <id lo> ...      forgets the saved values (power-up uses the default). Same replies as SAVE; if
//                                     nothing changes there is no flash write and the status is 0.
//   09 SAVED?                         (no payload) the pedal answers 16 SAVED for every saveable id in id order, then
//                                     18 SAVED END.
//   A SAVE/CLEAR with an empty or odd payload, an id >= the parameter count or a parameter on a switch or FS1 (not
//   saveable) logs `link: bad save message CC` and answers 17 STORE 2.
//   An id >= the parameter count or a wrong length logs `link: bad param message CC` and gets no reply.
//   Any other command with this header logs `link: unknown command CC`. SysEx for other devices (e.g. a flasher
//   broadcasting DFU frames for several devices) is ignored.
//   Identity Request F0 7E <id> 06 01 F7 (Universal Non-Real-Time, any id 00-7F): the pedal answers the Identity
//                                     Reply below.
// Pedal -> host:
//   12 LOG <printable ASCII <= 120>   one log line (boot, the controls and status lines once a second).
//   13 VALUE <id hi> <id lo> <hi> <lo>
//                                     parameter id's current code (14 bits: hi << 7 | lo).
//   14 PARAM <id hi> <id lo> <taper> <control> <cc> <group> <def hi> <def lo> <cur hi> <cur lo> <name|unit|min|max>
//                                     one table entry (params.h enums; codes 14 bits), then the ASCII text with min
//                                     and max formatted %g.
//   15 DUMP END <count hi> <count lo> the dump is complete; count = number of parameters.
//   16 SAVED <id hi> <id lo> <has> <hi> <lo>
//                                     has 1: the code of id's saved value; has 0: the code of its default.
//   17 STORE <status>                 0 ok, 1 the flash write failed (the saved values are unchanged), 2 bad request.
//   18 SAVED END <count hi> <count lo>
//                                     the SAVED? answer is complete; count = number of saveable parameters.
//   Identity Reply F0 7E 7F 06 02 7D 62 00 <board> 00 00 00 00 00 <build id> F7: manufacturer 7D, family 62 00
//                                     (device byte 62, then 00), member <board> 00 (1 HotHouse, board.h), version
//                                     00 00 00 00, then the build id: 1-32 printable ASCII bytes, the short git
//                                     commit with `+dirty` when tracked files differ (e.g. `711fa7c+dirty`), or
//                                     `unknown` outside git (daisy/Makefile writes it to $(BUILD_DIR)/build_id.c).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "params.h"

namespace btlink {

constexpr uint8_t kManufacturer = 0x7D, kDevice = 0x62, kVersion = 0x02;
constexpr uint8_t kHello = 0x01, kGet = 0x02, kSet = 0x03, kDump = 0x04, kWatch = 0x05, kDfu = 0x06, kSave = 0x07,
                  kClear = 0x08, kSavedQuery = 0x09;  // host -> pedal
constexpr uint8_t kLog = 0x12, kValue = 0x13, kParam = 0x14, kDumpEnd = 0x15, kSaved = 0x16, kStore = 0x17,
                  kSavedEnd = 0x18;  // pedal -> host
constexpr uint8_t kStoreOk = 0, kStoreFailed = 1, kStoreBadRequest = 2;
constexpr uint16_t kNoCode = 0xFFFF;  // WatchDue: nothing sent yet
constexpr int kWatchDeadband = 8;     // codes (about 0.05 % of travel): absorbs knob ADC jitter
constexpr size_t kMaxLog = 120, kMaxBuildId = 32;

// An id or a count as two 7-bit bytes: hi = v >> 7, lo = v & 0x7F (v < 16384).
inline void PutU14(uint8_t* p, unsigned v) {
    p[0] = uint8_t((v >> 7) & 0x7F);
    p[1] = uint8_t(v & 0x7F);
}
inline unsigned GetU14(const uint8_t* p) { return unsigned(p[0]) << 7 | p[1]; }
// A SAVE/CLEAR payload of bytes bytes (two per id) into ids; the number of ids, or 0 for an odd length or an id >=
// the parameter count. ids holds bytes / 2 entries.
inline size_t DecodeIds(const uint8_t* p, size_t bytes, uint8_t* ids) {
    if (bytes % 2 != 0) return 0;
    for (size_t i = 0; i < bytes / 2; ++i) {
        const unsigned id = GetU14(p + 2 * i);
        if (id >= btparam::kParamCount) return 0;
        ids[i] = uint8_t(id);
    }
    return bytes / 2;
}

// v as printf("%g") prints it for 1e-4 <= |v| < 1e6 (every table min/max) or 0, without float printf: the firmware
// links newlib-nano, whose float formatting is not linked and would allocate. Always NUL-terminates (size >= 1);
// returns the length written.
inline size_t FormatNumber(char* buf, size_t size, float v) {
    const bool neg = v < 0.0f;
    double a = neg ? -double(v) : double(v);
    if (!(a < 4.0e9)) a = 4.0e9;  // also NaN
    int intDigits = 1;
    for (double p = 10.0; a >= p && intDigits < 10; p *= 10.0) ++intDigits;
    int dec = intDigits < 6 ? 6 - intDigits : 0;  // 6 significant digits
    double pow10 = 1.0;
    for (int i = 0; i < dec; ++i) pow10 *= 10.0;
    uint32_t scaled = uint32_t(a * pow10 + 0.5);
    for (; dec > 0 && scaled % 10 == 0; --dec) scaled /= 10;  // trailing zeros
    char rev[24];  // digits, least significant first
    size_t n = 0;
    for (int i = 0; scaled != 0 || i <= dec; ++i) {  // at least one integer digit
        if (i == dec && dec > 0) rev[n++] = '.';
        rev[n++] = char('0' + scaled % 10);
        scaled /= 10;
    }
    if (neg && !(n == 1 && rev[0] == '0')) rev[n++] = '-';
    size_t len = 0;
    while (n > 0 && len + 1 < size) buf[len++] = rev[--n];
    buf[len] = '\0';
    return len;
}

class TxBuffer {
public:
    static constexpr size_t kCapacity = 1024;

    // Printable ASCII; other bytes become '?'. Truncated to kMaxLog chars.
    bool AppendLog(const char* text) { return AppendText(kLog, text, kMaxLog); }
    // Identity Reply: board, then buildId (printable ASCII; other bytes become '?'; truncated to kMaxBuildId chars;
    // empty becomes "unknown").
    bool AppendIdentity(uint8_t board, const char* buildId) {
        uint8_t msg[14 + kMaxBuildId + 1] = {0xF0, 0x7E, 0x7F, 0x06, 0x02, kManufacturer, kDevice, 0x00, board,
                                            0x00, 0x00, 0x00, 0x00, 0x00};
        size_t len = 14;
        if (!buildId || !buildId[0]) buildId = "unknown";
        for (size_t i = 0; i < kMaxBuildId && buildId[i]; i++) msg[len++] = Printable(uint8_t(buildId[i]));
        msg[len++] = 0xF7;
        return AppendRaw(msg, len);
    }

    // 13 VALUE <id hi> <id lo> <hi> <lo>.
    bool AppendValue(uint8_t id, uint16_t code) {
        uint8_t p[4];
        PutU14(p, id);
        PutU14(p + 2, code);
        return AppendSysEx(kValue, p, sizeof p);
    }
    // 14 PARAM for table entry d with current code cur.
    bool AppendParamInfo(const btparam::ParamDef& d, uint16_t cur) {
        const uint16_t def = btparam::ToCode(d, d.def);
        uint8_t p[kMaxLog];
        PutU14(p, d.id);
        p[2] = uint8_t(d.taper);
        p[3] = uint8_t(d.control);
        p[4] = d.cc;
        p[5] = uint8_t(d.group);
        PutU14(p + 6, def);
        PutU14(p + 8, cur);
        constexpr size_t kHead = 10;
        char lo[16], hi[16], text[kMaxLog];
        FormatNumber(lo, sizeof lo, d.min);
        FormatNumber(hi, sizeof hi, d.max);
        if (snprintf(text, sizeof text, "%s|%s|%s|%s", d.name, d.unit, lo, hi) < 0) return false;
        size_t len = kHead;
        for (size_t i = 0; text[i] && len < kMaxLog; ++i) p[len++] = Printable(uint8_t(text[i]));
        return AppendSysEx(kParam, p, len);
    }
    // 15 DUMP END <count hi> <count lo>.
    bool AppendDumpEnd(uint16_t count) { return AppendCount(kDumpEnd, count); }
    // 16 SAVED <id hi> <id lo> <has> <hi> <lo>.
    bool AppendSaved(uint8_t id, bool has, uint16_t code) {
        uint8_t p[5];
        PutU14(p, id);
        p[2] = uint8_t(has ? 1 : 0);
        PutU14(p + 3, code);
        return AppendSysEx(kSaved, p, sizeof p);
    }
    // 17 STORE <status>.
    bool AppendStore(uint8_t status) { return AppendSysEx(kStore, &status, 1); }
    // 18 SAVED END <count hi> <count lo>.
    bool AppendSavedEnd(uint16_t count) { return AppendCount(kSavedEnd, count); }

    const uint8_t* data() const { return buf_; }
    size_t size() const { return size_; }
    void Clear() { size_ = 0; }

private:
    static uint8_t Printable(uint8_t c) { return c >= 0x20 && c <= 0x7E ? c : uint8_t('?'); }
    bool AppendCount(uint8_t cmd, uint16_t count) {
        uint8_t p[2];
        PutU14(p, count);
        return AppendSysEx(cmd, p, sizeof p);
    }

    // All or nothing: false (and nothing appended) if the message does not fit.
    bool AppendSysEx(uint8_t cmd, const uint8_t* payload, size_t n) {
        uint8_t msg[6 + kMaxLog];
        if (n > kMaxLog) return false;
        size_t len = 0;
        msg[len++] = 0xF0;
        msg[len++] = kManufacturer;
        msg[len++] = kDevice;
        msg[len++] = kVersion;
        msg[len++] = cmd;
        for (size_t i = 0; i < n; i++) msg[len++] = payload[i];
        msg[len++] = 0xF7;
        return AppendRaw(msg, len);
    }
    // A complete F0 ... F7 message (n >= 2), packetized as is. All or nothing.
    bool AppendRaw(const uint8_t* msg, size_t n) {
        if (n < 2 || size_ + (n + 2) / 3 * 4 > kCapacity) return false;
        size_t i = 0;
        for (; n - i > 3; i += 3) Packet(0x04, msg + i, 3);
        Packet(uint8_t(0x04 + (n - i)), msg + i, n - i);  // 1/2/3 -> CIN 5/6/7
        return true;
    }
    void Packet(uint8_t cin, const uint8_t* b, size_t n) {
        uint8_t* q = buf_ + size_;
        q[0] = cin;
        for (size_t i = 0; i < 3; i++) q[1 + i] = i < n ? b[i] : 0;
        size_ += 4;
    }
    bool AppendText(uint8_t cmd, const char* text, size_t max) {
        uint8_t p[kMaxLog];
        size_t n = 0;
        for (; n < max && text[n]; n++) p[n] = Printable(uint8_t(text[n]));
        return AppendSysEx(cmd, p, n);
    }

    uint8_t buf_[kCapacity];
    size_t size_ = 0;
};

// Reassembles SysEx messages from a MIDI byte stream. Realtime bytes (>= F8) are ignored anywhere; F0 starts a new
// message (dropping an unfinished one); any other status byte aborts the message; a message longer than N bytes is
// dropped up to the next F0.
template <size_t N> class SysExAssembler {
public:
    // True when data()/size() hold a complete F0 ... F7 message; valid until the next Feed.
    bool Feed(uint8_t b) {
        if (b >= 0xF8) return false;
        if (b == 0xF0) {
            buf_[0] = b;
            size_ = 1;
            in_ = true;
            return false;
        }
        if (!in_) return false;
        if (b >= 0x80 && b != 0xF7) {
            in_ = false;
            return false;
        }
        if (size_ == N) {
            in_ = false;
            return false;
        }
        buf_[size_++] = b;
        if (b != 0xF7) return false;
        in_ = false;
        return true;
    }
    const uint8_t* data() const { return buf_; }
    size_t size() const { return size_; }

private:
    uint8_t buf_[N];
    size_t size_ = 0;
    bool in_ = false;
};

// A complete message from SysExAssembler is the Universal Non-Real-Time Identity Request F0 7E <id> 06 01 F7 (any id).
inline bool IsIdentityRequest(const uint8_t* m, size_t n) {
    return n == 6 && m[0] == 0xF0 && m[1] == 0x7E && m[2] <= 0x7F && m[3] == 0x06 && m[4] == 0x01 && m[5] == 0xF7;
}

// The command byte of a complete message with this pedal's header (F0 7D 62 02 <cmd> ... F7), else -1 (including
// other devices' frames, e.g. another device's DFU frame, and version-01 frames).
inline int PedalCommand(const uint8_t* m, size_t n) {
    return n >= 6 && m[1] == kManufacturer && m[2] == kDevice && m[3] == kVersion ? m[4] : -1;
}

// A parameter whose code went from sent to code is worth a VALUE in the WATCH stream: any change of an enum, the
// first message (sent == kNoCode) and the ends of the range; else a change of at least kWatchDeadband codes.
inline bool WatchDue(const btparam::ParamDef& d, uint16_t sent, uint16_t code) {
    if (sent == code) return false;
    if (d.taper == btparam::Taper::kEnum || sent == kNoCode || code == 0 || code == btparam::kCodeMax) return true;
    return std::abs(int(code) - int(sent)) >= kWatchDeadband;
}

}  // namespace btlink
