// host_session.h.
#include "host_session.h"

#include <cstring>

#include "board.h"
#include "firmware.h"
#include "link.h"
#include "midi.h"
#include "params.h"
#include "saved_store.h"
#include "spsc_ring.h"
#include "usb_link.h"

using namespace daisy;
using btparam::kParamCount;
using btparam::kParams;

namespace bthost {
namespace {

btparam::Store* gStore = nullptr;
QSPIHandle* gQspi = nullptr;
void (*gReadKnobs)(float k[6]) = nullptr;
void (*gOnHello)() = nullptr;

// Incoming: host messages (the longest, a SAVE of every saveable id, is 6 + 2 x kSaveableCount bytes), MIDI parsing.
btlink::SysExAssembler<6 + 2 * btparam::kSaveableCount> gHostIn;
btmidi::CcParser gCc;
btmidi::NrpnDecoder gNrpn;

// Ids waiting for a VALUE reply (GET/SET) until there is send room.
SpscRing<uint8_t, 64> gReplies;  // a full ring drops the reply
bool gSendIdentity = false;      // an Identity Reply waits for send room
int gDumpNext = -1;              // next id a DUMP sends, kParamCount = DUMP END next, -1 = no dump

// The saved power-up values (saved_store.h), the WATCH stream and the replies the editor waits for.
btsave::Saved gSaved;
int gSavedSlot = -1;  // the slot of the newest saved values, -1 = none
uint32_t gSavedSeq = 0;
uint8_t gSlotBuf[btsave::kHeaderBytes + btsave::kPayloadBytes];
bool gWatch = false;              // WATCH on: changed values are streamed
uint16_t gSentCode[kParamCount];  // the code of each parameter last sent to the host (btlink::kNoCode: none)
bool gSendSaved[kParamCount];     // a SAVED reply waits for send room
int gStoreResult = -1;            // a STORE reply with this status waits for send room, -1 = none
bool gSendSavedEnd = false;       // a SAVED END reply waits for send room
uint32_t gLastWatch = 0;

// A MIDI or SysEx write: held against its knob (param_store.h) and logged with the stored value.
void applyMidi(uint8_t id, float v) {
    float k[6];
    gReadKnobs(k);
    const float stored = gStore->SetFromMidi(id, v, k);
    char num[16];
    btlink::FormatNumber(num, sizeof num, stored);
    btusb::Log("midi %s = %s %s", kParams[id].name, num, kParams[id].unit);
}

const uint8_t* slotData(int s) { return reinterpret_cast<const uint8_t*>(btsave::kQspiBase + btsave::kSlotOffset[s]); }

// Writes next to the slot not holding the newest saved values and verifies it through the memory-mapped QSPI; true
// when the slot reads back exactly, and then next are the saved values. The engine keeps running (nothing it runs
// lives in QSPI).
bool writeSaved(const btsave::Saved& next) {
    const uint32_t t0 = System::GetNow();
    const int target = gSavedSlot == 0 ? 1 : 0;
    const uint32_t off = btsave::kSlotOffset[target];
    const uint32_t bytes = btsave::kHeaderBytes + btsave::kPayloadBytes;
    uint8_t payload[btsave::kPayloadBytes];
    btsave::Encode(next, payload);
    btsave::Build(gSlotBuf, gSavedSeq + 1, payload);
    bool ok = gQspi->Erase(off, off + btsave::kSlotBytes) == QSPIHandle::Result::OK &&
              gQspi->Write(off, bytes, gSlotBuf) == QSPIHandle::Result::OK;
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t*>(btsave::kQspiBase + off), int32_t(btsave::kSlotBytes));
    uint32_t written = 0;
    ok = ok && btsave::Valid(slotData(target), written) && written == gSavedSeq + 1 &&
         std::memcmp(slotData(target), gSlotBuf, bytes) == 0;
    if (!ok) {
        btusb::Log("saved write failed");
        return false;
    }
    gSaved = next;
    gSavedSlot = target;
    gSavedSeq = written;
    btusb::Log("saved slot %c seq %lu %lu ms", char('A' + target), (unsigned long)written,
               (unsigned long)(System::GetNow() - t0));
    return true;
}

// SAVE (save = true) or CLEAR of the ids; the replies are queued for sendReplies.
void handleSave(bool save, const uint8_t* ids, size_t count) {
    bool valid = count > 0;
    for (size_t i = 0; i < count; ++i) valid = valid && btparam::IsSaveable(ids[i]);
    if (!valid) {
        btusb::Log("link: bad save message %02X", save ? btlink::kSave : btlink::kClear);
        gStoreResult = btlink::kStoreBadRequest;
        return;
    }
    btsave::Saved next = gSaved;
    for (size_t i = 0; i < count; ++i) {
        next.has[ids[i]] = save;
        next.value[ids[i]] = save ? gStore->Value(ids[i]) : 0.0f;
    }
    bool changed = false;
    for (uint8_t id = 0; id < kParamCount; ++id)
        changed = changed || next.has[id] != gSaved.has[id] || (next.has[id] && next.value[id] != gSaved.value[id]);
    if (changed && !writeSaved(next)) {
        gStoreResult = btlink::kStoreFailed;
        return;
    }
    for (size_t i = 0; i < count; ++i) gSendSaved[ids[i]] = true;
    gStoreResult = btlink::kStoreOk;
}

// One complete SysEx message from the host.
void handleHost(const uint8_t* m, size_t n) {
    if (btlink::IsIdentityRequest(m, n)) {
        gSendIdentity = true;
        return;
    }
    const int cmd = btlink::PedalCommand(m, n);
    if (cmd < 0) return;
    if (cmd == btlink::kHello && n == 6) {
        gOnHello();
    } else if (cmd == btlink::kDfu && n == 6) {
        btusb::EnterDfu();
    } else if (cmd == btlink::kGet || cmd == btlink::kSet || cmd == btlink::kDump) {
        const size_t want = cmd == btlink::kGet ? 8 : cmd == btlink::kSet ? 10 : 6;
        if (n != want || (cmd != btlink::kDump && btlink::GetU14(m + 5) >= kParamCount)) {
            btusb::Log("link: bad param message %02X", cmd);
        } else if (cmd == btlink::kDump) {
            gDumpNext = 0;
        } else {
            const uint8_t id = uint8_t(btlink::GetU14(m + 5));
            if (cmd == btlink::kSet) applyMidi(id, btparam::FromCode(kParams[id], uint16_t(btlink::GetU14(m + 7))));
            gReplies.push(id);
        }
    } else if (cmd == btlink::kWatch) {
        if (n == 7 && m[5] <= 1) {
            gWatch = m[5] == 1;
            if (gWatch)
                for (uint8_t id = 0; id < kParamCount; ++id)
                    gSentCode[id] = btparam::ToCode(kParams[id], gStore->Value(id));
        } else {
            btusb::Log("link: bad watch message");
        }
    } else if (cmd == btlink::kSave || cmd == btlink::kClear) {
        uint8_t ids[btparam::kSaveableCount];
        const size_t bytes = n - 6;  // PedalCommand: n >= 6
        const size_t count = bytes / 2 <= size_t(btparam::kSaveableCount) ? btlink::DecodeIds(m + 5, bytes, ids) : 0;
        handleSave(cmd == btlink::kSave, ids, count);
    } else if (cmd == btlink::kSavedQuery && n == 6) {
        for (uint8_t id = 0; id < kParamCount; ++id) gSendSaved[id] = btparam::IsSaveable(id);
        gSendSavedEnd = true;
    } else {
        btusb::Log("link: unknown command %02X", cmd);
    }
}

// One Control Change from any channel: NRPN data entry for any parameter, or a parameter's own CC.
void handleCc(const btmidi::Cc& c) {
    uint16_t number, code;
    if (gNrpn.Feed(c, number, code)) {
        if (number >= kParamCount) return;  // number = the parameter id (MSB id >> 7, LSB id & 0x7F)
        const uint8_t id = uint8_t(number);
        applyMidi(id, btparam::FromCode(kParams[id], code));
        return;
    }
    const int id = btparam::FindByCc(c.number);
    if (id < 0) return;
    applyMidi(uint8_t(id), btparam::FromCc(kParams[id], c.value));
}

// VALUE replies, saved values and the dump, appended while they fit; the rest waits for the next pass. Each VALUE
// carries the value at send time.
void sendReplies() {
    btlink::TxBuffer& t = btusb::Tx();
    while (const uint8_t* id = gReplies.front()) {
        const uint16_t code = btparam::ToCode(kParams[*id], gStore->Value(*id));
        if (!t.AppendValue(*id, code)) break;
        gSentCode[*id] = code;
        uint8_t done;
        gReplies.pop(done);
    }
    // SAVED per flagged id, then STORE, then SAVED END: the editor sees the saved values before the status.
    uint8_t id = 0;
    for (; id < kParamCount; ++id) {
        if (!gSendSaved[id]) continue;
        const btparam::ParamDef& d = kParams[id];
        const bool has = gSaved.has[id];
        if (!t.AppendSaved(id, has, btparam::ToCode(d, has ? gSaved.value[id] : d.def))) break;
        gSendSaved[id] = false;
    }
    if (id == kParamCount) {
        if (gStoreResult >= 0 && t.AppendStore(uint8_t(gStoreResult))) gStoreResult = -1;
        if (gStoreResult < 0 && gSendSavedEnd) {
            if (t.AppendSavedEnd(uint16_t(btparam::kSaveableCount))) gSendSavedEnd = false;
        }
    }
    while (gDumpNext >= 0 && gDumpNext < kParamCount) {
        const btparam::ParamDef& d = kParams[gDumpNext];
        if (!t.AppendParamInfo(d, btparam::ToCode(d, gStore->Value(d.id)))) return;
        ++gDumpNext;
    }
    if (gDumpNext == kParamCount && t.AppendDumpEnd(kParamCount)) gDumpNext = -1;
}

// The WATCH stream: a VALUE for every parameter whose code moved since the last one sent, while send room lasts.
void sendWatch() {
    btlink::TxBuffer& t = btusb::Tx();
    for (uint8_t id = 0; id < kParamCount; ++id) {
        const btparam::ParamDef& d = kParams[id];
        const uint16_t code = btparam::ToCode(d, gStore->Value(id));
        if (!btlink::WatchDue(d, gSentCode[id], code)) continue;
        if (!t.AppendValue(id, code)) break;
        gSentCode[id] = code;
    }
}

}  // namespace

void Init(btparam::Store& store, QSPIHandle& qspi, void (*readKnobs)(float k[6]), void (*onHello)()) {
    gStore = &store;
    gQspi = &qspi;
    gReadKnobs = readKnobs;
    gOnHello = onHello;
    for (uint16_t& c : gSentCode) c = btlink::kNoCode;
}

[[gnu::cold]] void LoadSaved() {
    uint32_t seq = 0;
    const int s = btsave::Newest(slotData(0), slotData(1), seq);
    if (s < 0) {
        btusb::Log("saved empty");
        return;
    }
    btsave::Decode(slotData(s) + btsave::kHeaderBytes, btsave::PayloadLength(slotData(s)), gSaved);
    gSavedSlot = s;
    gSavedSeq = seq;
    for (uint8_t id = 0; id < kParamCount; ++id)
        if (gSaved.has[id]) gStore->Load(id, gSaved.value[id]);
    btusb::Log("saved slot %c seq %lu: %d params", char('A' + s), (unsigned long)seq, gSaved.Count());
}

void Feed(uint8_t byte) {
    if (gHostIn.Feed(byte)) handleHost(gHostIn.data(), gHostIn.size());
    btmidi::Cc cc;
    if (gCc.Feed(byte, cc)) handleCc(cc);
}

void Send(uint32_t nowMs) {
    if (gSendIdentity && btusb::Tx().AppendIdentity(btboard::kBoardId, btfw::kBuildId)) gSendIdentity = false;
    sendReplies();
    if (gWatch && nowMs - gLastWatch >= 20) {
        gLastWatch = nowMs;
        sendWatch();
    }
}

}  // namespace bthost
