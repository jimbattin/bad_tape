// MIDI channel-message parsing for parameter control (docs/MIDI.md): Control Change extraction from a raw MIDI byte
// stream and 14-bit NRPN decoding. Host-portable (no libDaisy, no allocation), covered by tests/midi_test.cpp.
// libDaisy's MidiUsbTransport delivers the raw bytes of each USB-MIDI packet with the CIN stripped
// (libDaisy/src/hid/usb_midi.cpp), so running status is handled here.
#pragma once

#include <cstdint>

namespace btmidi {

struct Cc {
    uint8_t channel, number, value;  // channel 0-15
};

// Emits every complete Control Change (Bn cc v, running status included). Other channel messages' data bytes are
// consumed by length (8n/9n/An/En: 2, Cn/Dn: 1), never read as CC. Realtime bytes (>= F8) are ignored anywhere.
// F0 enters SysEx: data bytes are ignored until F7 or another status byte. F1-F7 clear running status; F1/F3
// swallow 1 data byte, F2 swallows 2.
class CcParser {
public:
    bool Feed(uint8_t b, Cc& out) {
        if (b >= 0xF8) return false;
        if (b >= 0x80) {
            sysex_ = b == 0xF0;
            count_ = 0;
            if (b >= 0xF0) {
                status_ = 0;
                swallow_ = b == 0xF2 ? 2 : (b == 0xF1 || b == 0xF3) ? 1 : 0;
            } else {
                status_ = b;
                swallow_ = 0;
                need_ = (b & 0xF0) == 0xC0 || (b & 0xF0) == 0xD0 ? 1 : 2;
            }
            return false;
        }
        if (sysex_) return false;
        if (swallow_ > 0) {
            --swallow_;
            return false;
        }
        if (status_ == 0) return false;
        data_[count_++] = b;
        if (count_ < need_) return false;
        count_ = 0;
        if ((status_ & 0xF0) != 0xB0) return false;
        out = {uint8_t(status_ & 0x0F), data_[0], data_[1]};
        return true;
    }

private:
    uint8_t status_ = 0, need_ = 2, count_ = 0, swallow_ = 0;
    uint8_t data_[2] = {0, 0};
    bool sysex_ = false;
};

// NRPN from CCs, per channel: 99 number MSB and 98 number LSB select an NRPN; 101/100 select an RPN (later data entry
// is ignored). 6 (data MSB) emits code = msb << 7; 38 (data LSB) emits code = msb << 7 | lsb with the last data MSB
// since the selection (0 if none). Data entry is ignored until both 99 and 98 have arrived; 127/127 (the null NRPN)
// deselects. 96/97 (increment/decrement) are ignored.
class NrpnDecoder {
public:
    // True with number = msb << 7 | lsb and the 14-bit code when c completes a data entry.
    bool Feed(const Cc& c, uint16_t& number, uint16_t& code) {
        State& s = ch_[c.channel & 0x0F];
        switch (c.number) {
            case 99:
                s.numMsb = c.value;
                s.haveMsb = true;
                s.rpn = false;
                s.dataMsb = 0;
                return false;
            case 98:
                s.numLsb = c.value;
                s.haveLsb = true;
                s.rpn = false;
                s.dataMsb = 0;
                return false;
            case 101:
            case 100:
                s.rpn = true;
                return false;
            case 6:
            case 38:
                if (s.rpn || !s.haveMsb || !s.haveLsb || (s.numMsb == 127 && s.numLsb == 127)) return false;
                if (c.number == 6) s.dataMsb = c.value;
                number = uint16_t(s.numMsb << 7 | s.numLsb);
                code = uint16_t(s.dataMsb << 7 | (c.number == 38 ? c.value : 0));
                return true;
            default:
                return false;
        }
    }

private:
    struct State {
        uint8_t numMsb = 0, numLsb = 0, dataMsb = 0;
        bool haveMsb = false, haveLsb = false, rpn = false;
    };
    State ch_[16];
};

}  // namespace btmidi
