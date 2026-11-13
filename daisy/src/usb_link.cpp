// usb_link.h. gTx is CPU-copied to the OTG FIFO (libDaisy's usbd_conf.c: dma_enable = DISABLE), so cacheable SRAM
// needs no maintenance.
#include "usb_link.h"

#include <cstdarg>
#include <cstdio>

#include "hid/usb_midi.h"
#include "spsc_ring.h"

using namespace daisy;

namespace btusb {
namespace {

MidiUsbTransport gMidi;
UsbHandle* gUsb = nullptr;
btlink::TxBuffer gTx[2];  // gTx[gFill] fills while the other may be in flight
int gFill = 0;
SpscRing<uint8_t, 512> gRx;  // USB IRQ -> main loop: incoming MIDI bytes (dropped when full)
btlink::SysExAssembler<64> gHostIn;  // ServeHelloDfu's host messages

// USB IRQ: every incoming MIDI byte goes to the main loop.
void onMidiRx(uint8_t* data, size_t size, void*) {
    for (size_t i = 0; i < size; ++i) gRx.push(data[i]);
}

}  // namespace

void Init(DaisySeed& seed) {
    gUsb = &seed.usb_handle;
    MidiUsbTransport::Config cfg;
    cfg.periph = MidiUsbTransport::Config::INTERNAL;
    gMidi.Init(cfg);
    gMidi.StartRx(onMidiRx, nullptr);
}

void Poll() {
    if (!gMidi.RxActive()) {
        gMidi.FlushRx();
        gMidi.StartRx(onMidiRx, nullptr);
    }
}

bool ReadByte(uint8_t& b) { return gRx.pop(b); }

btlink::TxBuffer& Tx() { return gTx[gFill]; }

void Log(const char* fmt, ...) {
    static char buf[128];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    gTx[gFill].AppendLog(buf);
}

// A successful start means the previous transfer finished, so the other buffer can be refilled; a busy endpoint keeps
// the batch for the next pass. (libDaisy's MidiUsbHandler::SendMessage retries a busy endpoint with a delay, so the
// link bypasses it.)
void Flush() {
    btlink::TxBuffer& t = gTx[gFill];
    if (t.size() == 0) return;
    // The transfer only reads the buffer; libDaisy's API is not const-correct.
    if (gUsb->TransmitInternal(const_cast<uint8_t*>(t.data()), t.size()) != UsbHandle::Result::OK) return;
    gFill ^= 1;
    gTx[gFill].Clear();
}

void EnterDfu() {
    Log("dfu reset");
    for (int i = 0; i < 100; ++i) {
        Flush();
        System::Delay(1);
    }
    System::ResetToBootloader(System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
    for (;;) {
    }
}

void ServeHelloDfu(void (*onHello)()) {
    Poll();
    uint8_t b;
    while (ReadByte(b)) {
        if (!gHostIn.Feed(b)) continue;
        const int cmd = btlink::PedalCommand(gHostIn.data(), gHostIn.size());
        if (cmd == btlink::kHello && gHostIn.size() == 6) onHello();
        else if (cmd == btlink::kDfu && gHostIn.size() == 6) EnterDfu();
    }
    Flush();
}

void ServeHelloDfuFor(uint32_t ms, void (*onHello)()) {
    const uint32_t t0 = System::GetNow();
    while (System::GetNow() - t0 < ms) ServeHelloDfu(onHello);
}

}  // namespace btusb
