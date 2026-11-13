// The USB side of the host link (link.h), shared by all three images (bad_tape.cpp, bench.cpp, latency.cpp): the
// Seed's own USB port as a USB MIDI device, incoming bytes through an IRQ ring, outgoing messages double-buffered and
// sent without ever blocking, and the DFU restart. Main loop only, except the receive IRQ. libDaisy-dependent.
#pragma once

#include <cstdint>

#include "daisy_seed.h"
#include "link.h"

namespace btusb {

// MidiUsbTransport INTERNAL (the Seed's own port); reception started; keeps &seed.usb_handle for sending.
void Init(daisy::DaisySeed& seed);
// Restarts reception if libDaisy stopped it (its ring overflowed).
void Poll();
// The next byte from the USB IRQ ring; false when it is empty.
bool ReadByte(uint8_t& b);
// The send buffer being filled.
btlink::TxBuffer& Tx();
// A LOG message appended to Tx(); dropped if it does not fit. Never blocks.
__attribute__((format(printf, 1, 2))) void Log(const char* fmt, ...);
// Starts the filled buffer's transfer if the USB endpoint is free; never blocks.
void Flush();
// Logs "dfu reset", Flush()es for ~100 ms, then restarts into the Daisy bootloader's DFU mode (no timeout).
[[noreturn]] void EnterDfu();
// The bench's and the latency image's whole host session: Poll, then every complete message: HELLO calls onHello (it
// logs the boot line), DFU EnterDfu()s; then Flush.
void ServeHelloDfu(void (*onHello)());
// ServeHelloDfu repeatedly for ms milliseconds.
void ServeHelloDfuFor(uint32_t ms, void (*onHello)());

}  // namespace btusb
