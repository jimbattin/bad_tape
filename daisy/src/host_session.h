// The pedal's host session, on the main loop: incoming USB MIDI (SysEx host messages, link.h; CC and NRPN writes,
// midi.h) and the parameter writes they make (held against the knobs, param_store.h); the saved power-up values in
// QSPI (saved_store.h: read at boot, written on SAVE/CLEAR); and everything sent back, all SysEx: the Identity Reply,
// VALUE for GET/SET, the dump, SAVED/STORE and the WATCH stream. CC and NRPN writes get no reply: controllers such as
// the Morningstar MC6 Pro act on incoming CCs. Main loop only. libDaisy-dependent (QSPI).
#pragma once

#include <cstdint>

#include "daisy_seed.h"
#include "param_store.h"

namespace bthost {

// store: the main loop's parameter values; qspi: the flash holding the saved values; readKnobs: the knob
// positions a MIDI write is held against; onHello: logs the image's boot line.
void Init(btparam::Store& store, daisy::QSPIHandle& qspi, void (*readKnobs)(float k[6]), void (*onHello)());
void LoadSaved();          // boot, before the first publish: the newest saved values into the store (logged)
void Feed(uint8_t byte);   // one incoming USB MIDI byte: SysEx host messages, CC and NRPN writes
void Send(uint32_t nowMs); // Identity Reply, replies, dump, SAVED/STORE, and WATCH every 20 ms, while send room lasts

}  // namespace bthost
