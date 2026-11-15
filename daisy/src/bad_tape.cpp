// Bad Tape pedal firmware (Cleveland Music Co. HotHouse, board.h): EQ -> delay -> reverb, or delay -> reverb -> EQ
// (eq_position; dsp/engine.h), on the left input, copied to both outputs (MIMO). The controls, the LEDs and the log
// lines are documented in daisy/README.md, the MIDI and SysEx protocol in docs/MIDI.md.
//
// Two threads of control:
// - AudioCallback (the audio DMA interrupt, 96 kHz, 32-sample blocks), here: reads the controls (30 ms toggle
//   hysteresis), hands parameter changes to btdsp::Engine, runs it, drives the LEDs and measures the load and the
//   underruns.
// - The main loop: USB (usb_link.h), the host session (host_session.cpp: SysEx host messages, MIDI CC/NRPN, the
//   saved power-up values, the replies and the WATCH stream), the parameter values and their knob/MIDI arbitration
//   (param_store.h; the controls take effect from kControlSettleMs after the audio starts) and the once-a-second log
//   lines.
// They share only atomics, each with one writer:
// - callback -> main loop: gKnob, gToggle, gFsHeld, gFs1Presses, gUnderruns, gPeak, gLoadAvg, gLoadMax (relaxed);
// - main loop -> callback: gPeakReset, gLoadReset, gParam (relaxed) and gParamGen, incremented (release) after a
//   publish that stored a value; the callback scans gParam only when gParamGen moved (acquire), so it misses no update
//   and skips the kParamCount-entry scan otherwise.
// Memory (memory.h; the map is in daisy/README.md): AudioCallback and the DSP's per-sample and per-tick functions run
// from ITCM; the engine and the callback's own state live in DTCM; the delay line, the SOS line and the reverb pool's
// slow region in SDRAM, the pool's fast region in D2 SRAM. FPSCR.FZ is on (denormals flush to zero).
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "board.h"
#include "controls.h"
#include "dsp/engine.h"
#include "firmware.h"
#include "host_session.h"
#include "memory.h"
#include "param_store.h"
#include "params.h"
#include "usb_link.h"

using namespace daisy;
using btparam::kParamCount;
using btparam::kParams;

namespace {

constexpr uint32_t kControlSettleMs = 200;  // after StartAudio: knob filters and the 30 ms toggle filter have settled
constexpr float kLedFloorDb = -30.0f;  // LED2 off at or below this input level (dBFS; 1.0 = 0 dBFS)
constexpr float kLedFullDb = -10.0f;   // LED2 fully on at or above this input level (dBFS)
constexpr float kLedReleaseS = 0.15f;  // LED2 envelope release time constant
constexpr uint32_t kSosBlinkBlocks = 750;  // LED2 SOS blink half-period: 250 ms at the block rate
static_assert(kSosBlinkBlocks * btdsp::kBlockSize * 4 == size_t(btdsp::kSampleRate), "LED2 blinks at 2 Hz");
// Per-block release factor of the LED2 envelope (about 0.9978).
const float gLedRelease = std::exp(-float(btdsp::kBlockSize) / (kLedReleaseS * btdsp::kSampleRate));
static_assert(btdsp::kSampleRate == 96000.0f, "the codec runs at SAI_96KHZ");
static_assert(btctl::kToggleStableBlocks * btdsp::kBlockSize * 1000 == 30 * size_t(btdsp::kSampleRate),
              "the toggle filter holds 30 ms");
static_assert(btctl::kShiftPressBlocks * btdsp::kBlockSize * 1000 == 5 * size_t(btdsp::kSampleRate) &&
                  btctl::kShiftReleaseBlocks * btdsp::kBlockSize * 1000 == 30 * size_t(btdsp::kSampleRate),
              "FS2 accepts a press after 5 ms and a release after 30 ms");

btboard::Board hw;

// Main loop only: the parameter values and their arbitration.
btparam::Store gStore;
uint32_t gFs1Seen = 0;  // FS1 presses applied to gStore

// Audio callback only (after boot).
BT_DTCM Led gLed1;  // Update once per block (Init at the block rate)
BT_DTCM Led gLed2;  // Update every 8th sample (Init at btdsp::kSampleRate / 8: 12 kHz PWM updates)
BT_DTCM CpuLoadMeter gLoad;
BT_DTCM uint32_t gBlockTicks = 0;  // System::GetTick() ticks in one block period (set before StartAudio)
BT_DTCM btctl::ToggleFilter gToggleFilter[3];
BT_DTCM btctl::ShiftFilter gShiftFilter;
BT_DTCM float gInEnv = 0.0f;    // peak envelope of the input, linear (1.0 = 0 dBFS)
BT_DTCM float gPeakAcc = 0.0f;  // input peak since the main loop last read gPeak
BT_DTCM btdsp::Engine gEngine;
BT_DTCM uint32_t gSosBlink = 0;  // blocks since SOS went on (LED2 blinks)
BT_DTCM uint32_t gParamSeen = 0xFFFFFFFFu;  // the gParamGen of the last gParam scan
BT_SDRAM_BANK float gDelayMem[btdsp::kDelaySamples];
BT_SDRAM float gSosMem[btdsp::kSosSamples];
BT_D2 float gReverbFast[btdsp::kReverbFastFloats];
BT_SDRAM float gReverbSlow[btdsp::kReverbSlowFloats];

// Shared with the main loop: in SRAM (AXI), relaxed except gParamGen.
// Callback -> main loop.
std::atomic<float> gKnob[6];               // knobs 1-6, 0..1
std::atomic<uint8_t> gToggle[3]{1, 1, 1};  // SW1-SW3 filtered: 0 down, 1 middle, 2 up
std::atomic<uint8_t> gFsHeld{0};           // bit 0 FS1, bit 1 FS2 (debounced by gShiftFilter)
std::atomic<uint32_t> gFs1Presses{0};      // FS1 presses since boot
std::atomic<uint32_t> gUnderruns{0};       // blocks since boot whose callback ran longer than one block period
std::atomic<float> gPeak{0.0f};            // gPeakAcc, published every block
std::atomic<float> gLoadAvg{0.0f}, gLoadMax{0.0f};  // gLoad's, published every block
// Main loop -> callback.
std::atomic<bool> gPeakReset{false};  // restart gPeakAcc
std::atomic<bool> gLoadReset{false};  // restart gLoad
// The parameter values (gStore's), written by the main loop only, and gParamGen, incremented after every publish that
// stored a value.
std::atomic<float> gParam[kParamCount];
std::atomic<uint32_t> gParamGen{0};

// The codec's sample word in bits.
unsigned bitDepth(SaiHandle::Config::BitDepth d) {
    switch (d) {
        case SaiHandle::Config::BitDepth::SAI_16BIT: return 16;
        case SaiHandle::Config::BitDepth::SAI_24BIT: return 24;
        case SaiHandle::Config::BitDepth::SAI_32BIT: return 32;
    }
    return 0;
}

void logBoot() {
    btusb::Log("bad_tape %s sysclk %u MHz audio %u Hz %u bit block %u %s", btboard::kName,
               unsigned(System::GetSysClkFreq() / 1000000), unsigned(hw.seed.AudioSampleRate()),
               bitDepth(hw.seed.AudioSaiHandle().GetConfig().bit_depth), unsigned(hw.seed.AudioBlockSize()),
               btfw::kBuildId);
}

void readKnobs(float k[6]) {
    for (int i = 0; i < 6; ++i) k[i] = gKnob[i].load(std::memory_order_relaxed);
}

// The controls set their parameters (from kControlSettleMs after StartAudio).
void updateControls() {
    const uint32_t presses = gFs1Presses.load(std::memory_order_relaxed);
    for (; gFs1Seen != presses; ++gFs1Seen) gStore.ToggleBypass();
    float k[6];
    readKnobs(k);
    uint8_t t[3];
    for (int i = 0; i < 3; ++i) t[i] = gToggle[i].load(std::memory_order_relaxed);
    const bool shift = gFsHeld.load(std::memory_order_relaxed) & 2;
    const float sosBefore = gStore.Value(btparam::kDelaySos);
    const uint16_t taken = gStore.Update(k, t, shift);
    if (gStore.Value(btparam::kDelaySos) != sosBefore)
        btusb::Log("fs2 sos %s", gStore.Value(btparam::kDelaySos) > 0.5f ? "on" : "off");
    if (!taken) return;
    for (int s = 0; s < btparam::kSlots; ++s) {
        if (!(taken & (1u << s))) continue;
        const char* name = kParams[gStore.SlotParam(s)].name;
        if (s < btparam::kKnobs)
            btusb::Log("knob %d takes %s", s + 1, name);
        else
            btusb::Log("knob %d fs2 takes %s", s - btparam::kKnobs + 1, name);
    }
}

// The parameter values the callback hands to the engine. The increment comes after the stores (release), so a callback
// that sees it (acquire) sees them.
void publishParams() {
    bool stored = false;
    for (uint8_t id = 0; id < kParamCount; ++id) {
        const float v = gStore.Value(id);
        if (gParam[id].load(std::memory_order_relaxed) != v) {
            gParam[id].store(v, std::memory_order_relaxed);
            stored = true;
        }
    }
    if (stored) gParamGen.fetch_add(1, std::memory_order_release);
}

BT_ITCM void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    gLoad.OnBlockStart();
    const uint32_t start = System::GetTick();
    hw.Process();
    // FS1 counts toward bypass; FS2 is the knobs' shift and the SOS tap (ShiftFilter, param_store.h).
    if (hw.FootswitchRising(0)) gFs1Presses.fetch_add(1, std::memory_order_relaxed);

    for (int k = 0; k < 6; ++k) gKnob[k].store(hw.Knob(k), std::memory_order_relaxed);
    for (int t = 0; t < 3; ++t)
        gToggle[t].store(gToggleFilter[t].Process(hw.Toggle(t)), std::memory_order_relaxed);
    gFsHeld.store(uint8_t((hw.FootswitchPressed(0) ? 1 : 0) | (gShiftFilter.Process(hw.FootswitchRaw(1)) ? 2 : 0)),
                  std::memory_order_relaxed);

    // Only after a publish: every store is followed by an increment, and the scan runs after the increment is seen.
    const uint32_t gen = gParamGen.load(std::memory_order_acquire);
    if (gen != gParamSeen) {
        gParamSeen = gen;
        for (uint8_t id = 0; id < kParamCount; ++id) {
            const float v = gParam[id].load(std::memory_order_relaxed);
            if (v != gEngine.Param(id)) gEngine.SetParam(id, v);
        }
    }

    float peak = 0.0f;
    for (size_t i = 0; i < size; i++) peak = std::fmax(peak, std::fabs(in[0][i]));
    gEngine.Process(in[0], out[0], size);
    std::copy(out[0], out[0] + size, out[1]);

    if (gPeakReset.exchange(false, std::memory_order_relaxed)) gPeakAcc = 0.0f;
    gPeakAcc = std::fmax(gPeakAcc, peak);
    gPeak.store(gPeakAcc, std::memory_order_relaxed);

    gLed1.Set(gEngine.Param(btparam::kBypass) > 0.5f ? 0.0f : 1.0f);
    gLed1.Update();
    // LED2: while SOS is on (delay_sos with a mode), a 2 Hz blink, lit from the switch-on; otherwise the input's peak
    // envelope, duty linear in dB from kLedFloorDb (off) to kLedFullDb (full); the cbrt cancels Led::Set's cube.
    gInEnv = std::fmax(peak, gInEnv * gLedRelease);  // instant attack, kLedReleaseS release
    const bool sos = gEngine.Param(btparam::kDelaySos) > 0.5f && gEngine.Param(btparam::kDelaySosMode) > 0.5f;
    if (sos) {
        gLed2.Set((gSosBlink / kSosBlinkBlocks) % 2 == 0 ? 1.0f : 0.0f);
        ++gSosBlink;
    } else {
        gSosBlink = 0;
        const float db = 20.0f * std::log10(std::fmax(gInEnv, 1e-6f));
        const float level = std::fmin(std::fmax((db - kLedFloorDb) / (kLedFullDb - kLedFloorDb), 0.0f), 1.0f);
        gLed2.Set(std::cbrt(level));
    }
    for (size_t i = 0; i < size; i += 8) gLed2.Update();

    if (System::GetTick() - start > gBlockTicks) gUnderruns.fetch_add(1, std::memory_order_relaxed);
    // A Reset just before OnBlockEnd takes its first-cycle path: the published loads are never NaN.
    if (gLoadReset.exchange(false, std::memory_order_relaxed)) gLoad.Reset();
    gLoad.OnBlockEnd();
    gLoadAvg.store(gLoad.GetAvgCpuLoad(), std::memory_order_relaxed);
    gLoadMax.store(gLoad.GetMaxCpuLoad(), std::memory_order_relaxed);
}

int pct(float v) { return int(lroundf(v * 100.0f)); }

void logControls() {
    int k[6];
    for (int i = 0; i < 6; ++i) k[i] = pct(gKnob[i].load(std::memory_order_relaxed));
    const uint8_t fs = gFsHeld.load(std::memory_order_relaxed);
    btusb::Log("ctl k %d %d %d %d %d %d sw %u %u %u fs %d %d time %d ms type %s color %d", k[0], k[1], k[2], k[3],
               k[4], k[5], unsigned(gToggle[0].load(std::memory_order_relaxed)),
               unsigned(gToggle[1].load(std::memory_order_relaxed)),
               unsigned(gToggle[2].load(std::memory_order_relaxed)), fs & 1, (fs >> 1) & 1,
               int(lroundf(gStore.Value(btparam::kDelayTime))),
               btctl::Name(btctl::ToDelayType(uint8_t(gStore.Value(btparam::kDelayType)))),
               int(lroundf(gStore.Value(btparam::kDelayColor))));
}

void logStatus() {
    const float peak = gPeak.load(std::memory_order_relaxed);
    gPeakReset.store(true, std::memory_order_relaxed);
    const float avg = gLoadAvg.load(std::memory_order_relaxed), max = gLoadMax.load(std::memory_order_relaxed);
    gLoadReset.store(true, std::memory_order_relaxed);
    const int db = peak > 0.0f ? std::max(int(lroundf(20.0f * log10f(peak))), -120) : -120;
    const auto pos = [](uint8_t id) { return uint8_t(gStore.Value(id)); };
    btusb::Log("load avg %d max %d permille underruns %lu bypass %s delay %s route %s algo %s in %d dBFS sos %s %s",
               int(avg * 1000), int(max * 1000), (unsigned long)gUnderruns.load(std::memory_order_relaxed),
               gStore.Value(btparam::kBypass) > 0.5f ? "on" : "off",
               btctl::Name(btctl::ToDelayMode(pos(btparam::kDelayMode))),
               btctl::Name(btctl::ToRouting(pos(btparam::kRouting))),
               btctl::Name(btctl::ToAlgo(pos(btparam::kReverbAlgo))), db,
               btctl::Name(btctl::ToSosMode(pos(btparam::kDelaySosMode))),
               gStore.Value(btparam::kDelaySos) > 0.5f ? "on" : "off");
}

}  // namespace

int main() {
    btfw::Start(hw, btdsp::kBlockSize, SaiHandle::Config::SampleRate::SAI_96KHZ);
    gLed1.Init(btboard::Board::kLed1, false, btdsp::kSampleRate / btdsp::kBlockSize);
    gLed2.Init(btboard::Board::kLed2, false, btdsp::kSampleRate / 8);

    btusb::Init(hw.seed);
    logBoot();

    gStore.Init();
    bthost::Init(gStore, hw.seed.qspi, readKnobs, logBoot);
    bthost::LoadSaved();
    publishParams();
    gEngine.Init(btdsp::kSampleRate, gDelayMem, btdsp::SosLine{gSosMem, btdsp::kSosSamples},
                 btdsp::ReverbPool{gReverbFast, gReverbSlow});  // after hw.Init (SDRAM is up), before StartAudio

    for (int i = 0; i < 3; ++i) {
        gLed1.Set(1.0f);
        gLed1.Update();
        System::Delay(100);
        gLed1.Set(0.0f);
        gLed1.Update();
        System::Delay(100);
    }
    gLoad.Init(hw.seed.AudioSampleRate(), hw.seed.AudioBlockSize());
    gBlockTicks = uint32_t(uint64_t(System::GetTickFreq()) * btdsp::kBlockSize / uint32_t(btdsp::kSampleRate));
    for (btctl::ToggleFilter& f : gToggleFilter) f.Init(1);
    hw.seed.adc.Start();
    hw.seed.StartAudio(AudioCallback);
    const uint32_t audioStart = System::GetNow();

    uint32_t lastLog = audioStart;
    for (;;) {
        btusb::Poll();
        uint8_t rx;
        while (btusb::ReadByte(rx)) bthost::Feed(rx);
        const uint32_t now = System::GetNow();
        if (now - audioStart >= kControlSettleMs) updateControls();
        bthost::Send(now);
        if (now - lastLog >= 1000) {
            lastLog = now;
            logControls();
            logStatus();
        }
        publishParams();
        btusb::Flush();
    }
}
