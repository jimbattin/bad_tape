// The audio round-trip latency (daisy/Makefile APP=latency, build/<BOARD>-latency/bad_tape_latency.bin), measured
// through a cable from the left output to the left input. The pedal adds no latency of its own (no lookahead, no
// buffering beyond libDaisy's), so the round trip is the pedal's input-to-output delay: the click leaves through the
// output DMA and the DAC and comes back through the ADC and the input DMA, the same chain a guitar sample takes.
// For each sample rate and block size: audio restarted, 500 ms to settle, then kTrials clicks. A trial captures
// kCapture input samples, kPre of them before a one-sample click of kClick on both outputs, and reports the delay to
// the input's largest excursion (its peak: the codec filters' group delay, refined by a parabola) and to the first
// sample above a tenth of it (onset; the filters' pre-ringing stays below that).
// Log lines (usb_log.py; daisy/README.md "Latency"):
//   at power-on and on HELLO "bad_tape_latency <board> seed <version> <build id>";
//   per trial "lat SR B t T peak P.PP smp U us onset O smp amp A noise N" (A, N: 1/10000 full scale, signed A);
//   per trial with nothing above 4 x the noise "lat SR B t T no click (peak P noise N)";
//   per configuration "lat SR B peak min P.PP max Q.QQ smp U..V us"; then "lat pass N done seed <version>".
// Passes repeat forever, 2 s apart. The link's DFU message restarts into the bootloader (checked between trials).
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "board.h"
#include "firmware.h"
#include "usb_link.h"

using namespace daisy;

namespace {

constexpr size_t kCapture = 8192;  // samples per trial (171 ms at 48 kHz)
constexpr size_t kPre = 256;       // of them before the click
constexpr float kClick = 0.25f;
constexpr int kTrials = 5;
constexpr uint32_t kSettleMs = 500, kTrialGapMs = 200, kPassGapMs = 2000;

constexpr struct {
    SaiHandle::Config::SampleRate sr;
    uint32_t hz;
    size_t block;
} kConfigs[] = {
    {SaiHandle::Config::SampleRate::SAI_48KHZ, 48000, 48}, {SaiHandle::Config::SampleRate::SAI_48KHZ, 48000, 24},
    {SaiHandle::Config::SampleRate::SAI_48KHZ, 48000, 16}, {SaiHandle::Config::SampleRate::SAI_48KHZ, 48000, 8},
    {SaiHandle::Config::SampleRate::SAI_96KHZ, 96000, 48}, {SaiHandle::Config::SampleRate::SAI_96KHZ, 96000, 32},
    {SaiHandle::Config::SampleRate::SAI_96KHZ, 96000, 24},
};

btboard::Board hw;

// Callback -> main loop: the capture. Main arms a trial (gCapStart, then gArmed with release); the callback fills
// gCap and sets gDone (release) after its last sample.
float gCap[kCapture];
std::atomic<uint32_t> gN{0};  // samples since StartAudio (callback writes)
uint32_t gCapStart = 0;       // absolute sample of gCap[0]
std::atomic<bool> gArmed{false}, gDone{false};

void Callback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    const uint32_t n0 = gN.load(std::memory_order_relaxed);
    const bool armed = gArmed.load(std::memory_order_acquire);
    bool done = false;
    for (size_t i = 0; i < size; ++i) {
        float y = 0.0f;
        if (armed) {
            const uint32_t k = n0 + uint32_t(i) - gCapStart;  // wraps past kCapture before the start
            if (k < kCapture) {
                gCap[k] = in[0][i];
                if (k == kPre) y = kClick;
                if (k == kCapture - 1) done = true;
            }
        }
        out[0][i] = out[1][i] = y;
    }
    gN.store(n0 + uint32_t(size), std::memory_order_relaxed);
    if (done) {
        gArmed.store(false, std::memory_order_relaxed);
        gDone.store(true, std::memory_order_release);
    }
}

const char* SeedName(DaisySeed::BoardVersion v) {
    switch (v) {
        case DaisySeed::BoardVersion::DAISY_SEED: return "rev4-ak4556";
        case DaisySeed::BoardVersion::DAISY_SEED_1_1: return "1.1-wm8731";
        case DaisySeed::BoardVersion::DAISY_SEED_2_DFM: return "2dfm-pcm3060";
        case DaisySeed::BoardVersion::DAISY_SEED_1_2: return "1.2-pcm3060";
        case DaisySeed::BoardVersion::DAISY_SEED_3: return "3-tac5242";
    }
    return "unknown";
}

void logBoot() {
    btusb::Log("bad_tape_latency %s seed %s %s", btboard::kName, SeedName(hw.seed.CheckBoardVersion()),
               btfw::kBuildId);
}

// One click; returns the peak delay in hundredths of a sample, or -1 when nothing came back above the noise.
long trial(uint32_t hz, size_t block, int t) {
    gDone.store(false, std::memory_order_relaxed);
    gCapStart = gN.load(std::memory_order_relaxed) + 4 * uint32_t(block);  // a few blocks ahead of the callback
    gArmed.store(true, std::memory_order_release);
    while (!gDone.load(std::memory_order_acquire)) btusb::ServeHelloDfu(logBoot);

    double dc = 0.0;
    for (size_t k = 0; k < kPre; ++k) dc += gCap[k];
    dc /= double(kPre);
    float noise = 0.0f;
    for (size_t k = 0; k < kPre; ++k) noise = std::fmax(noise, std::fabs(float(gCap[k] - dc)));
    size_t p = kPre;
    for (size_t k = kPre; k < kCapture - 1; ++k)  // the parabola reads gCap[p + 1]
        if (std::fabs(gCap[k] - dc) > std::fabs(gCap[p] - dc)) p = k;
    const float peak = float(gCap[p] - dc);
    if (std::fabs(peak) <= 4.0f * noise) {
        btusb::Log("lat %lu %u t %d no click (peak %d noise %d)", (unsigned long)hz, unsigned(block), t,
                   int(std::lround(peak * 1e4f)), int(std::lround(noise * 1e4f)));
        btusb::ServeHelloDfu(logBoot);
        return -1;
    }
    // Parabola through the peak and its neighbours, in the peak's polarity.
    const float s = peak > 0.0f ? 1.0f : -1.0f;
    const float a = s * float(gCap[p - 1] - dc), b = s * peak, c = s * float(gCap[p + 1] - dc);
    const float den = a - 2.0f * b + c;
    const float frac = den != 0.0f ? 0.5f * (a - c) / den : 0.0f;
    const long cs = std::lround((float(p - kPre) + frac) * 100.0f);
    size_t onset = kPre;
    while (onset < p && std::fabs(gCap[onset] - dc) <= 0.1f * std::fabs(peak)) ++onset;
    const long us = std::lround(double(cs) * 1e4 / double(hz));
    btusb::Log("lat %lu %u t %d peak %ld.%02ld smp %ld us onset %u smp amp %d noise %d", (unsigned long)hz,
               unsigned(block), t, cs / 100, cs % 100, us, unsigned(onset - kPre), int(std::lround(peak * 1e4f)),
               int(std::lround(noise * 1e4f)));
    btusb::ServeHelloDfu(logBoot);
    return cs;
}

void runPass(unsigned pass) {
    for (const auto& c : kConfigs) {
        hw.seed.StopAudio();
        hw.seed.SetAudioSampleRate(c.sr);
        hw.seed.SetAudioBlockSize(c.block);
        gArmed.store(false, std::memory_order_relaxed);
        gN.store(0, std::memory_order_relaxed);
        hw.seed.StartAudio(Callback);
        btusb::ServeHelloDfuFor(kSettleMs, logBoot);
        long lo = -1, hi = -1;
        for (int t = 0; t < kTrials; ++t) {
            const long cs = trial(c.hz, c.block, t);
            if (cs >= 0) {
                lo = lo < 0 ? cs : std::min(lo, cs);
                hi = std::max(hi, cs);
            }
            btusb::ServeHelloDfuFor(kTrialGapMs, logBoot);
        }
        if (lo >= 0)
            btusb::Log("lat %lu %u peak min %ld.%02ld max %ld.%02ld smp %ld..%ld us", (unsigned long)c.hz,
                       unsigned(c.block), lo / 100, lo % 100, hi / 100, hi % 100,
                       std::lround(double(lo) * 1e4 / double(c.hz)), std::lround(double(hi) * 1e4 / double(c.hz)));
        btusb::ServeHelloDfu(logBoot);
    }
    btusb::Log("lat pass %u done seed %s", pass, SeedName(hw.seed.CheckBoardVersion()));
    btusb::ServeHelloDfuFor(kPassGapMs, logBoot);
}

}  // namespace

int main() {
    // 480 MHz; the block size and rate are set per configuration.
    btfw::Start(hw, 48, SaiHandle::Config::SampleRate::SAI_48KHZ);
    btusb::Init(hw.seed);
    logBoot();
    for (unsigned pass = 1;; ++pass) runPass(pass);
}
