// The DSP benchmark (daisy/Makefile APP=bench, build/<BOARD>-bench/bad_tape_bench.bin). No audio: the board, USB and
// SDRAM come up with the pedal's startup (firmware.h: FZ, the MPU), then every stage of the pedal's per-block work is
// timed with the DWT cycle counter on kBlockSize blocks of uniform noise (+-0.5): 200 ms of warm-up blocks, then 2 s of
// measured ones, interrupts masked inside each measured block (USB is serviced between blocks and between stages). The
// memory placement matches the pedal's (DSP objects in DTCM, the delay line and the pool's slow region in SDRAM, the
// fast region in D2, the timed block functions in ITCM like AudioCallback), plus a fast region in SDRAM for the pool
// placement comparison.
// Log lines (usb_log.py; daisy/README.md "Performance"):
//   at power-on and on HELLO "bad_tape_bench <board> sysclk N MHz <build id>";
//   "bench pass N budget B cycles" (B: cycles in one block), then per stage
//   "bench NAME cycles avg A max M permille avg P.p max Q.q" (permille of B), then "bench pass N done".
// Passes repeat forever, 2 s apart. The link's DFU message restarts into the bootloader (checked between stages).
#include <cstdint>
#include <cstdio>

#include "board.h"
#include "controls.h"
#include "dsp/engine.h"
#include "firmware.h"
#include "memory.h"
#include "params.h"
#include "usb_link.h"

using namespace daisy;
using btdsp::kBlockSize;
static_assert(btdsp::kSampleRate == 96000.0f, "the codec runs at SAI_96KHZ");

namespace {

// Block counts for a duration, so a stage covers the same audio time whatever kBlockSize is.
constexpr int MsBlocks(int ms) { return int(size_t(ms) * size_t(btdsp::kSampleRate) / 1000 / kBlockSize); }
constexpr int kWarmBlocks = MsBlocks(200), kRunBlocks = MsBlocks(2000), kNoiseBlocks = 64;
constexpr int kSweepBlocks = MsBlocks(20);  // the sweeps' half period
constexpr uint32_t kPassGapMs = 2000;

// The worst-case settings (daisy/README.md "Performance"; the device runs use the same list).
constexpr struct {
    uint8_t id;
    float v;
} kWorst[] = {
    {btparam::kDelayTime, 2000},        {btparam::kDelayFeedback, 110},    {btparam::kDelayColor, 100},
    {btparam::kDelayMix, 100},          {btparam::kReverbMix, 100},        {btparam::kPlateSize, 100},
    {btparam::kPlateChorus, 100},       {btparam::kPlateMidDecay, 20},     {btparam::kHallSize, 39},
    {btparam::kHallRtMid, 20},          {btparam::kHallWander, 100},       {btparam::kHallSpin, 100},
    {btparam::kHallSpread, 100},        {btparam::kHallEchoLevel1, 0},     {btparam::kHallEchoLevel2, 0},
    {btparam::kHallEchoLevel3, 0},      {btparam::kHallEchoLevel4, 0},     {btparam::kCanyonMasterDelay, 100},
    {btparam::kCanyonMasterDepth, 100}, {btparam::kCanyonFeedback, 100},  {btparam::kDelayType, 0},
    {btparam::kEqEnable, 1},            {btparam::kEqPosition, 1},         {btparam::kEqLowGain, 12},
    {btparam::kEqMidGain1, 12},         {btparam::kEqMidGain2, 12},        {btparam::kEqMidGain3, 12},
    {btparam::kEqHighGain, 12},
};

btboard::Board hw;
BT_DTCM btdsp::Engine gEngine;
BT_DTCM btdsp::Delay gDelay;
BT_DTCM btdsp::Plate gPlate;
BT_DTCM btdsp::Hall gHall;
BT_DTCM btdsp::Canyon gCanyon;
BT_DTCM btdsp::Eq gEq;
// The resample stage's reverb: its folded wet is its input (L = R = in / (2 kReverbFold)).
struct PassReverb final : btdsp::Reverb {
    void Init(float, const btdsp::ReverbPool&) override {}
    void Reset() override {}
    void SetParam(uint8_t, float) override {}
    void Snap() override {}
    BT_ITCM void Process(const float* in, float* outL, float* outR, size_t n) override {
        for (size_t i = 0; i < n; ++i) outL[i] = outR[i] = in[i] * (0.5f / btdsp::kReverbFold);
    }
};
BT_DTCM PassReverb gPassReverb;
BT_DTCM btdsp::HalfRateReverb gResample;
BT_DTCM btctl::ToggleFilter gToggleFilter[3];
BT_DTCM float gNoise[kBlockSize * kNoiseBlocks];
BT_DTCM float gL[kBlockSize], gR[kBlockSize], gOut[kBlockSize];
BT_SDRAM float gDelayMem[btdsp::kDelaySamples];
BT_SDRAM float gReverbSlow[btdsp::kReverbSlowFloats];
BT_D2 float gReverbFast[btdsp::kReverbFastFloats];
BT_SDRAM float gFastInSdram[btdsp::kReverbFastFloats];  // the alternative fast region
btdsp::Reverb* gStageReverb = nullptr;                 // the algorithm blockReverb runs (stageReverb)
uint32_t gBudget = 1;                                  // cycles in one block period

void logBoot() {
    btusb::Log("bad_tape_bench %s sysclk %u MHz %s", btboard::kName, unsigned(System::GetSysClkFreq() / 1000000),
               btfw::kBuildId);
}

// The timed blocks: k is the block (-kWarmBlocks .. kRunBlocks - 1, measured from 0), in its noise.

// The controls the callback reads: the board, the toggle filters, the knobs.
BT_ITCM void blockControls(int, const float*) {
    volatile float sink = 0.0f;
    hw.Process();
    for (int t = 0; t < 3; ++t) gToggleFilter[t].Process(hw.Toggle(t));
    float s = 0.0f;
    for (int k = 0; k < 6; ++k) s += hw.Knob(k);
    sink = s;
    (void)sink;
}
// gDelay alone.
BT_ITCM void blockDelay(int, const float* in) {
    for (size_t i = 0; i < kBlockSize; ++i) gOut[i] = gDelay.Process(in[i]);
}
// gDelay with its input colors (every scope all).
BT_ITCM void blockDelayAll(int, const float* in) {
    for (size_t i = 0; i < kBlockSize; ++i) gOut[i] = gDelay.Process(gDelay.ColorInput(in[i]));
}
// gStageReverb alone, at kReverbRate: half a block's samples, the block period's worth.
BT_ITCM void blockReverb(int, const float* in) { gStageReverb->Process(in, gL, gR, kBlockSize / 2); }
// The half-band resampler around a reverb that costs nothing.
BT_ITCM void blockResample(int, const float* in) { gResample.Process(gPassReverb, in, gOut, kBlockSize); }
// gEq alone.
BT_ITCM void blockEq(int, const float* in) {
    for (size_t i = 0; i < kBlockSize; ++i) gOut[i] = gEq.Process(in[i]);
}
// gEq alone, every gain flipped between +-12 dB and every frequency between its range's ends every kSweepBlocks, so
// every section recomputes its coefficients on every control tick.
BT_ITCM void blockEqSweep(int k, const float* in) {
    if (k % kSweepBlocks == 0) {
        using namespace btparam;
        const bool up = (k / kSweepBlocks) & 1;
        for (int b = 0; b < btdsp::eq::kBands; ++b) {
            const ParamDef& f = kParams[btdsp::eq::kFreqId[b]];
            gEq.SetParam(btdsp::eq::kGainId[b], up ? 12.0f : -12.0f);
            gEq.SetParam(f.id, up ? f.max : f.min);
        }
    }
    blockEq(k, in);
}
// The whole chain.
BT_ITCM void blockEngine(int, const float* in) { gEngine.Process(in, gOut, kBlockSize); }
// The whole chain, SW3 moved to the Plate at measured block 0.
BT_ITCM void blockEngineSw3(int k, const float* in) {
    if (k == 0) gEngine.SetParam(btparam::kReverbAlgo, 0.0f);
    gEngine.Process(in, gOut, kBlockSize);
}
// The whole chain, knob 4 swept 10 <-> 2000 ms every kSweepBlocks.
BT_ITCM void blockEngineTimeSweep(int k, const float* in) {
    if (k % kSweepBlocks == 0) gEngine.SetParam(btparam::kDelayTime, (k / kSweepBlocks) & 1 ? 10.0f : 2000.0f);
    gEngine.Process(in, gOut, kBlockSize);
}
// The whole chain, the EQ's five gains flipped between +-12 dB every kSweepBlocks.
BT_ITCM void blockEngineEqSweep(int k, const float* in) {
    if (k % kSweepBlocks == 0) {
        const float g = (k / kSweepBlocks) & 1 ? -12.0f : 12.0f;
        for (uint8_t id : btdsp::eq::kGainId) gEngine.SetParam(id, g);
    }
    gEngine.Process(in, gOut, kBlockSize);
}

// block(k, in) once per block, k from -kWarmBlocks. Logs the average and maximum cycles of the measured blocks. noipa:
// block stays an indirect call, so GCC cannot clone this (SRAM) function and inline an ITCM block into it.
[[gnu::noipa]] void timeStage(const char* name, void (*block)(int k, const float* in)) {
    uint64_t sum = 0;
    uint32_t max = 0;
    for (int k = -kWarmBlocks; k < kRunBlocks; ++k) {
        const float* in = gNoise + size_t((k + kWarmBlocks) % kNoiseBlocks) * kBlockSize;
        __disable_irq();
        __asm__ volatile("" ::: "memory");
        const uint32_t t0 = DWT->CYCCNT;
        block(k, in);
        __asm__ volatile("" ::: "memory");
        const uint32_t c = DWT->CYCCNT - t0;
        __enable_irq();
        if (k < 0) continue;
        sum += c;
        if (c > max) max = c;
    }
    const uint32_t avg = uint32_t(sum / kRunBlocks);
    const uint32_t pa = uint32_t(uint64_t(avg) * 10000 / gBudget), pm = uint32_t(uint64_t(max) * 10000 / gBudget);
    btusb::Log("bench %s cycles avg %lu max %lu permille avg %lu.%lu max %lu.%lu", name, (unsigned long)avg,
               (unsigned long)max, (unsigned long)(pa / 10), (unsigned long)(pa % 10), (unsigned long)(pm / 10),
               (unsigned long)(pm % 10));
    btusb::ServeHelloDfu(logBoot);
}

// The delay alone at 2000 ms, 110 % feedback, type at full color (knob 6 fully clockwise; digital: every crush
// option on). all: every scope all, the input colors timed too (forward only).
void stageDelay(const char* name, bool reverse, btctl::DelayType type, bool all = false) {
    using btparam::kParams;
    btdsp::DelayTargets t;
    t.timeMs = 2000.0f;
    t.feedback = 1.1f;
    t.type = type;
    t.color = 1.0f;
    t.crush.driveDb = kParams[btparam::kDelayCrushDrive].def;
    t.crush.rateHz = kParams[btparam::kDelayCrushRate].def;
    t.crush.aaRatio = kParams[btparam::kDelayCrushAa].def;
    t.crush.toneRatio = kParams[btparam::kDelayCrushTone].def;
    t.crush.open = t.crush.clock = t.crush.wrap = t.crush.bits = 1.0f;
    t.crush.mask = int(kParams[btparam::kDelayCrushMask].def);
    t.reverse = reverse;
    if (all) t.crushScope = t.tapeScope = t.bbdScope = 1.0f;
    gDelay.Init(btdsp::kSampleRate, gDelayMem);
    gDelay.SetTargets(t);
    gDelay.Snap();
    timeStage(name, all ? blockDelayAll : blockDelay);
}

void stageReverb(const char* name, btdsp::Reverb& r, bool worst, float* fast) {
    const btdsp::ReverbPool pool{fast, gReverbSlow};
    btdsp::ClearPool(pool, 0, btdsp::kReverbPoolFloats);
    r.Init(btdsp::kReverbRate, pool);
    if (worst)  // the other groups' ids are ignored
        for (const auto& s : kWorst) r.SetParam(s.id, btparam::Clamp(btparam::kParams[s.id], s.v));
    r.Snap();
    gStageReverb = &r;
    timeStage(name, blockReverb);
}

// The EQ alone at kWorst (every section on); sweep: every gain and frequency flips between its extremes every 20 ms
// (blockEqSweep), so every section recomputes its coefficients on every control tick.
void stageEq(const char* name, bool sweep) {
    gEq.Init(btdsp::kSampleRate);
    for (const auto& s : kWorst) gEq.SetParam(s.id, btparam::Clamp(btparam::kParams[s.id], s.v));  // others ignored
    gEq.Snap();
    timeStage(name, sweep ? blockEqSweep : blockEq);
}

// The engine at the worst settings (the BBD, the heaviest delay type, at full color), SW1 = mode, SW3 = algo.
void initEngine(int mode, int algo) {
    gEngine.Init(btdsp::kSampleRate, gDelayMem, btdsp::ReverbPool{gReverbFast, gReverbSlow});
    for (const auto& s : kWorst) gEngine.SetParam(s.id, s.v);
    gEngine.SetParam(btparam::kDelayMode, float(mode));
    gEngine.SetParam(btparam::kReverbAlgo, float(algo));
}

void runPass(unsigned pass) {
    btusb::Log("bench pass %u budget %lu cycles", pass, (unsigned long)gBudget);
    btusb::ServeHelloDfu(logBoot);
    timeStage("controls", blockControls);

    for (bool reverse : {false, true}) {
        for (btctl::DelayType type : {btctl::DelayType::kBbd, btctl::DelayType::kTape, btctl::DelayType::kDigital}) {
            char name[32];
            snprintf(name, sizeof name, "delay %s %s", reverse ? "rev" : "fwd", btctl::Name(type));
            stageDelay(name, reverse, type);
        }
    }
    for (btctl::DelayType type : {btctl::DelayType::kBbd, btctl::DelayType::kTape, btctl::DelayType::kDigital}) {
        char name[32];
        snprintf(name, sizeof name, "delay fwd %s all", btctl::Name(type));
        stageDelay(name, false, type, true);
    }

    stageReverb("plate default", gPlate, false, gReverbFast);
    stageReverb("plate worst", gPlate, true, gReverbFast);
    stageReverb("plate worst sdram", gPlate, true, gFastInSdram);
    stageReverb("hall default", gHall, false, gReverbFast);
    stageReverb("hall worst", gHall, true, gReverbFast);
    stageReverb("hall worst sdram", gHall, true, gFastInSdram);
    stageReverb("canyon default", gCanyon, false, gReverbFast);
    stageReverb("canyon worst", gCanyon, true, gReverbFast);
    stageReverb("canyon worst sdram", gCanyon, true, gFastInSdram);
    gResample.Reset();
    timeStage("resample", blockResample);
    stageEq("eq worst", false);
    stageEq("eq sweep", true);

    static const char* const kEngineNames[3][3] = {
        {"engine rev plate", "engine rev hall", "engine rev canyon"},
        {"engine off plate", "engine off hall", "engine off canyon"},
        {"engine fwd plate", "engine fwd hall", "engine fwd canyon"}};
    for (int mode = 2; mode >= 0; --mode) {
        for (int algo = 0; algo < 3; ++algo) {
            initEngine(mode, algo);
            timeStage(kEngineNames[mode][algo], blockEngine);
        }
    }

    // SW3 Hall -> Plate at measured block 0: the fade-out, the pool clear and the fade-in are inside the run.
    initEngine(0, 1);
    timeStage("engine sw3 switch", blockEngineSw3);
    // Knob 4 swept 10 <-> 2000 ms every 20 ms: tape and BBD glide, digital crossfades.
    for (btctl::DelayType type : {btctl::DelayType::kBbd, btctl::DelayType::kTape, btctl::DelayType::kDigital}) {
        char name[40];
        snprintf(name, sizeof name, "engine time sweep %s", btctl::Name(type));
        initEngine(2, 1);
        gEngine.SetParam(btparam::kDelayType, float(uint8_t(type)));
        timeStage(name, blockEngineTimeSweep);
    }
    // The EQ's gains flipped every 20 ms inside the whole chain (reverse, Plate: the pedal's worst row).
    initEngine(0, 0);
    timeStage("engine eq sweep rev plate", blockEngineEqSweep);

    btusb::Log("bench pass %u done", pass);
    btusb::ServeHelloDfuFor(kPassGapMs, logBoot);
}

}  // namespace

int main() {
    btfw::Start(hw, kBlockSize, SaiHandle::Config::SampleRate::SAI_96KHZ);
    btusb::Init(hw.seed);
    hw.seed.adc.Start();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    gBudget = uint32_t(uint64_t(System::GetSysClkFreq()) * kBlockSize / uint32_t(btdsp::kSampleRate));

    btdsp::XorShift32 rng{0x12345678u};
    for (float& v : gNoise) v = rng.Uniform() - 0.5f;
    for (btctl::ToggleFilter& f : gToggleFilter) f.Init(1);

    logBoot();
    for (unsigned pass = 1;; ++pass) runPass(pass);
}
