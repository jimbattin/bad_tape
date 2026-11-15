// Offline render of the whole DSP chain (daisy/src/dsp/engine.h), the same code the pedal runs.
//   render IN.wav OUT.wav [--set NAME=VALUE]... [--block N] [--tail SECONDS]
// IN.wav: 96 kHz (btdsp::kSampleRate), PCM 16/24/32 or float32; channel 0 (the pedal's L In) is rendered. OUT.wav:
// mono float32, 96 kHz.
// --set: a parameter by name (daisy/src/params.h, docs/MIDI.md) in engineering units, applied in order before
// processing as a MIDI write is on the pedal (param_store.h): a link source such as hall_space also sets its targets,
// and a later --set of either wins. Others stay at their defaults. --block: samples per Process call (default
// btdsp::kBlockSize, 32; 1..4096). --tail: seconds of silence appended to the input (default 0).
// Exit: 0 ok, 1 I/O or format error, 2 usage error.
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "dsp/engine.h"
#include "param_store.h"
#include "params.h"
#include "wav.h"

namespace {

constexpr uint32_t kRate = uint32_t(btdsp::kSampleRate);

[[noreturn]] void Usage(const char* msg) {
    if (msg) std::fprintf(stderr, "render: %s\n", msg);
    std::fprintf(stderr, "usage: render IN.wav OUT.wav [--set NAME=VALUE]... [--block N] [--tail SECONDS]\n");
    std::exit(2);
}

bool ParseNumber(const char* s, double& v) {
    char* end;
    errno = 0;
    v = std::strtod(s, &end);
    return *s && !*end && errno == 0;
}

}  // namespace

int main(int argc, char** argv) {
    const char* inPath = nullptr;
    const char* outPath = nullptr;
    std::vector<std::pair<int, float>> sets;
    size_t block = btdsp::kBlockSize;
    double tail = 0.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--set" || a == "--block" || a == "--tail") {
            if (i + 1 >= argc) Usage((a + " needs a value").c_str());
            const char* v = argv[++i];
            double x;
            if (a == "--set") {
                const char* eq = std::strchr(v, '=');
                if (!eq) Usage("--set needs NAME=VALUE");
                const std::string name(v, eq);
                const int id = btparam::FindByName(name.c_str());
                if (id < 0) {
                    std::fprintf(stderr, "render: unknown parameter '%s' (see docs/MIDI.md)\n", name.c_str());
                    return 2;
                }
                const btparam::ParamDef& d = btparam::kParams[id];
                if (!ParseNumber(eq + 1, x) || !std::isfinite(x) || x < d.min || x > d.max) {
                    std::fprintf(stderr, "render: %s=%s outside %g..%g\n", d.name, eq + 1, double(d.min),
                                 double(d.max));
                    return 2;
                }
                sets.emplace_back(id, float(x));
            } else if (a == "--block") {
                if (!ParseNumber(v, x) || x < 1 || x > 4096 || x != std::floor(x)) Usage("--block must be 1..4096");
                block = size_t(x);
            } else {
                if (!ParseNumber(v, x) || !(x >= 0) || x > 3600) Usage("--tail must be 0..3600 seconds");
                tail = x;
            }
        } else if (!a.empty() && a[0] == '-') {
            Usage(("unknown option " + a).c_str());
        } else if (!inPath) {
            inPath = argv[i];
        } else if (!outPath) {
            outPath = argv[i];
        } else {
            Usage("too many arguments");
        }
    }
    if (!outPath) Usage(nullptr);

    std::vector<float> x;
    uint32_t rate = 0;
    std::string err;
    if (!btwav::ReadWav(inPath, x, rate, err)) {
        std::fprintf(stderr, "render: %s\n", err.c_str());
        return 1;
    }
    if (rate != kRate) {
        std::fprintf(stderr, "render: %s is %u Hz, need %u\n", inPath, unsigned(rate), unsigned(kRate));
        return 1;
    }
    x.resize(x.size() + size_t(std::lround(tail * kRate)), 0.0f);

    std::vector<float> delayMem(btdsp::kDelaySamples), sosMem(btdsp::kSosSamples);
    std::vector<float> reverbFast(btdsp::kReverbFastFloats), reverbSlow(btdsp::kReverbSlowFloats);
    btdsp::Engine engine;
    engine.Init(btdsp::kSampleRate, delayMem.data(), btdsp::SosLine{sosMem.data(), sosMem.size()},
                btdsp::ReverbPool{reverbFast.data(), reverbSlow.data()});
    btparam::Store store;
    store.Init();
    const float knobs[6] = {};  // no knobs: nothing a --set holds is ever taken back
    for (const auto& s : sets) store.SetFromMidi(uint8_t(s.first), s.second, knobs);
    for (uint8_t id = 0; id < btparam::kParamCount; ++id) engine.SetParam(id, store.Value(id));
    std::vector<float> y(x.size());
    for (size_t i = 0; i < x.size(); i += block) {
        const size_t n = x.size() - i < block ? x.size() - i : block;
        engine.Process(x.data() + i, y.data() + i, n);
    }
    if (!btwav::WriteWavFloat(outPath, y.data(), y.size(), kRate, err)) {
        std::fprintf(stderr, "render: %s\n", err.c_str());
        return 1;
    }
    return 0;
}
