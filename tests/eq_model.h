// The EQ's model (docs/EQ.md section 2): each band's analogue prototype (RBJ cookbook bell and shelves) evaluated
// under the bilinear transform prewarped at its frequency, the five in series. Closed forms, independent of
// dsp/eq.h's Design: the reference the digital EQ is tested against.
#pragma once

#include <cmath>
#include <complex>

#include "dsp/eq.h"
#include "test_util.h"

namespace bttest {

using Cplx = std::complex<double>;

// One band: gain (dB), frequency (Hz: a bell's centre, a shelf's half-gain point) and Q.
struct BandSettings {
    double gain = 0.0, freq = 1000.0, q = 1.0;
};
// The EQ's settings other than eq_enable and eq_position, in processing order (btdsp::eq::kKind); the defaults are
// the table's.
struct EqSettings {
    BandSettings band[btdsp::eq::kBands] = {{0.0, 100.0, btdsp::eq::kShelfQ},
                                           {0.0, 250.0, 1.0},
                                           {0.0, 1000.0, 1.0},
                                           {0.0, 3000.0, 1.0},
                                           {0.0, 6000.0, btdsp::eq::kShelfQ}};
};

// The band's response at f: the prototype at s = j tan(pi f / fs) / tan(pi f0 / fs), fs = kFs.
inline Cplx BandModel(btdsp::eq::Kind k, double f, const BandSettings& b) {
    constexpr double kPi = btdsp::kPiT<double>;
    const double fs = double(kFs);
    const double A = std::pow(10.0, b.gain / 40.0), r = std::sqrt(A);
    const Cplx s(0.0, std::tan(kPi * f / fs) / std::tan(kPi * b.freq / fs));
    switch (k) {
        case btdsp::eq::kBell: return (s * s + s * (A / b.q) + 1.0) / (s * s + s / (A * b.q) + 1.0);
        case btdsp::eq::kLowShelf: return A * (s * s + (r / b.q) * s + A) / (A * s * s + (r / b.q) * s + 1.0);
        case btdsp::eq::kHighShelf: return A * (A * s * s + (r / b.q) * s + 1.0) / (s * s + (r / b.q) * s + A);
    }
    return 1.0;
}

inline double Db(Cplx h) { return 20.0 * std::log10(std::abs(h)); }

inline double EqModelDb(const EqSettings& s, double f) {
    Cplx h = 1.0;
    for (int b = 0; b < btdsp::eq::kBands; ++b) h *= BandModel(btdsp::eq::kKind[b], f, s.band[b]);
    return Db(h);
}

}  // namespace bttest
