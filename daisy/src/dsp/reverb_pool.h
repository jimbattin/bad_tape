// The reverb pool sized for the largest algorithm: each region holds the largest of the three algorithms' footprints
// (hall.h, plate.h, canyon.h), so adding a larger one grows the pool. The engine owns and clears it (NOLOAD on the
// Seed: the fast region in D2 SRAM, the slow one in SDRAM). Host-portable.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstring>

#include "../memory.h"
#include "canyon.h"
#include "hall.h"
#include "plate.h"
#include "reverb.h"

namespace btdsp {

constexpr size_t kReverbFastFloats = std::max({kHallFastFloats, kPlateFastFloats, kCanyonFastFloats});
constexpr size_t kReverbSlowFloats = std::max({kHallSlowFloats, kPlateSlowFloats, kCanyonSlowFloats});
constexpr size_t kReverbPoolFloats = kReverbFastFloats + kReverbSlowFloats;

// Zeroes pool floats [from, from + n): indices below kReverbFastFloats are in p.fast, the rest in p.slow.
BT_ITCM inline void ClearPool(const ReverbPool& p, size_t from, size_t n) {
    const size_t to = from + n;
    if (from < kReverbFastFloats) {
        const size_t e = std::min(to, kReverbFastFloats);
        std::memset(p.fast + from, 0, (e - from) * sizeof(float));
        from = e;
    }
    if (from < to) std::memset(p.slow + (from - kReverbFastFloats), 0, (to - from) * sizeof(float));
}

}  // namespace btdsp
