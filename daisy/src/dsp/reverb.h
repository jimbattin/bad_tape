// The reverb interface, implemented by every algorithm (hall.h, plate.h, canyon.h), the ReverbPool of delay-line
// memory they share (sized in reverb_pool.h): only one algorithm runs at a time, so each lays its lines over the whole
// pool at Init. The pool's owner (btdsp::Engine) clears it: in full at Init and in slices on every algorithm switch.
// The algorithms never write pool memory outside Process. Also AlgoParams, the parameter table plumbing they share;
// their building blocks are in blocks.h. No libDaisy, no allocation.
#pragma once

#include <cstddef>
#include <cstdint>

#include "../memory.h"
#include "../params.h"
#include "blocks.h"

namespace btdsp {

// The pedal's mono fold of the stereo wet: every algorithm's L and R are decorrelated (tests: StereoIsDecorrelated):
// (L + R) / sqrt(2) keeps the level.
constexpr float kReverbFold = 0.70710678f;

// Caller-owned: fast holds kReverbFastFloats floats, slow kReverbSlowFloats (reverb_pool.h). NOLOAD on the Seed.
struct ReverbPool {
    float* fast;
    float* slow;
};

// One reverb algorithm. Only one runs at a time and all share one ReverbPool (the engine switches between them).
class Reverb {
public:
    // Lays out its lines in the pool, sets every parameter to its table default, Reset(), Snap(). Never writes pool
    // memory: the pool's owner (Engine) zeroes it.
    virtual void Init(float sampleRate, const ReverbPool& pool) = 0;
    // Zeroes every filter, LFO and random state outside the pool and restarts the control tick; glides and ramps are
    // left for Snap(), which the engine always calls next. Parameters kept; pool memory untouched.
    virtual void Reset() = 0;
    // v already btparam::Clamp()ed; ids of other groups are ignored.
    virtual void SetParam(uint8_t id, float v) = 0;
    // Every smoother and glide jumps to its target.
    virtual void Snap() = 0;
    // Any n; in must not alias outL or outR.
    virtual void Process(const float* in, float* outL, float* outR, size_t n) = 0;

protected:
    // Protected and non-virtual: no deleting destructor, so no operator delete (no heap) is linked.
    ~Reverb() = default;
};

// An algorithm's parameter range [First, Last]: table values and their control-rate smoothers. Which values smooth
// (ticked(i)) and what each chases (target(i)) are the algorithm's; i = id - First. The per-tick members are
// always_inline: a template's out-of-line copy loses BT_ITCM (../memory.h).
template <uint8_t First, uint8_t Last> class AlgoParams {
public:
    static constexpr uint8_t kFirst = First;
    static constexpr int kCount = Last - First + 1;
    static constexpr bool Owns(uint8_t id) { return id >= First && id <= Last; }
    void Set(uint8_t id, float v) { t_[id - First] = v; }
    [[gnu::always_inline]] inline float T(uint8_t id) const { return t_[id - First]; }
    [[gnu::always_inline]] inline float At(int i) const { return t_[i]; }
    [[gnu::always_inline]] inline float Smoothed(uint8_t id) const { return sm_[id - First].Value(); }
    // Smoother i at target(i), with its table smoothMs at the control rate, where ticked(i).
    template <class Ticked, class Target> void Snap(float fs, Ticked ticked, Target target) {
        const float ctlRate = fs / float(kReverbControlInterval);
        for (int i = 0; i < kCount; ++i)
            if (ticked(i)) sm_[i].Init(ctlRate, btparam::SmoothSeconds(uint8_t(First + i)), target(i));
    }
    // One control tick: smoother i steps toward target(i), where ticked(i).
    template <class Ticked, class Target> [[gnu::always_inline]] inline void Tick(Ticked ticked, Target target) {
        for (int i = 0; i < kCount; ++i)
            if (ticked(i)) sm_[i].Process(target(i));
    }
    // set(id, table default), First..Last in order.
    template <class F> static void ForEachDefault(F set) {
        for (int id = First; id <= Last; ++id) set(uint8_t(id), btparam::kParams[id].def);
    }

private:
    float t_[kCount] = {};  // table units
    Smoother sm_[kCount];   // only the ticked ids
};

}  // namespace btdsp
