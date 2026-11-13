// The parameter values and the knob/MIDI arbitration (docs/MIDI.md "Arbitration").
//
// Knob slots (params.h): slot s < 6 is knob s+1's primary, slot s >= 6 the FS2 secondary of knob s-5; each drives the
// parameter its assignment parameter (knob_N_primary/secondary) resolves to (ResolveSlot: a Space id follows SW3).
// - Primaries are what you see is what you get. A MIDI/SysEx write to a parameter on a slot holds it until that
//   slot's knob moves more than kPickup from where it was at the write; then the knob takes it back.
// - FS2 (shift, btctl::ShiftFilter): while it is held a knob drives only its secondary, once the knob has moved more
//   than kPickup from where it was at the press. A knob moved more than kPickup while FS2 was held keeps its primary
//   until it moves more than kPickup from where it was at the release.
// - A parameter on more than one slot follows the knob that moved last: a slot writing it holds the others.
// - A newly assigned parameter keeps its value until its knob moves more than kPickup. At the first Update the
//   primaries take their knobs' positions (a saved value of a parameter on a primary is replaced).
// - A toggle that changes position always wins; an unchanged toggle leaves a MIDI value alone.
// Links (params.h): a change of a source (knob or MIDI) writes its targets, except targets on a slot (a knob owns
// them); a target written over MIDI is held until its source moves more than kPickup of its travel from where it was
// at the write. A saved power-up value (saved_store.h, Load) is applied before the first Update; a saved link target
// is held like a MIDI write, anchored where its source is at its first change.
#pragma once

#include <cmath>
#include <cstdint>

#include "controls.h"
#include "params.h"

namespace btparam {

constexpr float kPickup = 0.03f;  // knob travel (0..1) that takes a parameter back from MIDI

class Store {
public:
    // Values = defaults, nothing held, toggles and slots not yet seen (the first Update sets every toggle and
    // primary parameter).
    void Init() {
        for (const ParamDef& d : kParams) {
            value_[d.id] = d.def;
            linkHeld_[d.id] = false;
            linkAnchor_[d.id] = 0.0f;
            anchorPending_[d.id] = false;
            pendingHold_[d.id] = false;
        }
        for (int s = 0; s < kSlots; ++s) {
            slot_[s] = kNoParam;
            assign_[s] = 0xFF;
            slotHeld_[s] = false;
            slotAnchor_[s] = 0.0f;
        }
        for (int k = 0; k < kKnobs; ++k) {
            shiftAnchor_[k] = 0.0f;
            shiftMoved_[k] = false;
        }
        shift_ = false;
        started_ = false;
        for (uint8_t& t : lastToggle_) t = 0xFF;
        active_ = btctl::ToAlgo(uint8_t(value_[kReverbAlgo]));
    }

    float Value(uint8_t id) const { return id < kParamCount ? value_[id] : 0.0f; }
    // The parameter slot s drives as of the last Update, or kNoParam.
    uint8_t SlotParam(int s) const { return s >= 0 && s < kSlots ? slot_[s] : kNoParam; }

    // A saved power-up value (saved_store.h), before the first Update: Clamp()ed. A link target is held against its
    // source, anchored where the source is at its first change (the knob position the first Update reads).
    void Load(uint8_t id, float v) {
        if (id >= kParamCount) return;
        value_[id] = Clamp(kParams[id], v);
        if (LinkSource(id) >= 0) {
            linkHeld_[id] = true;
            anchorPending_[id] = true;
        }
    }

    // MIDI/SysEx write: Clamp()ed and held against every slot it is on until that slot's knob moves more than kPickup
    // from knobs[] as of now (on no slot: held when SW3 or the first Update puts it on one). Returns the stored value.
    float SetFromMidi(uint8_t id, float v, const float knobs[6]) {
        if (id >= kParamCount) return 0.0f;
        value_[id] = Clamp(kParams[id], v);
        bool onSlot = false;
        for (int s = 0; s < kSlots; ++s) {
            if (slot_[s] != id) continue;
            HoldSlot(s, knobs);
            onSlot = true;
        }
        if (!onSlot && IsAssignable(id)) pendingHold_[id] = true;
        const int src = LinkSource(id);
        if (src >= 0) {
            linkHeld_[id] = true;
            linkAnchor_[id] = ToNorm(kParams[src], value_[src]);
            anchorPending_[id] = false;
        }
        if (IsLinkSource(id)) Relink(id);
        return value_[id];
    }

    // Every main-loop pass once the controls have settled; shift = FS2 held (debounced). Returns a bitmask: bit s set
    // = slot s took its parameter back from a hold.
    uint16_t Update(const float knobs[6], const uint8_t toggles[3], bool shift) {
        for (int t = 0; t < 3; ++t) {
            if (lastToggle_[t] == toggles[t]) continue;
            const int id = ToggleParam(t);
            value_[id] = Clamp(kParams[id], float(toggles[t]));
            lastToggle_[t] = toggles[t];
        }
        active_ = btctl::ToAlgo(uint8_t(value_[kReverbAlgo]));

        // Resolve the slots: a slot that changes parameter is held when its assignment changed (a new assignment
        // keeps the value) or the parameter was written over MIDI while on no slot; a held parameter leaving its slot
        // (a Space on SW3) stays held for when it returns.
        uint8_t landed[kSlots];
        int nLanded = 0;
        for (int s = 0; s < kSlots; ++s) {
            const float av = value_[SlotAssignId(s)];
            const uint8_t a = uint8_t(std::lround(av));
            const uint8_t id = ResolveSlot(av, active_);
            const bool assignChanged = started_ && a != assign_[s];
            assign_[s] = a;
            if (id == slot_[s]) continue;
            if (slot_[s] != kNoParam && slotHeld_[s]) pendingHold_[slot_[s]] = true;
            slot_[s] = id;
            if (id == kNoParam) {
                slotHeld_[s] = false;
                continue;
            }
            slotHeld_[s] = assignChanged || pendingHold_[id];
            slotAnchor_[s] = knobs[s % kKnobs];
            landed[nLanded++] = id;
        }
        for (int i = 0; i < nLanded; ++i) pendingHold_[landed[i]] = false;
        started_ = true;

        if (shift && !shift_) {
            for (int k = 0; k < kKnobs; ++k) {
                shiftAnchor_[k] = knobs[k];
                shiftMoved_[k] = false;
                if (slot_[kKnobs + k] == kNoParam) continue;
                HoldSlot(kKnobs + k, knobs);
            }
        } else if (!shift && shift_) {
            for (int k = 0; k < kKnobs; ++k) {
                if (!shiftMoved_[k] || slot_[k] == kNoParam) continue;
                HoldSlot(k, knobs);
            }
        }
        shift_ = shift;

        uint16_t taken = 0;
        for (int k = 0; k < kKnobs; ++k) {
            if (shift) {
                if (!shiftMoved_[k] && std::fabs(knobs[k] - shiftAnchor_[k]) > kPickup) shiftMoved_[k] = true;
                DriveSlot(kKnobs + k, k, knobs, taken);
            } else {
                DriveSlot(k, k, knobs, taken);
            }
        }
        return taken;
    }

    // FS1 press.
    void ToggleBypass() { value_[kBypass] = value_[kBypass] > 0.5f ? 0.0f : 1.0f; }

private:
    // Slot s (knob k) writes its parameter from the knob unless held; a write holds every other slot on the same
    // parameter that is not held already (re-anchoring a held one would let the writing knob's jitter keep it held).
    void DriveSlot(int s, int k, const float knobs[6], uint16_t& taken) {
        const uint8_t id = slot_[s];
        if (id == kNoParam) return;
        if (slotHeld_[s]) {
            if (std::fabs(knobs[k] - slotAnchor_[s]) <= kPickup) return;
            slotHeld_[s] = false;
            taken |= uint16_t(1u << s);
        }
        const float v = FromNorm(kParams[id], knobs[k]);
        if (v == value_[id]) return;
        value_[id] = v;
        for (int t = 0; t < kSlots; ++t) {
            if (t == s || slot_[t] != id || slotHeld_[t]) continue;
            HoldSlot(t, knobs);
        }
        if (IsLinkSource(id)) Relink(id);
    }

    // Slot s held, anchored at its knob's position.
    void HoldSlot(int s, const float knobs[6]) {
        slotHeld_[s] = true;
        slotAnchor_[s] = knobs[s % kKnobs];
    }

    bool OnSlot(uint8_t id) const {
        for (uint8_t p : slot_)
            if (p == id) return true;
        return false;
    }

    // Writes source's link targets from its value, except those on a slot (a knob owns them) and those still held
    // against the source.
    void Relink(uint8_t source) {
        const float n = ToNorm(kParams[source], value_[source]);
        for (const Link& l : kLinks) {
            if (l.source != source || OnSlot(l.target)) continue;
            if (linkHeld_[l.target]) {
                if (anchorPending_[l.target]) {
                    linkAnchor_[l.target] = n;
                    anchorPending_[l.target] = false;
                    continue;
                }
                if (std::fabs(n - linkAnchor_[l.target]) <= kPickup) continue;
                linkHeld_[l.target] = false;
            }
            value_[l.target] = LinkValue(l, value_[source]);
        }
    }

    float value_[kParamCount];
    // Link holds (targets only, in source-norm terms); a pending anchor is set at the source's first change.
    bool linkHeld_[kParamCount];
    float linkAnchor_[kParamCount];
    bool anchorPending_[kParamCount];
    // A MIDI write to a parameter on no slot: it lands held when SW3 or the first Update puts it on a slot.
    bool pendingHold_[kParamCount];
    uint8_t slot_[kSlots];    // resolved parameter per slot as of the last Update (kNoParam = none)
    uint8_t assign_[kSlots];  // lround(assignment value) as of the last Update
    bool slotHeld_[kSlots];   // knob hold, anchored in knob terms
    float slotAnchor_[kSlots];
    bool shift_;                // FS2 as of the last Update
    float shiftAnchor_[kKnobs]; // knob positions at the FS2 press
    bool shiftMoved_[kKnobs];   // the knob moved more than kPickup while FS2 was held
    bool started_;
    uint8_t lastToggle_[3];
    btctl::Algo active_;
};

}  // namespace btparam
