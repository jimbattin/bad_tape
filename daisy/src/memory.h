// Memory placement (STM32H750IB_sram_badtape.lds; the map is in daisy/README.md "Memory map").
//   BT_ITCM: a function in ITCM (zero-wait instruction fetch). An inlined copy runs from its caller's section, so every
//   caller of a BT_ITCM function is itself BT_ITCM or cold (Init, Snap, SetParam: never per sample). Each use gets its
//   own input section (.itcm_text.N): GCC rejects a named section that holds both inline (COMDAT) functions, such as
//   the DSP's header functions, and ordinary ones, such as AudioCallback, in one translation unit. GCC 13 drops the
//   section attribute from template instantiations (function templates and class-template members): a BT_ITCM
//   template runs from ITCM only when inlined into its caller, so the hot ones are [[gnu::always_inline]] (SvfT,
//   DbToGain, StepSmoothers, AlgoParams::Tick, ReverseGrains::Process); `arm-none-eabi-nm` on the image shows what
//   landed where.
//   BT_DTCM: a variable in DTCM (zero-wait data, not cached). Any initialiser works.
//   BT_SDRAM: a variable in the external SDRAM (.sdram_bss). NOLOAD: startup neither initialises nor zeroes it, so
//   its owner clears it (the delay line: btdsp::Delay::Init). No initialiser.
//   BT_D2: a variable in D2 SRAM (.d2_bss), NOLOAD, cleared by its owner (btdsp::Engine::Init, and in slices on every
//   SW3 switch).
// ITCM and DTCM are linked to run in the TCMs and loaded in SRAM; memory.cpp copies them from their SRAM load image
// in .preinit_array, after startup's .data/.bss init and before any C++ constructor or main(). Calls between ITCM
// (0x00000000) and SRAM (0x24000000) are out of BL range; GNU ld inserts long-branch veneers for them.
// On the host (tests, render) the macros are empty.
#pragma once

#if defined(__arm__)
#define BT_ITCM_STR(x) #x
#define BT_ITCM_SECTION(n) ".itcm_text." BT_ITCM_STR(n)
#define BT_ITCM __attribute__((section(BT_ITCM_SECTION(__COUNTER__))))
#define BT_DTCM __attribute__((section(".dtcm_data")))
#define BT_SDRAM __attribute__((section(".sdram_bss")))
#define BT_D2 __attribute__((section(".d2_bss")))
#else
#define BT_ITCM
#define BT_DTCM
#define BT_SDRAM
#define BT_D2
#endif
