# bad_tape: the Bad Tape delay/reverb pedal on the Daisy Seed in a Cleveland Music Co. HotHouse (src/board.h).
# APP=bad_tape (default): the pedal firmware, build/<BOARD>/bad_tape.bin.
# APP=bench: the DSP benchmark (src/bench.cpp, no audio), build/<BOARD>-bench/bad_tape_bench.bin.
# APP=latency: the audio round trip through a left output -> left input cable (src/latency.cpp),
# build/<BOARD>-latency/bad_tape_latency.bin.

# The app runs from SRAM, loaded from QSPI by the Daisy bootloader.
export APP_TYPE = BOOT_SRAM

# BOARD: hothouse (default, Cleveland Music Co. HotHouse). Other hosts are added here and in src/board.h.
BOARD ?= hothouse
ifeq ($(BOARD),hothouse)
BOARD_DEF = -DBADTAPE_BOARD_HOTHOUSE=1
else
$(error BOARD must be hothouse, not '$(BOARD)')
endif

# Source lists must be set before including the core Makefile: it evaluates vpath directives at parse time.
# One build directory per board and app (the objects differ); libDaisy's mkdir has no -p.
APP ?= bad_tape
ifeq ($(APP),bad_tape)
TARGET = bad_tape
CPP_SOURCES = src/bad_tape.cpp src/host_session.cpp src/usb_link.cpp src/memory.cpp
override BUILD_DIR = build/$(BOARD)
else ifeq ($(APP),bench)
TARGET = bad_tape_bench
CPP_SOURCES = src/bench.cpp src/usb_link.cpp src/memory.cpp
override BUILD_DIR = build/$(BOARD)-bench
else ifeq ($(APP),latency)
TARGET = bad_tape_latency
CPP_SOURCES = src/latency.cpp src/usb_link.cpp
override BUILD_DIR = build/$(BOARD)-latency
else
$(error APP must be bad_tape, bench or latency, not '$(APP)')
endif
$(shell mkdir -p $(BUILD_DIR))

# libDaisy is the daisy/libDaisy submodule (v9.0.0).
LIBDAISY_DIR ?= libDaisy
# Stop with the fix if the libDaisy submodule, its nested driver submodules, or its library is missing.
ifeq ($(wildcard $(LIBDAISY_DIR)/core/Makefile),)
$(error libDaisy not found at $(LIBDAISY_DIR). From the repo root run: git submodule update --init --recursive && make -C daisy/libDaisy)
endif
ifeq ($(wildcard $(LIBDAISY_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h),)
$(error libDaisy's driver submodules are missing under $(LIBDAISY_DIR). From the repo root run: git submodule update --init --recursive && make -C daisy/libDaisy)
endif
ifeq ($(wildcard $(LIBDAISY_DIR)/build/libdaisy.a),)
$(error libdaisy.a not built. From the repo root run: make -C daisy/libDaisy)
endif

OPT = -O3
CPP_STANDARD = -std=gnu++17
# libDaisy's BOOT_SRAM script with .data/.bss in AXI SRAM, plus .dtcm_data/.itcm_text (copied at boot, src/memory.cpp)
# and .d2_bss (the reverb pool).
LDSCRIPT = STM32H750IB_sram_badtape.lds

# The build id the Identity Reply carries (src/link.h): the repo's short commit, with +dirty when tracked
# files differ, or unknown outside git. $(BUILD_DIR)/build_id.c is rewritten only when the id changes, so an
# unchanged tree relinks nothing.
BUILD_ID_HEAD := $(shell git -C $(CURDIR)/.. rev-parse --short HEAD 2>/dev/null)
BUILD_ID := $(if $(BUILD_ID_HEAD),$(BUILD_ID_HEAD)$(if $(shell git -C $(CURDIR)/.. status --porcelain --untracked-files=no 2>/dev/null),+dirty),unknown)
BUILD_ID_C := $(BUILD_DIR)/build_id.c
BUILD_ID_SRC := const char kBuildId[] = "$(BUILD_ID)";
ifneq ($(BUILD_ID_SRC),$(shell cat $(BUILD_ID_C) 2>/dev/null))
$(shell printf '%s\n' '$(BUILD_ID_SRC)' > $(BUILD_ID_C))
endif
C_SOURCES = $(BUILD_ID_C)

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

C_DEFS += $(BOARD_DEF)

# Relink when the linker script changes: the core link rule lists only $(OBJECTS) and Makefile as prerequisites.
$(BUILD_DIR)/$(TARGET).elf: $(LDSCRIPT)
