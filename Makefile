# Host builds, no libDaisy: `make host` -> build/host/tests and build/host/render; `make test` runs the tests and
# checks docs/MIDI.md is current; `make midi-doc` regenerates docs/MIDI.md from daisy/src/params.h.
HOST_DIR := build/host
HOST_CXXFLAGS := -std=gnu++17 -O2 -g -Wall -Wextra -Werror -Idaisy/src -Ihost
GTEST_CFLAGS := $(shell pkg-config --cflags gtest_main)
GTEST_LIBS := $(shell pkg-config --libs gtest_main)
TEST_SOURCES := $(wildcard tests/*.cpp)
DSP_HEADERS := $(wildcard daisy/src/*.h daisy/src/dsp/*.h tests/*.h)

host: $(HOST_DIR)/tests $(HOST_DIR)/render

$(HOST_DIR)/tests: $(TEST_SOURCES) $(DSP_HEADERS) | $(HOST_DIR)
	$(CXX) $(HOST_CXXFLAGS) $(GTEST_CFLAGS) $(TEST_SOURCES) -o $@ $(GTEST_LIBS)

$(HOST_DIR)/render: host/render.cpp host/wav.h $(DSP_HEADERS) | $(HOST_DIR)
	$(CXX) $(HOST_CXXFLAGS) host/render.cpp -o $@

$(HOST_DIR)/gen_midi_doc: host/gen_midi_doc.cpp $(DSP_HEADERS) | $(HOST_DIR)
	$(CXX) $(HOST_CXXFLAGS) host/gen_midi_doc.cpp -o $@

$(HOST_DIR):
	mkdir -p $@

test: host check-midi-doc
	$(HOST_DIR)/tests

midi-doc: $(HOST_DIR)/gen_midi_doc
	$(HOST_DIR)/gen_midi_doc > docs/MIDI.md

# Fails when docs/MIDI.md is not what `make midi-doc` would write.
check-midi-doc: $(HOST_DIR)/gen_midi_doc
	$(HOST_DIR)/gen_midi_doc | diff -u docs/MIDI.md -

clean:
	rm -rf build

.PHONY: host test midi-doc check-midi-doc clean
