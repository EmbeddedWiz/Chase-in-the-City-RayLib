# Thin project Makefile. CMake is the canonical build system;
# this file only delegates configuration and compilation to it.
#
#   make            -> Release build
#   make debug      -> Debug build
#   make release    -> Release build
#   make test       -> build + run the headless logic tests (Release)
#   make clean      -> remove the CMake build tree
#
# Pass a prebuilt raylib directory if it is not auto-detected:
#   make release RAYLIB_ROOT=<dir>

BUILD     ?= build
RAYLIB    ?=

RAYLIB_FLAG = $(if $(RAYLIB_ROOT),-DRAYLIB_ROOT=$(RAYLIB_ROOT),)

.PHONY: all debug release test clean

all: release

configure:
	cmake -S . -B $(BUILD) $(RAYLIB_FLAG)

debug: configure
	cmake --build $(BUILD) --config Debug

release: configure
	cmake --build $(BUILD) --config Release

test:
	cmake -S . -B $(BUILD) -DCHASE_BUILD_TESTS=ON $(RAYLIB_FLAG)
	cmake --build $(BUILD) --config Release --target chase_tests
	ctest --test-dir $(BUILD) -C Release --output-on-failure

clean:
	rm -rf $(BUILD)
