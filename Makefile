# =============================================================================
# TODO: Add file description
# =============================================================================
# License: MIT
# Author: Yuxuan Zhang (zhangyuxuan@ufl.edu)
# =============================================================================


NAME := SpinnakerCamera
# This is a helper Makefile for CMake Project
DEPS := $(wildcard **/*.cpp)
DEPS += $(wildcard **/*.h)
DEPS += $(wildcard **/*.hpp)
# Make without output
MAKE := make --no-print-directory

examples:
	$(eval BUILD_DIR := build/$@/)
	@ cmake -S examples -B $(BUILD_DIR) \
	  && mv $(BUILD_DIR)/compile_commands.json . \
	  && $(MAKE) -C $(BUILD_DIR) -j$(shell nproc --all)

clean:
	@ rm -rf build bin

include $(wildcard scripts/*.mk)

.PHONY: examples clean
