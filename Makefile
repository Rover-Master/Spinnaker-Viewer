NAME := SpinnakerCamera
# This is a heper Makefile for CMake Project
DEPS := $(wildcard **/*.cpp)
DEPS += $(wildcard **/*.h)
DEPS += $(wildcard **/*.hpp)
# Make without output
MAKE := make --no-print-directory
# CMake Build Directory and Build Type
CMAKE_BUILD_TYPE ?= Unknown

release: CMAKE_BUILD_TYPE := Release
debug: CMAKE_BUILD_TYPE := Debug

release debug:
	$(eval BUILD_DIR := .build/$@/)
	@ $(MAKE) $(BUILD_DIR)/Makefile CMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
	  && ln -sf $(BUILD_DIR)/compile_commands.json . \
	  && $(MAKE) -C $(BUILD_DIR) -j$(shell nproc --all) \
	  && ln -sf $(BUILD_DIR)/$(NAME) $(NAME)

.build/%/Makefile: CMakeLists.txt
	$(eval BUILD_DIR := $(shell dirname $@))
	$(info Generating CMake Files For $*)
	@ mkdir -p $(BUILD_DIR) \
	  && cd $(BUILD_DIR) \
	  && cmake -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) $(PWD)

clean:
	@ rm -rf .build

include $(wildcard scripts/*.mk)

.PHONY: release debug init clean
