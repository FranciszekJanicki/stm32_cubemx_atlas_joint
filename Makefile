include make/build.mk
include make/linters.mk
include make/flash.mk
include make/debug.mk
include make/monitor.mk
include make/cubemx.mk
include make/submodules.mk
include make/scripts.mk

.DEFAULT_GOAL := build

.PHONY: setup
setup:
	$(MAKE) setup_scripts
	$(MAKE) setup_submodules
	$(MAKE) setup_cmake

.PHONY: all
all:
	$(MAKE) build
	$(MAKE) flash
	@sleep 1 
	$(MAKE) monitor
