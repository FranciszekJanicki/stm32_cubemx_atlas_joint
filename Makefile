include make/build.mk
include make/linters.mk
include make/cubemx.mk
include make/flash.mk
include make/components.mk
include make/third_party.mk

.PHONY: setup
setup:
	$(MAKE) setup_components
	$(MAKE) setup_third_party
	$(MAKE) setup_cmake

.PHONY: all
all:
	$(MAKE) lint
	$(MAKE) build
	$(MAKE) flash_uart
	$(MAKE) monitor_uart
