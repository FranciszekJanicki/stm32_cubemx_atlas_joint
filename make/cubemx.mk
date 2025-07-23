include make/common.mk

CUBEMX_DIR := $(PROJECT_DIR)/cubemx
CUBEMX_BINARY := stm32cubemx
CUBEPROG_BINARY := /opt/stm32cubeprog/bin/STM32CubeProgrammer

.PHONY: cubemx
cubemx: 
	$(CUBEMX_BINARY)

.PHONY: cubeprog
cubeprog:
	$(CUBEPROG_BINARY)