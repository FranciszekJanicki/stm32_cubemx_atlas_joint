PROJECT_DIR := $(shell pwd)
MAIN_DIR := $(PROJECT_DIR)/main
COMPONENTS_DIR := $(PROJECT_DIR)/components
THIRD_PARTY_DIR := $(PROJECT_DIR)/third_party
STM32_CUBEMX_DIR := $(PROJECT_DIR)/stm32_cubemx
BUILD_DIR := $(PROJECT_DIR)/build
SCRIPTS_DIR := $(PROJECT_DIR)/scripts

PROJECT_NAME := project
PROJECT_BINARY := $(BUILD_DIR)/main/main.elf

UART_BAUD_RATE := 115200
UART_PORT := /dev/ttyACM0
USB_BAUD_RATE := 115200
USB_PORT := /dev/ttyACM0

OPENOCD=openocd
GDB=arm-none-eabi-gdb
OPENOCD_INTERFACE=interface/stlink.cfg
OPENOCD_TARGET=target/stm32l4x.cfg