include make/common.mk

.PHONY: flash_uart
flash_uart: 
	STM32_Programmer_CLI -c port=swd -d $(PROJECT_BINARY) -rst

.PHONY: flash_usb
flash_usb:
	STM32_Programmer_CLI -c port=$(USB_PORT) -d $(PROJECT_BINARY) -rst

.PHONY: monitor_uart
monitor_uart:
	minicom -D $(UART_PORT) -b $(UART_BAUD_RATE)

.PHONY: monitor_usb
monitor_usb:
	minicom -D $(USB_PORT) -b $(USB_BAUD_RATE)

.PHONY: debug
debug:
	@echo "Starting OpenOCD..."
	@openocd -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) & \
	OPENOCD_PID=$$!; \
	sleep 1; \
	echo "Starting GDB..."; \
	$(GDB) $(PROJECT_BINARY) -ex "target extended-remote :3333" -ex "load" -ex "monitor reset halt" -ex "continue"; \
	echo "Stopping OpenOCD..."; \
	kill $$OPENOCD_PID