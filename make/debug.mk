include make/common.mk

OPENOCD := openocd
OPENOCD_GDB := arm-none-eabi-gdb
OPENOCD_INTERFACE ?=
OPENOCD_TARGET ?=

.PHONY: debug
debug: $(PROJECT_BINARY)
	@$(OPENOCD) -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" & \
	OPENOCD_PID=$$!; \
	trap "kill $$OPENOCD_PID" EXIT; \
	while ! nc -z localhost 3333; do sleep 0.1; done; \
	$(OPENOCD_GDB) "$<" \
		-ex "target extended-remote :3333" \
		-ex "load" \
		-ex "monitor reset halt" \
		-ex "continue"
