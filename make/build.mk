include make/common.mk

.PHONY: build
build: 
	$(MAKE) -C "$(BUILD_DIR)"

.PHONY: clean
clean: 
	rm -rf "$(BUILD_DIR)"

.PHONY: setup_cmake
setup_cmake:
	$(MAKE) clean
	mkdir -p "$(BUILD_DIR)"
	cmake -S . -B "$(BUILD_DIR)"
