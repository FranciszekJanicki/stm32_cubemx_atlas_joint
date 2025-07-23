include make/common.mk

.PHONY: clang_tidy
clang_tidy:
	"$(SCRIPTS_DIR)/clang_tidy.sh"

.PHONY: clang_format
clang_format:
	"$(SCRIPTS_DIR)/clang_format.sh"

.PHONY: lint
lint: clang_tidy clang_format
