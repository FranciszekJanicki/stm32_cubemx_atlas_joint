include make/common.mk

LINTERS_DIRS := $(MAIN_DIR) $(COMPONENTS_DIR) 
LINTERS_SCRIPT := $(SCRIPTS_DIR)/linters.sh

.PHONY: clang_tidy
clang_tidy:
	"$(LINTERS_SCRIPT)" tidy $(LINTERS_DIRS)

.PHONY: clang_format
clang_format:
	"$(LINTERS_SCRIPT)" format $(LINTERS_DIRS)

.PHONY: lint
lint: clang_tidy clang_format

