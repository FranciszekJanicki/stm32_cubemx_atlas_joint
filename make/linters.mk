include make/common.mk

.PHONY: clang_tidy
clang_tidy:
	for ext in h c cpp hpp; do \
		find $(COMPONENTS_DIR) -iname "*.$$ext" -print0 | xargs -0 -r clang-tidy -fix; \
	done

.PHONY: clang_format
clang_format:
	for ext in h c cpp hpp; do \
		find $(COMPONENTS_DIR) -iname "*.$$ext" -print0 | xargs -0 -r clang-format -i; \
	done

.PHONY: lint
lint: clang_tidy clang_format
