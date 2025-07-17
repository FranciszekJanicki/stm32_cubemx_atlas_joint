include make/common.mk

.PHONY: add_components
add_components:
	$(SCRIPTS_DIR)/add_components.sh

.PHONY: remove_components
remove_components:
	$(SCRIPTS_DIR)/remove_components.sh

.PHONY: update_components
update_components:
	$(SCRIPTS_DIR)/update_components.sh

.PHONY: setup_components
setup_components:
	chmod +x scripts/*
	$(MAKE) add_components