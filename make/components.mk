include make/common.mk

COMPONENTS_DIR := components
COMPONENTS_FILE := requirements/components.txt

.PHONY: setup_components
setup_components:
	"$(SCRIPTS_DIR)/submodules.sh" "$(COMPONENTS_FILE)" "$(COMPONENTS_DIR)" add

.PHONY: remove_components
remove_components:
	"$(SCRIPTS_DIR)/submodules.sh" "$(COMPONENTS_FILE)" "$(COMPONENTS_DIR)" remove

.PHONY: update_components
update_components:
	"$(SCRIPTS_DIR)/submodules.sh" "$(COMPONENTS_FILE)" "$(COMPONENTS_DIR)" update
