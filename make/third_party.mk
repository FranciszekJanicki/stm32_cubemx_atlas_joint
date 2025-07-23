include make/common.mk

THIRD_PARTY_DIR := third_party
THIRD_PARTY_FILE := requirements/third_party.txt

.PHONY: setup_third_party
setup_third_party:
	"$(SCRIPTS_DIR)/submodules.sh" "$(THIRD_PARTY_FILE)" "$(THIRD_PARTY_DIR)" add

.PHONY: remove_third_party
remove_third_party:
	"$(SCRIPTS_DIR)/submodules.sh" "$(THIRD_PARTY_FILE)" "$(THIRD_PARTY_DIR)" remove

.PHONY: update_third_party
update_third_party:
	"$(SCRIPTS_DIR)/submodules.sh" "$(THIRD_PARTY_FILE)" "$(THIRD_PARTY_DIR)" update
