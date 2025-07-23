#!/bin/bash
set -e

if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <submodule_url> <submodule_dir> <submodule_name> <submodule_action>"
    exit 1
fi

SUBMODULE_URL="$1"
SUBMODULE_DIR="$2"
SUBMODULE_NAME="$3"
SUBMODULE_ACTION="$4"
SUBMODULE_PATH="$SUBMODULE_DIR/$SUBMODULE_NAME"

function add_submodule {
    if [ -d "$SUBMODULE_PATH" ]; then
        echo "Submodule $SUBMODULE_NAME already exists at $SUBMODULE_PATH" 
        return
    fi

    echo "Adding submodule $SUBMODULE_NAME → $SUBMODULE_PATH"
    git submodule add -f "$SUBMODULE_URL" "$SUBMODULE_PATH"
}

function remove_submodule {
    if [ ! -d "$SUBMODULE_PATH" ]; then
        echo "Submodule $SUBMODULE_NAME doesnt exist at $SUBMODULE_PATH"        
        return
    fi

    echo "Removing submodule $SUBMODULE_NAME"
    git submodule deinit -f "$SUBMODULE_PATH"
    git rm -rf "$SUBMODULE_PATH"
    rm -rf ".git/modules/$SUBMODULE_PATH"
    rm -rf "$SUBMODULE_PATH"
}

function update_submodule {
    if [ ! -d "$SUBMODULE_PATH" ]; then
        echo "Submodule $SUBMODULE_NAME doesnt exist at $SUBMODULE_PATH"
        return
    fi

    echo "Updating submodule $SUBMODULE_NAME"
    git submodule update --remote "$SUBMODULE_PATH"
}

case "$SUBMODULE_ACTION" in
    add) add_submodule ;;
    remove) remove_submodule ;;
    update) update_submodule ;;
    *) echo "Invalid action: $SUBMODULE_ACTION"; exit 1 ;;
esac
