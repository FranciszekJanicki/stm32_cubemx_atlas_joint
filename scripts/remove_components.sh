#!/bin/bash
set -e

function remove_component {
    if [ "$#" -ne 1 ]; then
      echo "Usage: $0 <component_name>"
      exit 1
    fi

    COMP_NAME=$1
    COMP_PATH="components/$COMP_NAME"

    if [ ! -d "$COMP_PATH" ]; then
      echo "Component $COMP_NAME does not exist at $COMP_PATH"
      return 0
    fi

    echo "Removing component $COMP_NAME"
    git submodule deinit -f "$COMP_PATH"
    git rm -rf "$COMP_PATH"
    rm -rf ".git/modules/$COMP_PATH"
    rm -rf "$COMP_PATH"
    echo "Done."
}

COMPONENTS_FILE="components.txt"

if [ ! -f "$COMPONENTS_FILE" ]; then
    echo "File $COMPONENTS_FILE not found"
    exit 1
fi

while read -r REPO NAME; do
    [[ -z "$NAME" || "$NAME" =~ ^# ]] && continue
    remove_component "$NAME"
done < "$COMPONENTS_FILE"

rm -rf .gitmodules
