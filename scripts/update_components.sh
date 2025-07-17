#!/bin/bash
set -e

function update_component {
    if [ "$#" -ne 1 ]; then
      echo "Usage: update_component <component_name>"
      exit 1
    fi

    COMP_NAME=$1
    COMP_PATH="components/$COMP_NAME"

    if [ ! -d "$COMP_PATH" ]; then
      echo "Component $COMP_NAME not found at $COMP_PATH, skipping."
      return 0
    fi

    echo "Updating component: $COMP_NAME"
    git submodule update --remote "$COMP_PATH"
    echo "Done."
}

COMPONENTS_FILE="components.txt"

if [ ! -f "$COMPONENTS_FILE" ]; then
    echo "File $COMPONENTS_FILE not found"
    exit 1
fi

git submodule update --init --recursive

while read -r REPO NAME; do
    [[ -z "$REPO" || "$REPO" =~ ^# ]] && continue
    update_component "$NAME"
done < "$COMPONENTS_FILE"
