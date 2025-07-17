#!/bin/bash
set -e

function add_component {
    if [ "$#" -ne 2 ]; then
      echo "Usage: $0 <git_repo_url> <component_name>"
      exit 1
    fi

    REPO_URL=$1
    COMP_NAME=$2
    COMP_PATH="components/$COMP_NAME"

    if [ -d "$COMP_PATH" ]; then
      echo "Component $COMP_NAME already exists at $COMP_PATH"
      return 0
    fi

    echo "Adding component $COMP_NAME"
    git submodule add -f "$REPO_URL" "$COMP_PATH"
    echo "Done."
}

COMPONENTS_FILE="components.txt"

if [ ! -f "$COMPONENTS_FILE" ]; then
    echo "File $COMPONENTS_FILE not found"
    exit 1
fi

if [ ! -f ".gitmodules" ]; then
    echo "File .gitmodules not found, creating..."
    touch .gitmodules
fi

while read -r REPO NAME; do
    [[ -z "$REPO" || "$REPO" =~ ^# ]] && continue
    add_component "$REPO" "$NAME"
done < "$COMPONENTS_FILE"
