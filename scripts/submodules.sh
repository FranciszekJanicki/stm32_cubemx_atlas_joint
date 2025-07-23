#!/bin/bash
set -e

if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <submodules_file> <submodules_dir> <submodules_action>"
fi

SUBMODULES_FILE=$1
SUBMODULES_DIR=$2
SUBMODULES_ACTION=$3

if [ ! -f "$SUBMODULES_FILE" ]; then
    echo "File $SUBMODULES_FILE not found"
    exit 1
fi

if [ ! -d "$SUBMODULES_DIR" ]; then
    echo "Directory $SUBMODULES_DIR not found"
    exit 1
fi

if [[ "$SUBMODULES_ACTION" != "add" && 
      "$SUBMODULES_ACTION" != "remove" && 
      "$SUBMODULES_ACTION" != "update" ]]; then
    echo "Invalid action: $SUBMODULES_ACTION. Valid actions are 'add', 'remove', 'update'."
    exit 1
fi

while read -r SUBMODULE_URL SUBMODULE_NAME; do
    [[ -z "SUBMODULE_NAME" || "$SUBMODULE_NAME" =~ ^# ]] && continue
    source scripts/submodule.sh "$SUBMODULE_URL" "$SUBMODULES_DIR" "$SUBMODULE_NAME" "$SUBMODULES_ACTION"
done < "$SUBMODULES_FILE"