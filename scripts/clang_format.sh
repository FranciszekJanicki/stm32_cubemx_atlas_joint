#!/bin/bash

if [ "$#" -lt 1 ]; then 
    echo "Usage: $0 <format_dir> ..."
    exit 1 
fi

FORMAT_FILE_EXTS=("h" "hpp" "c" "cpp")

for FORMAT_DIR in "$@"; do
    for EXT in "${FORMAT_FILE_EXTS[@]}"; do
        find "$FORMAT_DIR" -type f -iname "*.$EXT" -print0 | xargs -0 -r clang-format -i
    done
done
