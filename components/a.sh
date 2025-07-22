#!/usr/bin/bash

dirs=$(find -maxdepth 1)
for dir in $dirs; do
    cd $dir
    git add .
    git commit -m"changes"
    git push
    cd ..
done