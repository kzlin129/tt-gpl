#!/usr/bin/env bash

for i in `ls -d */`; do
  pushd $i
  wget -i ./list.txt
  popd
done
