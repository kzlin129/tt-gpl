#!/usr/bin/env bash

for i in `ls`; do
  pushd $i
  wget -i ./list.txt
  popd
done
