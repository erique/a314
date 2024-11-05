#!/bin/bash

result=$( docker images -q a314 )
if [[ ! -n "$result" ]]; then
  docker build --platform=aarch64 -t a314 .
fi

docker run --platform=aarch64 --volume "$PWD":/host --workdir /host -it --rm a314 /bin/bash -c "make -C Software"
docker run --platform=aarch64 --volume "$PWD":/host --workdir /host -it --rm a314 /bin/bash -c "make -C Software -f Makefile-amiga"
