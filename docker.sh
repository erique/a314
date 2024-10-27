#!/bin/bash

result=$( docker images -q a314 )
if [[ ! -n "$result" ]]; then
  docker build -t a314 .
fi

docker run --volume "$PWD":/host --workdir /host -it --rm a314 /bin/bash -c "make -C Software"
