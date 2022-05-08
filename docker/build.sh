#!/bin/bash

image="pc_projection_to_image"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)