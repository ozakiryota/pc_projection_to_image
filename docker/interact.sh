#!/bin/bash

xhost +

image="pc_projection_to_image"
tag="latest"

docker run -it --rm \
	-e "DISPLAY" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v $(pwd)/..:/root/catkin_ws/src/$image \
	-v $HOME/rosbag:/root/rosbag \
	$image:$tag