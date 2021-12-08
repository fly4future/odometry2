#!/bin/bash

set -eu

## for installing additional non-ROS2 dependencies to debian package generated by bloom
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        echo "[INFO] Initialize rosdep"
        sudo rosdep init
fi

mod_dir=$(echo ${1} | sed 's/\/$//')
yamlpath=${mod_dir}/packaging/rosdep.yaml

echo "[INFO] Add module specific dependencies"
cat $yamlpath
mkdir -p /etc/ros/rosdep/sources.list.d
echo "yaml file://${yamlpath}" > /etc/ros/rosdep/sources.list.d/51-fogsw-module.list

if [ "$(id -u)" != "0" ]; then
        echo "[INFO] Updating rosdep"
        rosdep update
fi

exit 0
