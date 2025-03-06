#!/usr/bin/env bash

set -eou pipefail

BASE_PATH=$(git rev-parse --show-toplevel)
ROS_PKG_NAME="adi_imu"

echo "Installing dependencies"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    graphviz \
    doxygen

echo "Generating the build directory"
pushd "$BASE_PATH" >/dev/null || exit
mkdir -p _build

echo "Building the docs using rosdoc2"
cd "$BASE_PATH/_build" || exit
rosdoc2 build --package-path "$BASE_PATH" --debug

popd >/dev/null

if [ $? -eq 0 ]; then
    echo "rosdoc2 build command executed successfully."
else
    echo "rosdoc2 build command failed."
    exit 1
fi

if [ ! -d "$BASE_PATH/_build/docs_output/$ROS_PKG_NAME" ]; then
    echo "Error: The docs_output/$ROS_PKG_NAME directory does not exist."
    exit 1
fi
