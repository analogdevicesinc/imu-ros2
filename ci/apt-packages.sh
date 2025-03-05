#!/usr/bin/env bash

apt_packages=(
    # libiio
    ## Basic system setup
    build-essential
    libxml2-dev
    libzstd-dev
    bison
    flex
    libcdk5-dev
    cmake
    ## Backend deps
    libaio-dev
    libusb-1.0-0-dev
    libserialport-dev
    libavahi-client-dev
)
