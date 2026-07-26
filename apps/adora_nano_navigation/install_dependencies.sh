#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  libeigen3-dev \
  libceres-dev \
  libsuitesparse-dev \
  libboost-serialization-dev \
  libboost-thread-dev \
  libtbb-dev \
  nlohmann-json3-dev \
  libssl-dev \
  python3-yaml
