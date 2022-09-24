#!/usr/bin/env bash

set -e

REPO=daohu527/carla
TAG="${REPO}:bridge-x86-18.04"

docker pull "${TAG}"

docker run -it -v $(pwd):/home/carla_bridge --net=host "${TAG}" /bin/bash
