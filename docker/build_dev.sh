#!/usr/bin/env bash

set -e

REPO=daohu527/carla
TAG="${REPO}:bridge-x86-18.04"

docker build -t "${TAG}" -f bridge.x86.dockerfile .

# push docker to hub.docker.com
# docker tag apollo:bridge-x86-18.04 daohu527/carla:bridge-x86-18.04
# docker push daohu527/carla:bridge-x86-18.04
