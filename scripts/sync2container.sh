#!/bin/bash
set -e

# copy sources
docker cp ./src client:/opt/ep_ws
# build
docker exec -w /opt/ep_ws client ./build.sh
