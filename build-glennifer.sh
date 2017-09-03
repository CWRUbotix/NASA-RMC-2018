#!/usr/bin/env bash

# This script builds Glennifer. The resulting binaries are stored in ./build-glennifer
# To start Glennifer, execute ./build-glennifer/start

# Currently all this does is generate the protobuf classes

./build-pb.sh

cd glennifer/robot_state
mvn install
cd ../..