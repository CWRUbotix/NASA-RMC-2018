#!/usr/bin/env bash

# This script builds all the protobuf classes for java, cpp, and python

mkdir -p pb
protoc --cpp_out=pb/. --java_out=pb/. --python_out=pb/. \
  --python_out=glennifer/. \
  --python_out=glennifer/motor_dispatch/src/python/. \
  --python_out=glennifer/obstacle_detection/examples/. \
  --cpp_out=client/. \
  --cpp_out=glennifer/localization/. \
  --java_out=glennifer/robot_state/src/main/java/. \
  --java_out=glennifer/HCI/src/main/java/. \
  --java_out=glennifer/automodule/src/main/java/. \
  messages.proto
