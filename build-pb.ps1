# This script builds all the protobuf classes for java, cpp, and python on windows

New-Item -Force -ItemType directory -Path pb

.\client\bin\protoc.exe --cpp_out=pb/. --java_out=pb/. --python_out=pb/. `
  --python_out=glennifer/. `
  --python_out=glennifer/motor_dispatch/src/python/. `
  --cpp_out=client/. `
  --java_out=glennifer/robot_state/src/main/java/. `
  --java_out=glennifer/HCI/src/main/java/. `
  --java_out=glennifer/autodrill/src/main/java/. `
  messages.proto
