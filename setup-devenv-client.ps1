# This PowerShell script sets up everything you need to build and run the client on Windows.

# This script has the following prerequisites:
# * You are running a64-bit windows operating system
# * You have installed Visual Studio 2015
# * You have installed CMake, and it is in your system PATH
# * You have installed Qt
# * You have installed git for windows, and it is in your system PATH
# * You have installed Conan for windows

# Use conan to install other deps (rabbitmq-c and protobuf)

cd client
conan install
cd ..


