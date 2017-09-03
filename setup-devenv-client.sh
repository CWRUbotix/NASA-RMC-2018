#!/usr/bin/env bash

# This script sets up everything you need to build and run the client on Ubuntu

# Install C++ compiler, tools, and debugger
sudo apt-get -q -y install make cmake g++ gdb

# Install Qt5
sudo apt-get -q -y remove qt4-qmake
sudo apt-get -q -y install qt5-default qtdeclarative5-dev

# Install the protobuf compiler, protoc
sudo apt-get -q -y install protobuf-compiler

# Install protobuf library for C/C++
sudo apt-get -q -y install libprotobuf-dev

# Install Conan
sudo apt-get -q -y install python python-pip
sudo pip install conan

# Use conan to install other deps (rabbitmq-c)
cd client
conan install --build missing
cd ..
