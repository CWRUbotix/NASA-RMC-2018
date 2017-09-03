#!/usr/bin/env bash

# This script sets up everything you need to build and run Glennifer on Ubuntu

apt-get -q update

# Install C++ compiler and tools
apt-get -q -y install autoconf automake make g++ 

# Install python 3 and tools
apt-get -q -y install python3 python3-pip

# Install the protobuf compiler, protoc
apt-get -q -y install protobuf-compiler

# Install the protobuf library for python
pip3 install protobuf

# Install pika, the AMQP library for python
pip3 install pika

# Install the Java 8 Development Kit and Maven
apt-get -q -y install software-properties-common
add-apt-repository -y ppa:openjdk-r/ppa
apt-get update
apt-get -q -y install openjdk-8-jdk maven

# Install and set up the RabbitMQ server
apt-get -q -y install wget
echo 'deb http://www.rabbitmq.com/debian/ testing main' |
        tee /etc/apt/sources.list.d/rabbitmq.list
wget -O- https://www.rabbitmq.com/rabbitmq-release-signing-key.asc |
        apt-key add -
apt-get update
apt-get -q -y install rabbitmq-server
invoke-rc.d rabbitmq-server start

# Rabbitmq user setup
cp rabbitmq.config /etc/rabbitmq/

# Let the cwrubotix user use serial ports
usermod -a -G dialout cwrubotix

# Run on boot
cp rc.local /etc/rc.local

# SSH server for debugging
apt-get -q -y install openssh-server
