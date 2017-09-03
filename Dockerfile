FROM ubuntu:16.04
RUN apt-get -q update && \
	apt-get -q -y install autoconf automake make g++ python3 python3-pip protobuf-compiler software-properties-common && \
	pip3 install protobuf pika && \
	add-apt-repository ppa:openjdk-r/ppa && \
	apt-get -q update && \
	apt-get -q -y install openjdk-8-jdk maven wget && \
	echo 'deb http://www.rabbitmq.com/debian/ testing main' | tee /etc/apt/sources.list.d/rabbitmq.list && \
	wget -O- https://www.rabbitmq.com/rabbitmq-release-signing-key.asc | apt-key add - && \
	apt-get -q update && \
	apt-get -q -y install rabbitmq-server