/*
 * ConsumerThread.h
 *
 *  Created on: May 14, 2018
 *      Author: mesozoicprime
 */


#ifndef CONSUMERTHREAD_H_
#define CONSUMERTHREAD_H_

using namespace std;
#include<string>
#include<iostream>
#include<thread>
#include <amqpcpp/AMQPcpp.h>
#include "messages.pb.h"


class ConsumerThread  {

public:
	ConsumerThread();
	ConsumerThread(string loginStr, string topic);
	~ConsumerThread();
	//bool isValid() const;

protected:
	void run();

private:
	AMQP *m_amqp;
	string m_topic;
};

#endif /* CONSUMERTHREAD_H_ */
