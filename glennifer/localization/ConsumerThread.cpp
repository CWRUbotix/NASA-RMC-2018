/*
 * ConsumerThread.cpp
 *
 *  Created on: May 14, 2018
 *      Author: mesozoicprime
 */

#include "ConsumerThread.h"
#include <string>
#include <amqpcpp/AMQPcpp.h>

using namespace std;

struct signal {
	//bool
};

/*int handleReceivedMessage(AMQPMessage *message) {
	string topic = message->getQueue()->getName();
	//TODO: add checkforinstances
	string key = message->getRoutingKey();
	uint32_t len = 0;
	char *data = message->getMessage(&len);
	return 0;
}

ConsumerThread::ConsumerThread() {
	m_amqp = 0;
	m_topic = "";
}

ConsumerThread::ConsumerThread(string loginstr, string topic) {
	m_amqp = new AMQP(loginstr);
	m_topic = topic;
}

ConsumerThread::~ConsumerThread() {
	if(m_amqp) {
		delete m_amqp;
	}
	//TODO: add checkforinstances
}*/

/*bool ConsumerThread::isValid() {
	return m_amqp != 0;
}*/
/*
void ConsumerThread::run() {
	//add checkforinstances
	AMQPQueue *queue = m_amqp->createQueue(m_topic);
	queue->Declare();
	queue->Bind("amq.topic", m_topic);
	queue->addEvent(AMQP_MESSAGE, handleReceivedMessage);
	//qu2->addEvent(AMQP_CANCEL, onCancel );
	queue->Consume(AMQP_NOACK);
}*/

//TODO: add map for instances of consumer thread like in the client code
