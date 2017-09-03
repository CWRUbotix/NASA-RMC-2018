#include "consumerthread.h"
#include <AMQPcpp.h>
#include <QDebug>
#include "messages.pb.h"
#include <QString>
#include <QMap>

using namespace com::cwrubotix::glennifer;

ConsumerThread::ConsumerThread() {
    m_amqp = 0;
    m_topic = "";
}

ConsumerThread::ConsumerThread(QString loginStr, QString topic) {
    m_amqp = new AMQP(loginStr.toStdString());
    m_topic = topic;
}

ConsumerThread::~ConsumerThread() {
    if (m_amqp) {
        delete m_amqp;
    }
    if (consumerThreadInstances.contains(m_topic)) {
        consumerThreadInstances.remove(m_topic);
    }
}

bool ConsumerThread::isValid() const {
    return m_amqp != 0;
}

void ConsumerThread::run() {
    if (consumerThreadInstances.contains(m_topic)) {
        qDebug() << "ConsumerThread created with duplicate topic";
    } else {
        consumerThreadInstances.insert(m_topic, this);
        AMQPQueue *queue = m_amqp->createQueue(m_topic.toStdString());
        queue->Declare();
        queue->Bind("amq.topic", m_topic.toStdString());
        queue->addEvent(AMQP_MESSAGE, handleReceivedMessage);
        queue->Consume(AMQP_NOACK);
    }
}

int handleReceivedMessage(AMQPMessage *message) {
    QString topic = QString::fromStdString(message->getQueue()->getName());
    if (!consumerThreadInstances.contains(topic)) {
        qDebug() << "Received message with unknown topic";
        return 1;
    }
    ConsumerThread *foundConsumerThread = consumerThreadInstances.value(topic);
    if (foundConsumerThread == 0) {
        qDebug() << "Associated consumer thread is null";
        return 1;
    }
    if (!foundConsumerThread->isValid()) {
        qDebug() << "Associated consumer thread is invalid";
        return 1;
    }
    QString key = QString::fromStdString(message->getRoutingKey());
    uint32_t len = 0;
    char *data = message->getMessage(&len);
    if (!data) {
        qDebug() << "No data";
        return 1;
    }
    QByteArray dataArray = QByteArray::fromRawData(data, len);
    emit foundConsumerThread->receivedMessage(key, dataArray);
    return 0;
}

QMap<QString, ConsumerThread*> consumerThreadInstances;
