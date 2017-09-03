#ifndef CONSUMERTHREAD_H
#define CONSUMERTHREAD_H

#include <QThread>
#include <AMQPcpp.h>
#include "messages.pb.h"
#include <QString>
#include <QMap>
#include <QByteArray>

using namespace com::cwrubotix::glennifer;

class ConsumerThread : public QThread
{
    Q_OBJECT
public:
    ConsumerThread();
    ConsumerThread(QString loginStr, QString topic);
    ~ConsumerThread();
    bool isValid() const;
protected:
    void run();
signals:
    void receivedMessage(QString key, QByteArray data);
private:
    AMQP *m_amqp;
    QString m_topic;
};

extern QMap<QString, ConsumerThread*> consumerThreadInstances;

int handleReceivedMessage(AMQPMessage *message);

#endif // CONSUMERTHREAD_H
