#include "camerafive.h"
#include "ui_camerafive.h"
#include <opencv/cv.h>
#include <opencv2/cvconfig.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <io.h>
#include <iostream>
#include "consumerthread.h"

typedef unsigned char byte;

CameraFive::CameraFive(QWidget *parent, QString login) :
    QDialog(parent),
    ui(new Ui::CameraFive)
{
    ui->setupUi(this);
    str_login = login;
}

CameraFive::~CameraFive()
{
    delete ui;
}

void CameraFive::handleFrame(QString key, QByteArray data) {
    QPixmap pix;
    pix.loadFromData((uchar*)data.data(), data.length(), "JPEG");
    ui->cam5lbl->setPixmap(pix);
}

void CameraFive::camFiveSubscription() {
    QString login = str_login;
    ConsumerThread *thread = new ConsumerThread(str_login, "camera.five");
    connect(thread, &ConsumerThread::receivedMessage, this, &CameraFive::handleFrame);
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

void CameraFive::camFiveStream() {
    CameraFive::camFiveSubscription();
}
