#include "camerafour.h"
#include "ui_camerafour.h"
#include <opencv/cv.h>
#include <opencv2/cvconfig.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <io.h>
#include <iostream>
#include "consumerthread.h"

typedef unsigned char byte;

CameraFour::CameraFour(QWidget *parent, QString login) :
    QDialog(parent),
    ui(new Ui::CameraFour)
{
    ui->setupUi(this);
    str_login = login;
}

CameraFour::~CameraFour()
{
    delete ui;
}

void CameraFour::handleFrame(QString key, QByteArray data) {
    QPixmap pix;
    pix.loadFromData((uchar*)data.data(), data.length(), "JPEG");
    ui->cam4lbl->setPixmap(pix);
}

void CameraFour::camFourSubscription() {
    QString login = str_login;
    ConsumerThread *thread = new ConsumerThread(str_login, "camera.four");
    connect(thread, &ConsumerThread::receivedMessage, this, &CameraFour::handleFrame);
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

void CameraFour::camFourStream() {
    CameraFour::camFourSubscription();
}
