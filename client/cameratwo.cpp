#include "cameratwo.h"
#include "ui_cameratwo.h"
#include <opencv/cv.h>
#include <opencv2/cvconfig.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <io.h>
#include <iostream>
#include "consumerthread.h"

typedef unsigned char byte;

CameraTwo::CameraTwo(QWidget *parent, QString login) :
    QDialog(parent),
    ui(new Ui::CameraTwo)
{
    ui->setupUi(this);
    str_login = login;
}

CameraTwo::~CameraTwo()
{
    delete ui;
}

void CameraTwo::handleFrame(QString key, QByteArray data) {
    QPixmap pix;
    pix.loadFromData((uchar*)data.data(), data.length(), "JPEG");
    ui->cam2lbl->setPixmap(pix);
}

void CameraTwo::camTwoSubscription() {
    QString login = str_login;
    ConsumerThread *thread = new ConsumerThread(str_login, "camera.two");
    connect(thread, &ConsumerThread::receivedMessage, this, &CameraTwo::handleFrame);
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

void CameraTwo::camTwoStream() {
    CameraTwo::camTwoSubscription();
}
