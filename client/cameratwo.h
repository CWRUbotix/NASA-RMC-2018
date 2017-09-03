#ifndef CAMERATWO_H
#define CAMERATWO_H
#include <opencv2/core/core.hpp>
#include <opencv2/video.hpp>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <AMQPcpp.h>
#include <QDialog>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc.hpp>

namespace Ui {
class CameraTwo;
}

class CameraTwo : public QDialog
{
    Q_OBJECT

public:
    explicit CameraTwo(QWidget *parent = 0, QString login = "login");
    ~CameraTwo();

public slots:
    void handleFrame(QString key, QByteArray data);
    void camTwoStream();
    void camTwoSubscription();

private slots:


private:
    Ui::CameraTwo *ui;
    QString str_login;
    AMQP *m_amqp;
};

#endif // CAMERATWO_H
