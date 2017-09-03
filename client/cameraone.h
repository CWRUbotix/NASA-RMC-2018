#ifndef CAMERAONE_H
#define CAMERAONE_H
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
class CameraOne;
}

class CameraOne : public QDialog
{
    Q_OBJECT

public:
    explicit CameraOne(QWidget *parent = 0, QString login = "login");
    ~CameraOne();

public slots:
    void handleFrameOne(QString key, QByteArray data);
    void camOneStream();
    void camOneSubscription();

    void handleFrameTwo(QString key, QByteArray data);
    void camTwoStream();
    void camTwoSubscription();

    void handleFrameThree(QString key, QByteArray data);
    void camThreeStream();
    void camThreeSubscription();

    void handleFrameFour(QString key, QByteArray data);
    void camFourStream();
    void camFourSubscription();

    void handleFrameFive(QString key, QByteArray data);
    void camFiveStream();
    void camFiveSubscription();

private slots:


private:
    Ui::CameraOne *ui;
    QString str_login;
    AMQP *m_amqp1;
    AMQP *m_amqp2;
    AMQP *m_amqp3;
    AMQP *m_amqp4;
    AMQP *m_amqp5;
};

#endif // CAMERAONE_H
