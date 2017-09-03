#ifndef CAMERATHREE_H
#define CAMERATHREE_H
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
class CameraThree;
}

class CameraThree : public QDialog
{
    Q_OBJECT

public:
    explicit CameraThree(QWidget *parent = 0, QString login = "login");
    ~CameraThree();

public slots:
    void handleFrame(QString key, QByteArray data);
    void camThreeStream();
    void camThreeSubscription();

private slots:


private:
    Ui::CameraThree *ui;
    QString str_login;
    AMQP *m_amqp;
};


#endif // CAMERATHREE_H
