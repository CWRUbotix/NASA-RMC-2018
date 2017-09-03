#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <AMQPcpp.h>
#include "messages.pb.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QCloseEvent>
#include "cameraone.h"
using namespace com::cwrubotix::glennifer;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    explicit MainWindow(QString loginStr, QWidget *parent = 0);
    ~MainWindow();
    void initSubscription();

    static MainWindow instance;

public slots:
    void handleLocomotionUp();
    void handleLocomotionDown();
    void handleLocomotionLeft();
    void handleLocomotionRight();
    void handleLocomotionRelease();
    void handleLocomotionStop();
    void handleLocomotionStraight();
    void handleLocomotionTurn();
    void handleLocomotionStrafe();
    void handleFrontLeftWheelStop();
    void handleFrontLeftWheelSet(int value);
    void handleFrontRightWheelStop();
    void handleFrontRightWheelSet(int value);
    void handleBackLeftWheelStop();
    void handleBackLeftWheelSet(int value);
    void handleBackRightWheelStop();
    void handleBackRightWheelSet(int value);
    void handleFrontLeftWheelPodStraight();
    void handleFrontLeftWheelPodTurn();
    void handleFrontLeftWheelPodStrafe();
    void handleFrontLeftWheelPodSet(int value);
    void handleFrontRightWheelPodStraight();
    void handleFrontRightWheelPodTurn();
    void handleFrontRightWheelPodStrafe();
    void handleFrontRightWheelPodSet(int value);
    void handleBackLeftWheelPodStraight();
    void handleBackLeftWheelPodTurn();
    void handleBackLeftWheelPodStrafe();
    void handleBackLeftWheelPodSet(int value);
    void handleBackRightWheelPodStraight();
    void handleBackRightWheelPodTurn();
    void handleBackRightWheelPodStrafe();
    void handleBackRightWheelPodSet(int value);
    void handleExcavationArmSet(int value);
    void handleExcavationArmDig();
    void handleExcavationArmJog();
    void handleExcavationArmStore();
    void handleExcavationTranslationSet(int value);
    void handleExcavationTranslationExtend();
    void handleExcavationTranslationStop();
    void handleExcavationTranslationRetract();
    void handleExcavationConveyor(bool checked);
    void handleDepositionDumpSet(int value);
    void handleDepositionDumpDump();
    void handleDepositionDumpStop();
    void handleDepositionDumpStore();
    void handleDepositionConveyor(bool checked);
    void handleLowerCurrent(int value);
    void handleUpperCurrent(int value);
    void handleDigSpeed(int value);
    void handleVibrate(bool checked);
    void handleEStop();
    void handleEUnstop();

    void handleTankPivotR();
    void handleTankPivotL();

    void handleExcavationArmDrive();

    void handleTankPivotRK();
    void handleTankPivotLK();

    void actionTabRight();
    void actionTabLeft();

    void digConfig();
    void dumpConfig();
    void forwardConfig();
    void turnConfig();
    void strafeConfig();

    void drill(float value, QString key);
    void handleDrill(int type, float value);
    void digDeep(float meters);
    void digFwd(float meters);
    void digRev(float meters);
    void digEnd();
    void bcktWdraw();
    void bcktFwd();
    void bcktRev();
    void dumpExtend();
    void dumpRetract();
    void dumpConveyor(bool checked);
    void drillParameters(double depth);
    void armDrive();
    void armDig();
    void armGTFO();
    void armPrep();
    void inverseExcavationConveyer(bool checked);
    void regularExcavationConveyer(bool checked);

    void handleDigDeep();
    void handleDigSurface();
    void handleDigReverse();
    void handleDigStop();
    void handleExcavationTargetDepthSet(int value);
    void handleExcavationDigSpeedSet(int value);
    void handleExcavationMoveSpeedSet(int value);

    void handleState(QString key, QByteArray data);

    void handleA_KeyPress();
    void handleD_KeyPress();


    void keyPressEvent(QKeyEvent *ev);
    void keyReleaseEvent(QKeyEvent *ev);
    void wheelEvent(QWheelEvent* event);
    void closeEvent(QCloseEvent *event);

private slots:
    void on_commandLinkButton_clicked();

private:
    Ui::MainWindow *ui;
    AMQP *m_amqp;
    QString m_loginStr;
    QGraphicsScene *locomotionScene;
    QGraphicsScene *excavationScene;
    QGraphicsScene *depositionScene;
    QGraphicsRectItem *rectangle1;
    QGraphicsRectItem *rectangle2;
    QGraphicsRectItem *rectangle3;
    QGraphicsRectItem *rectangle4;
    int m_desiredConfig = 0; // 0 is straight, 1 is turn, 2 is strafe
    int m_configSpeeds[3] = {100, 60, 50};

    CameraOne *cameraOne;
    int m_digConfig = 0; //0 is drive, 1 is dig, 2 is GTFO
    bool isInDig = false;
    bool isInDump = false;
    //int drillType = 0; //0 is deep, 1 is surface, 2 is reverse
    //new value for drilling to restrict slider and box values in for example drill deep and drill surface like michael told me
    float drillValue = 0.0F;
    QString drillDeep = "drill.deep";
    QString drillFwd = "drill.surface";
    QString drillEnd = "drill.end";
    int m_dumpConfig = 0; //0 is store, 1 is dump/fully extended
};

#endif // MAINWINDOW_H
