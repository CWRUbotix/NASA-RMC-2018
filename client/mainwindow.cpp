#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <AMQPcpp.h>
#include "messages.pb.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QImage>
#include <QMessageBox>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QDebug>
#include "consumerthread.h"
#include <QCloseEvent>
#include "doubleedit.h"
#include "drillslider.h"

/*
 * In this file, the state of the robot is queried by RPC.
 * Since those RPC calls do not exist yet, instead we are using fake calls like this:
 *
 * float x = 0; // getLocomotionFrontLeftWheelRpm();
 * float y = 0; // getLocomotionFrontLeftWheelPodPos();
 * LocomotionConfiguration x = STRAIGHT; // getLocomotionConfiguration();
 * float a = 0; // getLocomotionStraightSpeed();
 * float b = 0; // getLocomotionTurnSpeed();
 * float c = 0; // getLocomotionStrafeSpeed();
 */

using namespace com::cwrubotix::glennifer;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    /*

    QMap<QLineEdit*, QSlider*> lineSliders;

    lineSliders.insert(ui->lineEdit_BackLeftWheel, ui->slider_BackLeftWheel);

    ui->lineEdit_FrontLeftWheel->setValidator(new QIntValidator(-60, 60));
    */

    /*
    ui->lineEdit_FrontLeftWheel->setValidator(new QDoubleValidator(-60, 60, 0));
    ui->lineEdit_FrontRightWheel->setValidator(new QDoubleValidator(-60, 60, 0));
    ui->lineEdit_BackLeftWheel->setValidator(new QDoubleValidator(-60, 60, 0));
    ui->lineEdit_BackRightWheel->setValidator(new QDoubleValidator(-60, 60, 0));
    ui->lineEdit_FrontLeftWheelPod->setValidator(new QDoubleValidator(0, 90, 0));
    ui->lineEdit_FrontRightWheelPod->setValidator(new QDoubleValidator(0, 90, 0));
    ui->lineEdit_BackLeftWheelPod->setValidator(new QDoubleValidator(0, 90, 0));
    ui->lineEdit_BackRightWheelPod->setValidator(new QDoubleValidator(0, 90, 0));
    ui->lineEdit_DepositionDump->setValidator(new QDoubleValidator(-100, 100, 0));
    ui->lineEdit_ExcavationArm->setValidator(new QDoubleValidator(0, 90, 0));
    ui->lineEdit_ExcavationTranslation->setValidator(new QDoubleValidator(-100, 100, 0));
    */

    //ui->drill_Slider->setRange(0,);

    connect(ui->lineEdit_FrontLeftWheel, &IntEdit::valueEdited, ui->slider_FrontLeftWheel, &QSlider::setValue);
    connect(ui->lineEdit_FrontRightWheel, &IntEdit::valueEdited, ui->slider_FrontRightWheel, &QSlider::setValue);
    connect(ui->lineEdit_BackLeftWheel, &IntEdit::valueEdited, ui->slider_BackLeftWheel, &QSlider::setValue);
    connect(ui->lineEdit_BackRightWheel, &IntEdit::valueEdited, ui->slider_BackRightWheel, &QSlider::setValue);
    connect(ui->lineEdit_FrontLeftWheelPod, &IntEdit::valueEdited, ui->slider_FrontLeftWheelPod, &QSlider::setValue);
    connect(ui->lineEdit_FrontRightWheelPod, &IntEdit::valueEdited, ui->slider_FrontRightWheelPod, &QSlider::setValue);
    connect(ui->lineEdit_BackLeftWheelPod, &IntEdit::valueEdited, ui->slider_BackLeftWheelPod, &QSlider::setValue);
    connect(ui->lineEdit_BackRightWheelPod, &IntEdit::valueEdited, ui->slider_BackRightWheelPod, &QSlider::setValue);
    connect(ui->lineEdit_DepositionDump, &IntEdit::valueEdited, ui->slider_DepositionDump, &QSlider::setValue);
    connect(ui->lineEdit_ExcavationArm, &IntEdit::valueEdited, ui->slider_ExcavationArm, &QSlider::setValue);
    connect(ui->lineEdit_ExcavationTranslation, &IntEdit::valueEdited, ui->slider_ExcavationTranslation, &QSlider::setValue);

    //New Sliders for Drilling
    connect(ui->pushButton_DigDeep, &QPushButton::clicked, this, &MainWindow::handleDigDeep);
    connect(ui->pushButton_DigSurface, &QPushButton::clicked, this, &MainWindow::handleDigSurface);
    connect(ui->pushButton_DigStop, &QPushButton::clicked, this, &MainWindow::handleDigStop);
    connect(ui->checkBox_BucketConveyorOn, &QCheckBox::stateChanged, this, &MainWindow::handleExcavationConveyor);

    locomotionScene = new QGraphicsScene(this);
    ui->graphicsView->setScene(locomotionScene);

    //excavationScene = new QGraphicsScene(this);
    //ui->graphicsView_2->setScene(excavationScene);

    depositionScene = new QGraphicsScene(this);
    ui->graphicsView_3->setScene(depositionScene);

    QBrush greenBrush(Qt::green);
    QBrush grayBrush(Qt::gray);
    QBrush redBrush(Qt::red);
    QBrush blueBrush(Qt::blue);
    QPen outlinePen(Qt::black);
    outlinePen.setWidth(2);

    rectangle1 = locomotionScene->addRect(-50, -80, 10, 20, outlinePen, greenBrush);
    rectangle1->setTransformOriginPoint(-45, -70);
    rectangle2 = locomotionScene->addRect(50, -80, 10, 20, outlinePen, greenBrush);
    rectangle2->setTransformOriginPoint(55, -70);
    rectangle3 = locomotionScene->addRect(-50, 80, 10, 20, outlinePen, greenBrush);
    rectangle3->setTransformOriginPoint(-45, 90);
    rectangle4 = locomotionScene->addRect(50, 80, 10, 20, outlinePen, greenBrush);
    rectangle4->setTransformOriginPoint(55, 90);

    //excavationScene->addRect(-80, -20, 160, 40, outlinePen, grayBrush);
    //excavationScene->addRect(-100, -10, 160, 20, outlinePen, blueBrush);

    QPolygonF poly(4);
    poly[0] = QPointF(-120, -60);
    poly[1] = QPointF(20, 80);
    poly[2] = QPointF(50, 80);
    poly[3] = QPointF(80, -60);

    depositionScene->addPolygon(poly, outlinePen, redBrush);

    QObject::connect(ui->locomotion_UpButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionUp);
    QObject::connect(ui->locomotion_UpButton, &QPushButton::released,
                     this, &MainWindow::handleLocomotionRelease);
    QObject::connect(ui->locomotion_DownButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionDown);
    QObject::connect(ui->locomotion_DownButton, &QPushButton::released,
                     this, &MainWindow::handleLocomotionRelease);
    QObject::connect(ui->locomotion_LeftButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionLeft);
    QObject::connect(ui->locomotion_LeftButton, &QPushButton::released,
                     this, &MainWindow::handleLocomotionRelease);
    QObject::connect(ui->locomotion_RightButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionRight);
    QObject::connect(ui->locomotion_RightButton, &QPushButton::released,
                     this, &MainWindow::handleLocomotionRelease);
    QObject::connect(ui->locomotion_StopButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionStop);
    QObject::connect(ui->locomotion_StraightButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionStraight);
    QObject::connect(ui->locomotion_TurnButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionTurn);
    QObject::connect(ui->locomotion_StrafeButton, &QPushButton::clicked,
                     this, &MainWindow::handleLocomotionStrafe);
    QObject::connect(ui->pushButton_FrontLeftWheelStop, &QPushButton::clicked,
                     this, &MainWindow::handleFrontLeftWheelStop);
    QObject::connect(ui->pushButton_FrontRightWheelStop, &QPushButton::clicked,
                     this, &MainWindow::handleFrontRightWheelStop);
    QObject::connect(ui->pushButton_BackLeftWheelStop, &QPushButton::clicked,
                     this, &MainWindow::handleBackLeftWheelStop);
    QObject::connect(ui->pushButton_BackRightWheelStop, &QPushButton::clicked,
                     this, &MainWindow::handleBackRightWheelStop);
    QObject::connect(ui->pushButton_FrontLeftWheelPodStraight, &QPushButton::clicked,
                     this, &MainWindow::handleFrontLeftWheelPodStraight);
    QObject::connect(ui->pushButton_FrontRightWheelPodStraight, &QPushButton::clicked,
                     this, &MainWindow::handleFrontRightWheelPodStraight);
    QObject::connect(ui->pushButton_BackLeftWheelPodStraight, &QPushButton::clicked,
                     this, &MainWindow::handleBackLeftWheelPodStraight);
    QObject::connect(ui->pushButton_BackRightWheelPodStraight, &QPushButton::clicked,
                     this, &MainWindow::handleBackRightWheelPodStraight);
    QObject::connect(ui->pushButton_FrontLeftWheelPodTurn, &QPushButton::clicked,
                     this, &MainWindow::handleFrontLeftWheelPodTurn);
    QObject::connect(ui->pushButton_FrontRightWheelPodTurn, &QPushButton::clicked,
                     this, &MainWindow::handleFrontRightWheelPodTurn);
    QObject::connect(ui->pushButton_BackLeftWheelPodTurn, &QPushButton::clicked,
                     this, &MainWindow::handleBackLeftWheelPodTurn);
    QObject::connect(ui->pushButton_BackRightWheelPodTurn, &QPushButton::clicked,
                     this, &MainWindow::handleBackRightWheelPodTurn);
    QObject::connect(ui->pushButton_FrontLeftWheelPodStrafe, &QPushButton::clicked,
                     this, &MainWindow::handleFrontLeftWheelPodStrafe);
    QObject::connect(ui->pushButton_FrontRightWheelPodStrafe, &QPushButton::clicked,
                     this, &MainWindow::handleFrontRightWheelPodStrafe);
    QObject::connect(ui->pushButton_BackLeftWheelPodStrafe, &QPushButton::clicked,
                     this, &MainWindow::handleBackLeftWheelPodStrafe);
    QObject::connect(ui->pushButton_BackRightWheelPodStrafe, &QPushButton::clicked,
                     this, &MainWindow::handleBackRightWheelPodStrafe);
    QObject::connect(ui->slider_FrontLeftWheel, &QSlider::valueChanged,
                     this, &MainWindow::handleFrontLeftWheelSet);
    QObject::connect(ui->slider_FrontRightWheel, &QSlider::valueChanged,
                     this, &MainWindow::handleFrontRightWheelSet);
    QObject::connect(ui->slider_BackLeftWheel, &QSlider::valueChanged,
                     this, &MainWindow::handleBackLeftWheelSet);
    QObject::connect(ui->slider_BackRightWheel, &QSlider::valueChanged,
                     this, &MainWindow::handleBackRightWheelSet);
    QObject::connect(ui->slider_FrontLeftWheelPod, &QSlider::valueChanged,
                     this, &MainWindow::handleFrontLeftWheelPodSet);
    QObject::connect(ui->slider_FrontRightWheelPod, &QSlider::valueChanged,
                     this, &MainWindow::handleFrontRightWheelPodSet);
    QObject::connect(ui->slider_BackLeftWheelPod, &QSlider::valueChanged,
                     this, &MainWindow::handleBackLeftWheelPodSet);
    QObject::connect(ui->slider_BackRightWheelPod, &QSlider::valueChanged,
                     this, &MainWindow::handleBackRightWheelPodSet);
    QObject::connect(ui->slider_ExcavationArm, &QSlider::valueChanged,
                     this, &MainWindow::handleExcavationArmSet);
    QObject::connect(ui->pushButton_ExcavationArmDig, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationArmDig);
    QObject::connect(ui->pushButton_ExcavationArmJog, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationArmJog);
    QObject::connect(ui->pushButton_ExcavationArmStore, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationArmStore);
    QObject::connect(ui->slider_ExcavationTranslation, &QSlider::valueChanged,
                     this, &MainWindow::handleExcavationTranslationSet);
    QObject::connect(ui->pushButton_ExcavationTranslationExtend, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationTranslationExtend);
    QObject::connect(ui->pushButton_ExcavationTranslationStop, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationTranslationStop);
    QObject::connect(ui->pushButton_ExcavationTranslationRetract, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationTranslationRetract);
    QObject::connect(ui->checkBox_ExcavationConveyor, &QCheckBox::stateChanged,
                     this, &MainWindow::handleExcavationConveyor);
    QObject::connect(ui->slider_DepositionDump, &QSlider::valueChanged,
                     this, &MainWindow::handleDepositionDumpSet);
    QObject::connect(ui->pushButton_DepositionDumpDump, &QPushButton::clicked,
                     this, &MainWindow::handleDepositionDumpDump);
    QObject::connect(ui->pushButton_DepositionDumpStop, &QPushButton::clicked,
                     this, &MainWindow::handleDepositionDumpStop);
    QObject::connect(ui->pushButton_DepositionDumpStore, &QPushButton::clicked,
                     this, &MainWindow::handleDepositionDumpStore);
    QObject::connect(ui->checkBox_DepositionConveyor, &QCheckBox::stateChanged,
                     this, &MainWindow::handleDepositionConveyor);
    QObject::connect(ui->slider_ExcavationTargetDepth, &QSlider::valueChanged,
                     this, &MainWindow::handleExcavationTargetDepthSet);
    QObject::connect(ui->slider_ExcavationDigSpeed, &QSlider::valueChanged,
                     this, &MainWindow::handleExcavationDigSpeedSet);
    QObject::connect(ui->slider_ExcavationMoveSpeed, &QSlider::valueChanged,
                     this, &MainWindow::handleExcavationMoveSpeedSet);
    QObject::connect(ui->spinBox_lowerCurrent, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), // This is ugly because: http://doc.qt.io/qt-5/qspinbox.html#valueChanged
                     this, &MainWindow::handleLowerCurrent);
    QObject::connect(ui->spinBox_upperCurrent, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), // This is ugly because: http://doc.qt.io/qt-5/qspinbox.html#valueChanged
                     this, &MainWindow::handleUpperCurrent);
    QObject::connect(ui->slider_ExcavationDigSpeed, &QSlider::valueChanged,
                     this, &MainWindow::handleDigSpeed);
    QObject::connect(ui->pushButton_EStop, &QPushButton::clicked,
                     this, &MainWindow::handleEStop);
    QObject::connect(ui->pushButton_EUnstop, &QPushButton::clicked,
                     this, &MainWindow::handleEUnstop);
    /*
    QObject::connect(ui->lineEdit_FrontLeftWheel, &QLineEdit::textChanged,
                     ui->slider_BackLeftWheel, &QSlider::setValue)
*/
    //add tankPivotButtonR and tankPivotButtonL
    QObject::connect(ui->tankPivotButtonR, &QPushButton::clicked,
                     this, &MainWindow::handleTankPivotR);
    QObject::connect(ui->tankPivotButtonR, &QPushButton::released,
                     this, &MainWindow::handleLocomotionRelease);
    QObject::connect(ui->tankPivotButtonL, &QPushButton::clicked,
                     this, &MainWindow::handleTankPivotL);
    QObject::connect(ui->tankPivotButtonL, &QPushButton::released,
                     this, &MainWindow::handleLocomotionRelease);

    //Drive Configuration
    QObject::connect(ui->pushButton_ExcavationArmDrive, &QPushButton::clicked,
                     this, &MainWindow::handleExcavationArmDrive);
    QObject::connect(ui->checkBox_Vibrate, &QCheckBox::stateChanged,
                     this, &MainWindow::handleVibrate);
}

MainWindow::MainWindow(QString loginStr, QWidget *parent) :
    MainWindow::MainWindow(parent)
{
    m_loginStr = loginStr;

    try {
        m_amqp = new AMQP(m_loginStr.toStdString());
    } catch (AMQPException) {
        QMessageBox::critical(0,"Error",QString::fromStdString("AMQP connection error"));
        m_amqp = 0;
    }
}

MainWindow::~MainWindow()
{
    delete locomotionScene;
    delete ui;
}

void MainWindow::handleLocomotionUp() {
    if (0 == m_desiredConfig) { // straight
        LocomotionControlCommandStraight msg;
        msg.set_speed(ui->slider_LocomotionSpeed->value() / 100.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.straight");

        free(msg_buff);
    } else {
        ui->consoleOutputTextBrowser->append("Wrong config");
    }
}

void MainWindow::handleLocomotionDown() {
    if (0 == m_desiredConfig) { // straight
        LocomotionControlCommandStraight msg;
        msg.set_speed(ui->slider_LocomotionSpeed->value() / -100.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.straight");

        free(msg_buff);
    } else {
        ui->consoleOutputTextBrowser->append("Wrong config");
    }
}

void MainWindow::handleLocomotionLeft() {
    if (1 == m_desiredConfig) { // turn
        LocomotionControlCommandTurn msg;
        msg.set_speed(ui->slider_LocomotionSpeed->value() / -100.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.turn");

        free(msg_buff);
    } else if (2 == m_desiredConfig) { // strafe
        LocomotionControlCommandStrafe msg;
        msg.set_speed(ui->slider_LocomotionSpeed->value() / -100.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.strafe");

        free(msg_buff);
    } else {
        ui->consoleOutputTextBrowser->append("Wrong config");
    }
}

void MainWindow::handleLocomotionRight() {
    if (1 == m_desiredConfig) { // turn
        LocomotionControlCommandTurn msg;
        msg.set_speed(ui->slider_LocomotionSpeed->value() / 100.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.turn");

        free(msg_buff);
    } else if (2 == m_desiredConfig) { // strafe
        LocomotionControlCommandStrafe msg;
        msg.set_speed(ui->slider_LocomotionSpeed->value() / 100.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.strafe");

        free(msg_buff);
    } else {
        ui->consoleOutputTextBrowser->append("Wrong config");
    }
}

void MainWindow::handleLocomotionRelease() {
    /*
    if (0 == m_desiredConfig) { // straight
        LocomotionControlCommandStraight msg;
        msg.set_speed(0.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.straight");

        free(msg_buff);
    } else if (1 == m_desiredConfig) { // turn
        LocomotionControlCommandTurn msg;
        msg.set_speed(0.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.turn");

        free(msg_buff);
    } else if (2 == m_desiredConfig) { // strafe
        LocomotionControlCommandStrafe msg;
        msg.set_speed(0.0F);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.strafe");

        free(msg_buff);
    } else {
        ui->consoleOutputTextBrowser->append("Wrong config");
    }
    */
}

void MainWindow::handleLocomotionStop() {
    LocomotionControlCommandStrafe msg;
    msg.set_speed(0.0F);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.strafe");

    free(msg_buff);
}

void MainWindow::handleLocomotionStraight() {
    m_configSpeeds[m_desiredConfig] = ui->slider_LocomotionSpeed->value();
    m_desiredConfig = 0;
    ui->slider_LocomotionSpeed->setValue(m_configSpeeds[m_desiredConfig]);
    LocomotionControlCommandConfigure msg;
    msg.set_power(100.0F);
    msg.set_target(LocomotionControlCommandConfigure_Configuration_STRAIGHT_CONFIG);
    msg.set_timeout(456.0F);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.configure");

    free(msg_buff);
}

void MainWindow::handleLocomotionTurn() {
    //check the arm is in drive and hopper maybe if ()
    m_configSpeeds[m_desiredConfig] = ui->slider_LocomotionSpeed->value();
    m_desiredConfig = 1;
    ui->slider_LocomotionSpeed->setValue(m_configSpeeds[m_desiredConfig]);
    LocomotionControlCommandConfigure msg;
    msg.set_power(100.0F);
    msg.set_target(LocomotionControlCommandConfigure_Configuration_TURN_CONFIG);
    msg.set_timeout(456.0F);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.configure");

    free(msg_buff);
}

void MainWindow::handleLocomotionStrafe() {
    //check the arm is in drive and hopper maybe if ()
    m_configSpeeds[m_desiredConfig] = ui->slider_LocomotionSpeed->value();
    m_desiredConfig = 2;
    ui->slider_LocomotionSpeed->setValue(m_configSpeeds[m_desiredConfig]);
    LocomotionControlCommandConfigure msg;
    msg.set_power(100.0F);
    msg.set_target(LocomotionControlCommandConfigure_Configuration_STRAFE_CONFIG);
    msg.set_timeout(456.0F);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.locomotion.configure");

    free(msg_buff);
}

void MainWindow::handleFrontLeftWheelStop() {
    ui->slider_FrontLeftWheel->setValue(0);
}

void MainWindow::handleFrontLeftWheelSet(int value) {
    SpeedContolCommand msg;
    msg.set_rpm(60 * (value / 100.0F));
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.front_left.wheel_rpm");

    free(msg_buff);

    ui->lineEdit_FrontLeftWheel->setText(QString::number(value));
}

void MainWindow::handleFrontRightWheelStop() {
    ui->slider_FrontRightWheel->setValue(0);
}

void MainWindow::handleFrontRightWheelSet(int value) {
    SpeedContolCommand msg;
    msg.set_rpm(60 * (value / 100.0F));
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.front_right.wheel_rpm");

    free(msg_buff);

    ui->lineEdit_FrontRightWheel->setText(QString::number(value));
}

void MainWindow::handleBackLeftWheelStop() {
    ui->slider_BackLeftWheel->setValue(0);
}

void MainWindow::handleBackLeftWheelSet(int value) {
    SpeedContolCommand msg;
    msg.set_rpm(60 * (value / 100.0F));
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.back_left.wheel_rpm");

    free(msg_buff);

    ui->lineEdit_BackLeftWheel->setText(QString::number(value));
}

void MainWindow::handleBackRightWheelStop() {
    ui->slider_BackRightWheel->setValue(0);
}

void MainWindow::handleBackRightWheelSet(int value) {
    SpeedContolCommand msg;
    msg.set_rpm(60 * (value / 100.0F));
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.back_right.wheel_rpm");

    free(msg_buff);

    ui->lineEdit_BackRightWheel->setText(QString::number(value));
}

void MainWindow::handleFrontLeftWheelPodStrafe() {
    ui->slider_FrontLeftWheelPod->setValue(90);
}

void MainWindow::handleFrontLeftWheelPodTurn() {
    ui->slider_FrontLeftWheelPod->setValue(60);
}

void MainWindow::handleFrontLeftWheelPodStraight() {
    ui->slider_FrontLeftWheelPod->setValue(0);
}

void MainWindow::handleFrontLeftWheelPodSet(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.front_left.wheel_pod_pos");

    free(msg_buff);

    ui->lineEdit_FrontLeftWheelPod->setText(QString::number(value));
}

void MainWindow::handleFrontRightWheelPodStrafe() {
    ui->slider_FrontRightWheelPod->setValue(90);
}

void MainWindow::handleFrontRightWheelPodTurn() {
    ui->slider_FrontRightWheelPod->setValue(60);
}

void MainWindow::handleFrontRightWheelPodStraight() {
    ui->slider_FrontRightWheelPod->setValue(0);
}

void MainWindow::handleFrontRightWheelPodSet(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.front_right.wheel_pod_pos");

    free(msg_buff);

    ui->lineEdit_FrontRightWheelPod->setText(QString::number(value));
}

void MainWindow::handleBackLeftWheelPodStrafe() {
    ui->slider_BackLeftWheelPod->setValue(90);
}

void MainWindow::handleBackLeftWheelPodTurn() {
    ui->slider_BackLeftWheelPod->setValue(60);
}

void MainWindow::handleBackLeftWheelPodStraight() {
    ui->slider_BackLeftWheelPod->setValue(0);
}

void MainWindow::handleBackLeftWheelPodSet(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.back_left.wheel_pod_pos");

    free(msg_buff);

    ui->lineEdit_BackLeftWheelPod->setText(QString::number(value));
}

void MainWindow::handleBackRightWheelPodStrafe() {
    ui->slider_BackRightWheelPod->setValue(90);
}

void MainWindow::handleBackRightWheelPodTurn() {
    ui->slider_BackRightWheelPod->setValue(60);
}

void MainWindow::handleBackRightWheelPodStraight() {
    ui->slider_BackRightWheelPod->setValue(0);
}

void MainWindow::handleBackRightWheelPodSet(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.locomotion.back_right.wheel_pod_pos");

    free(msg_buff);

    ui->lineEdit_BackRightWheelPod->setText(QString::number(value));
}

void MainWindow::handleExcavationArmSet(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.excavation.arm_pos");

    free(msg_buff);

    ui->lineEdit_ExcavationArm->setText(QString::number(value));
}

void MainWindow::handleExcavationArmDig() {
    ui->slider_ExcavationArm->setValue(100);
}

void MainWindow::handleExcavationArmJog() {
    ui->slider_ExcavationArm->setValue(70);
}

void MainWindow::handleExcavationArmDrive() {
    ui->slider_ExcavationArm->setValue(50);
}

void MainWindow::handleExcavationArmStore() {
    ui->slider_ExcavationArm->setValue(10);
}

void MainWindow::handleExcavationTranslationSet(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.excavation.conveyor_translation_displacement");

    free(msg_buff);

    ui->lineEdit_ExcavationTranslation->setText(QString::number(value));
}

void MainWindow::handleExcavationTranslationExtend() {
    ui->slider_ExcavationTranslation->setValue(100);
}

void MainWindow::handleExcavationTranslationStop() {
    ui->slider_ExcavationTranslation->setValue(50);
}

void MainWindow::handleExcavationTranslationRetract() {
    ui->slider_ExcavationTranslation->setValue(0);
}

void MainWindow::handleExcavationConveyor(bool checked) {
    SpeedContolCommand msg;
    int speed = ui->slider_ExcavationConveyor->value();
    speed = ui->checkBox_ExcavationConveyorReverse->isChecked() ? speed : -speed;
    speed = checked ? speed : 0;
    msg.set_rpm(speed);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.excavation.bucket_conveyor_rpm");

    free(msg_buff);
}

void MainWindow::handleDepositionDumpSet(int value) {
    ui->consoleOutputTextBrowser->append("Dump actuators do not have position control\n");
}

void MainWindow::handleDepositionDumpDump() {
    PositionContolCommand msg;
    msg.set_position(100);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.deposition.dump_pos");

    free(msg_buff);

    ui->lineEdit_DepositionDump->setText(QString::number(100));
}

void MainWindow::handleDepositionDumpStop() {
    PositionContolCommand msg;
    msg.set_position(0);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.deposition.dump_pos");

    free(msg_buff);

    ui->lineEdit_DepositionDump->setText(QString::number(0));
}

void MainWindow::handleDepositionDumpStore() {
    PositionContolCommand msg;
    msg.set_position(-100);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.deposition.dump_pos");

    free(msg_buff);

    ui->lineEdit_DepositionDump->setText(QString::number(-100));
}

void MainWindow::handleDepositionConveyor(bool checked) {
    SpeedContolCommand msg;
    int speed = ui->slider_DepositionConveyor->value();
    speed = ui->checkBox_DepositionConveyorReverse->isChecked() ? speed : -speed;
    speed = checked ? speed : 0;
    msg.set_rpm(speed);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.deposition.conveyor_rpm");

    free(msg_buff);
}

void MainWindow::handleLowerCurrent(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.lower_current");

    free(msg_buff);
}

void MainWindow::handleVibrate(bool checked) {
    SpeedContolCommand msg;
    int speed = ui->spinBox_Vibrate->value();
    speed = checked ? speed : 0;
    msg.set_rpm(speed);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.deposition.vibration_rpm");

    free(msg_buff);
}

void MainWindow::handleUpperCurrent(int value) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.upper_current");

    free(msg_buff);
}

void MainWindow::handleDigSpeed(int value) {
    PositionContolCommand msg;
    msg.set_position(value / 10.0F);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.dig_speed");

    free(msg_buff);
}

void MainWindow::handleEStop() {
    StopAllCommand msg;
    msg.set_stop(true);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.system.stop_all");

    free(msg_buff);
}

void MainWindow::handleEUnstop() {
    StopAllCommand msg;
    msg.set_stop(false);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "motorcontrol.system.stop_all");

    free(msg_buff);
}

void MainWindow::handleDigDeep() {
    ExcavationControlCommandDigDeep msg;
    msg.set_depth((float)ui->slider_ExcavationTargetDepth->value());
    msg.set_dig_speed((float)ui->slider_ExcavationDigSpeed->value() / 10);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.dig_deep");

    free(msg_buff);
}

void MainWindow::handleDigSurface() {
    ExcavationControlCommandDigSurface msg;
    msg.set_depth((float)ui->slider_ExcavationTargetDepth->value());
    msg.set_dig_speed((float)ui->slider_ExcavationDigSpeed->value() / 10);
    msg.set_drive_speed((float)ui->slider_ExcavationMoveSpeed->value() / 100);
    msg.set_dist(123);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.dig_surface");

    free(msg_buff);
}

void MainWindow::handleDigReverse() { //same as surface but in opposite direction(negative Speed)
    ExcavationControlCommandDigSurface msg;
    msg.set_depth((float)ui->slider_ExcavationTargetDepth->value());
    msg.set_dig_speed((float)ui->slider_ExcavationDigSpeed->value() / 10);
    msg.set_drive_speed((float)((-1)*(ui->slider_ExcavationMoveSpeed->value() / 100)));
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.dig_surface");

    free(msg_buff);
}

void MainWindow::handleDigStop() {
    ExcavationControlCommandDigEnd msg;
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "subsyscommand.excavation.dig_end");

    free(msg_buff);
}

void MainWindow::handleExcavationTargetDepthSet(int value) {
    ui->lcdNumber_ExcavationTargetDepth->display(value);
}

void MainWindow::handleExcavationDigSpeedSet(int value) {
    ui->lcdNumber_ExcavationDigSpeed->display(value / (10.0F));
}

void MainWindow::handleExcavationMoveSpeedSet(int value) {
    ui->lcdNumber_ExcavationMoveSpeed->display(value / (100.0F));
}

void MainWindow::initSubscription() {
    ConsumerThread *thread = new ConsumerThread(m_loginStr, "abcde");
    connect(thread, &ConsumerThread::receivedMessage, this, &MainWindow::handleState);
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();

    StateSubscribe msg;
    msg.set_replykey("abcde");
    msg.set_interval(0.2F);
    msg.set_locomotion_summary(true);
    msg.set_locomotion_detailed(true);
    msg.set_deposition_summary(true);
    msg.set_deposition_detailed(true);
    msg.set_excavation_summary(true);
    msg.set_excavation_detailed(true);

    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }

    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "state.subscribe");

    on_commandLinkButton_clicked();
}

void MainWindow::handleState(QString key, QByteArray data) {
    State s;
    s.ParseFromArray(data.data(), data.length());
    float fl_rpm = s.locdetailed().front_left_rpm();
    float fr_rpm = s.locdetailed().front_right_rpm();
    float bl_rpm = s.locdetailed().back_left_rpm();
    float br_rpm = s.locdetailed().back_right_rpm();
    float fl_pos = s.locdetailed().front_left_pos();
    float fr_pos = s.locdetailed().front_right_pos();
    float bl_pos = s.locdetailed().back_left_pos();
    float br_pos = s.locdetailed().back_right_pos();
    float speed = s.locsummary().speed();
    float arm_pos = s.excsummary().arm_pos();
    float translation_pos = s.excsummary().displacement();
    float bc_current = s.excdetailed().conveyor_motor_current();
    bool exc_ext_left = s.excdetailed().translation_left_extended();
    bool exc_ext_right = s.excdetailed().translation_right_extended();
    bool exc_ret_left = s.excdetailed().translation_left_retracted();
    bool exc_ret_right = s.excdetailed().translation_right_retracted();
    ui->lcdNumber_FrontLeftWheel->display(fl_rpm);
    if (fl_rpm >= 0) {
        ui->progressBar_FrontLeftWheelForwards->setValue(fl_rpm);
    } else {
        ui->progressBar_FrontLeftWheelBackwards->setValue(-fl_rpm);
    }
    ui->lcdNumber_FrontRightWheel->display(fr_rpm);
    if (fr_rpm >= 0) {
        ui->progressBar_FrontRightWheelForwards->setValue(fr_rpm);
    } else {
        ui->progressBar_FrontRightWheelBackwards->setValue(-fr_rpm);
    }
    ui->lcdNumber_BackLeftWheel->display(bl_rpm);
    if (bl_rpm >= 0) {
        ui->progressBar_BackLeftWheelForwards->setValue(bl_rpm);
    } else {
        ui->progressBar_BackLeftWheelBackwards->setValue(-bl_rpm);
    }
    ui->lcdNumber_BackRightWheel->display(br_rpm);
    if (br_rpm >= 0) {
        ui->progressBar_BackRightWheelForwards->setValue(br_rpm);
    } else {
        ui->progressBar_BackRightWheeBackwards->setValue(-br_rpm);
    }
    ui->lcdNumber_FrontLeftWheelPod->display(fl_pos);
    ui->lcdNumber_FrontRightWheelPod->display(fr_pos);
    ui->lcdNumber_BackLeftWheelPod->display(bl_pos);
    ui->lcdNumber_BackRightWheelPod->display(br_pos);
    ui->progressBar_FrontLeftWheelPod->setValue(fl_pos);
    ui->progressBar_FrontRightWheelPod->setValue(fr_pos);
    ui->progressBar_BackLeftWheelPod->setValue(bl_pos);
    ui->progressBar_BackRightWheelPod->setValue(br_pos);
    rectangle1->setRotation(fl_pos);
    rectangle2->setRotation(-fr_pos);
    rectangle3->setRotation(-bl_pos);
    rectangle4->setRotation(br_pos);
    ui->speedometer->setSpeed(speed);
    ui->speedometer->setPower((speed * 100) / 0.7F);
    ui->progressBar_ExcavationArm->setValue(arm_pos);
    ui->progressBar_ExcavationTranslation->setValue(translation_pos);
    ui->ledIndicator_ExcavationTranslationExtendLeft->setState(exc_ext_left);
    ui->ledIndicator_ExcavationTranslationExtendRight->setState(exc_ext_right);
    ui->ledIndicator_ExcavationTranslationRetractLeft->setState(exc_ret_left);
    ui->ledIndicator_ExcavationTranslationRetractRight->setState(exc_ret_right);
    ui->speedometer_Excavation->setSpeed(translation_pos);
    ui->speedometer_Excavation->setPower(bc_current * (100.0F/20.0F));
}

void MainWindow::keyPressEvent(QKeyEvent *ev) {
    if (ev->isAutoRepeat()) {
        QWidget::keyPressEvent(ev);
    } else {
        switch (ev->key()) {
        case (Qt::Key_Space):
            handleEStop();
            //stop all other motors and automatic procesess
            break;
        case (Qt::Key_W):
            handleLocomotionUp();
            break;
        case (Qt::Key_A):
            handleA_KeyPress();
            break;
        case (Qt::Key_S):
            handleLocomotionDown();
            break;
        case (Qt::Key_D):
            handleD_KeyPress();
            break;
        case (Qt::Key_I):
            handleLocomotionStraight();
            break;
        case (Qt::Key_O):
            turnConfig();
            break;
        case (Qt::Key_P):
            strafeConfig();
            break;
        case (Qt::Key_J):
            ui->slider_LocomotionSpeed->setValue(ui->slider_LocomotionSpeed->value() - 10);
            break;
        case (Qt::Key_K):
            ui->slider_LocomotionSpeed->setValue(ui->slider_LocomotionSpeed->value() + 10);
            break;
        //case (Qt::Key_R):
          //  handleTankPivotRK();
          //  break;
        //case (Qt::Key_L):
          //  handleTankPivotLK;
          //  break;
        case (Qt::Key_E):
             actionTabRight();
             break;
        case (Qt::Key_Q):
             actionTabLeft();
             break;
        case (Qt::Key_Y):
             digConfig();
             break;
        case (Qt::Key_U):
             dumpConfig();
             break;
        case (Qt::Key_1):
             handleDigDeep();
             break;
        case (Qt::Key_2):
             handleDigSurface();
             break;
        case (Qt::Key_3):
             handleDigReverse();
             break;
        case (Qt::Key_4):
             handleDigStop();
             break;
        case (Qt::Key_5):
             handleExcavationTranslationStop();
             break;
        case (Qt::Key_6):
             regularExcavationConveyer(true);
             break;
        case (Qt::Key_7):
             inverseExcavationConveyer(true);
             break;
        case (Qt::Key_8):
             armPrep();
             break;
        case (Qt::Key_9):
             armDrive();
             break;
        case (Qt::Key_0):
             armDig();
             break;
        case (Qt::Key_Minus):
             armGTFO();
             break;
        case (Qt::Key_Z):
             dumpExtend();
             break;
        case (Qt::Key_X):
             dumpRetract();
             break;
        case (Qt::Key_C):
             dumpConveyor(true);
             break;
        case (Qt::Key_V):
             //Extra
             break;
        default:
            QWidget::keyPressEvent(ev);
            break;
        }
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *ev) {
    if (ev->isAutoRepeat()) {
        QWidget::keyReleaseEvent(ev);
    } else {
        switch (ev->key()) {
        case (Qt::Key_Space):
            break;
        case (Qt::Key_W):
            handleLocomotionStop();
            break;
        case (Qt::Key_A):
            handleLocomotionStop();
            break;
        case (Qt::Key_S):
            handleLocomotionStop();
            break;
        case (Qt::Key_D):
            handleLocomotionStop();
            break;
        case (Qt::Key_I):
            break;
        case (Qt::Key_O):
            break;
        case (Qt::Key_P):
            break;
        case (Qt::Key_J):
            break;
        case (Qt::Key_K):
            break;
        case (Qt::Key_R):
            //handleLocomotionStop();
            break;
        case (Qt::Key_L):
            //handleLocomotionStop();
            break;
        case (Qt::Key_E):
            break;
        case (Qt::Key_Q):
            break;
        case (Qt::Key_Y):
             break;
        case (Qt::Key_U):
             break;
        case (Qt::Key_1):
             //dig deep
             break;
        case (Qt::Key_2):
             //dig forward
             break;
        case (Qt::Key_3):
             //dig rev
             break;
        case (Qt::Key_4):
             //dig stop
             break;
        case (Qt::Key_5):
             //arm bucket retract
             break;
        case (Qt::Key_6):
             handleExcavationConveyor(false);
             break;
        case (Qt::Key_7):
             handleExcavationConveyor(false);
             break;
        case (Qt::Key_8):
             //arm dig
             break;
        case (Qt::Key_9):
             //arm drive
             break;
        case (Qt::Key_0):
             //arm dig
             break;
        case (Qt::Key_Minus):
             //arm GTFO
             break;
        case (Qt::Key_Z):
             //DUMP extend
             break;
        case (Qt::Key_X):
             //DUMP retract
             break;
        case (Qt::Key_C):
             handleDepositionConveyor(false);
             break;
        case (Qt::Key_V):
             //extra
             break;
        default:
            QWidget::keyReleaseEvent(ev);
            break;
        }
    }
}

void MainWindow::wheelEvent(QWheelEvent* event) {
    int delta = event->angleDelta().y();
    delta = (delta > 0) ? 5 : -5;
    ui->slider_LocomotionSpeed->setValue(ui->slider_LocomotionSpeed->value() + delta);
}

void MainWindow::closeEvent(QCloseEvent *event) {
    StateSubscribe msg;
    msg.set_replykey("abcde");
    msg.set_interval(0.2F);
    msg.set_locomotion_summary(false);
    msg.set_locomotion_detailed(true);
    msg.set_deposition_summary(false);
    msg.set_deposition_detailed(false);
    msg.set_excavation_summary(false);
    msg.set_excavation_detailed(false);

    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }

    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "state.unsubscribe");
}

void MainWindow::on_commandLinkButton_clicked()
{
    //reset frame and img to make sure it is not conflicting
    cameraOne = new CameraOne(this, m_loginStr);
    cameraOne->CameraOne::camOneStream();
    cameraOne->CameraOne::camTwoStream();
    cameraOne->CameraOne::camThreeStream();
    cameraOne->CameraOne::camFourStream();
    cameraOne->CameraOne::camFiveStream();
    cameraOne->show();

    /*cameraTwo = new CameraTwo(this, m_loginStr);
    cameraTwo->CameraTwo::camTwoStream();
    cameraTwo->show();

    cameraThree = new CameraThree(this, m_loginStr);
    cameraThree->CameraThree::camThreeStream();
    cameraThree->show();

    cameraFour = new CameraFour(this, m_loginStr);
    cameraFour->CameraFour::camFourStream();
    cameraFour->show();

    cameraFive = new CameraFive(this, m_loginStr);
    cameraFive->CameraFive::camFiveStream();
    cameraFive->show();*/
}


void MainWindow::handleTankPivotR() {
    if (0 == m_desiredConfig) { // straight
        int leftSide = (ui->slider_LocomotionSpeed->value()); //left wheel speed
        int rightSide = (ui->slider_UpsetSpeed->value()* (-1)); //right wheel speed
        handleFrontRightWheelSet(rightSide);
        handleBackRightWheelSet(rightSide);
        handleFrontLeftWheelSet(leftSide);
        handleBackLeftWheelSet(leftSide);

    } else {
        ui->consoleOutputTextBrowser->append("Wrong config, tank pivot only works in straight");
    }
}

void MainWindow::handleTankPivotL() {
    if (0 == m_desiredConfig) { // straight
        int rightSide = (ui->slider_LocomotionSpeed->value()); //right wheel speed
        int leftSide = (ui->slider_UpsetSpeed->value()* (-1)); //left wheel speed
        handleFrontRightWheelSet(rightSide);
        handleBackRightWheelSet(rightSide);
        handleFrontLeftWheelSet(leftSide);
        handleBackLeftWheelSet(leftSide);

    } else {
        ui->consoleOutputTextBrowser->append("Wrong config, tank pivot only works on straight");
    }
}

void MainWindow::handleTankPivotRK() {
    if (0 == m_desiredConfig) { // straight
        int leftSide = (ui->slider_LocomotionSpeed->value()); //right wheel speed
        int rightSide = (ui->slider_LocomotionSpeed->value()* (-1)); //left wheel speed
        handleFrontRightWheelSet(rightSide);
        handleBackRightWheelSet(rightSide);
        handleFrontLeftWheelSet(leftSide);
        handleBackLeftWheelSet(leftSide);

    } else {
        ui->consoleOutputTextBrowser->append("Wrong config, tank pivot only works on straight");
    }
}

void MainWindow::handleTankPivotLK() {
    if (0 == m_desiredConfig) { // straight
        int rightSide = (ui->slider_LocomotionSpeed->value()); //right wheel speed
        int leftSide = (ui->slider_LocomotionSpeed->value()* (-1)); //left wheel speed
        handleFrontRightWheelSet(rightSide);
        handleBackRightWheelSet(rightSide);
        handleFrontLeftWheelSet(leftSide);
        handleBackLeftWheelSet(leftSide);

    } else {
        ui->consoleOutputTextBrowser->append("Wrong config, tank pivot only works on straight");
    }
}

//Tab Right
void MainWindow::actionTabRight() {
    if(ui->tabWidget->currentIndex() >= -1) {
        ui->tabWidget->setCurrentIndex(ui->tabWidget->currentIndex()+1);
    }
    if(ui->tabWidget->currentIndex() >4) {
        ui->tabWidget->setCurrentIndex(1);
    }
}

//Tab Left
void MainWindow::actionTabLeft() {
    if(ui->tabWidget->currentIndex() <= 5) {
        ui->tabWidget->setCurrentIndex(ui->tabWidget->currentIndex()-1);
    }
    if(ui->tabWidget->currentIndex() < 1) {
        ui->tabWidget->setCurrentIndex(5);
    }
}

//Configurations
void MainWindow::forwardConfig() {
    if (0 == m_desiredConfig) {
    ui->consoleOutputTextBrowser->append("Already in forward Configuration");
    isInDig = false;
    isInDump = false;
    }
    else
        ui->consoleOutputTextBrowser->append("Switching to forward Configuration");
        handleLocomotionStraight();
        isInDig = false;
        isInDump = false;
}

void MainWindow::digConfig() {
    if (0 == m_desiredConfig) {
    ui->consoleOutputTextBrowser->append("Already in Dig Configuration");
    isInDig = true;
    isInDump = false;
    }
    else
        ui->consoleOutputTextBrowser->append("Switching to Dig Configuration");
        handleLocomotionStraight();
        isInDig = true;
        isInDump = false;
}

void MainWindow::dumpConfig() {
    if (0 == m_desiredConfig) {
    ui->consoleOutputTextBrowser->append("Already in Dump Configuration");
    isInDig = false;
    isInDump = true;
    }
    else
        ui->consoleOutputTextBrowser->append("Switching to Dump Configutation");
        handleLocomotionStraight();
        isInDig = false;
        isInDump = true;
}

void MainWindow::turnConfig() {
    if (ui->slider_ExcavationArm->value() > 50) {
        ui->consoleOutputTextBrowser->append("Excavation arm is preventing turn configuration,\n please retract the Excavation arm");
        isInDig = false;
        isInDump = false;
    }
    else
        handleLocomotionTurn();
        isInDig = false;
        isInDump = false;
}

void MainWindow::strafeConfig() {
    if (ui->slider_ExcavationArm->value() > 50) {
        ui->consoleOutputTextBrowser->append("Excavation arm is preventing strafe configuration,\n please retract the Excavation arm");
        isInDig = false;
        isInDump = false;
    }
    else
        handleLocomotionStrafe();
        isInDig = false;
        isInDump = false;
}

/* * ON ALL OF THESE MAKE SURE THE KEY RELEASE WORKS AS IT IS MEANT TO * */
void MainWindow::drill(float value, QString key) {
    PositionContolCommand msg;
    msg.set_position(value);
    msg.set_timeout(456);
    int msg_size = msg.ByteSize();
    void *msg_buff = malloc(msg_size);
    if (!msg_buff) {
        ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
        return;
    }
    msg.SerializeToArray(msg_buff, msg_size);

    AMQPExchange * ex = m_amqp->createExchange("amq.topic");
    ex->Declare("amq.topic", "topic", AMQP_DURABLE);
    ex->Publish((char*)msg_buff, msg_size, "drill.deep");

    free(msg_buff);
}

void MainWindow::handleDrill(int type, float value) {
    if(m_digConfig == 1) {
        switch(type) {
        case 0:
            digDeep(value);
            break;
        case 1:
            digFwd(value); //dig surface
            break;
        case 2:
            digRev(value);
            break;
        }
    }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dig mode\n please press enter dig configuration, thenn arm dig");
}

void MainWindow::digDeep(float meters) {
    if(meters == 0) {
        if(meters <= 0.5) {
            drill(meters, drillDeep);
        }
        else
            ui->consoleOutputTextBrowser->append("Input Value Exceeds Drill Deep Parameters\n please enter a value between 0.0 and 0.5 meters");
    }
}

void MainWindow::digFwd(float meters) {
    if(meters == 0) {
        if(meters <= 3.0) {
            drill(meters, drillFwd);
        }
        else
            ui->consoleOutputTextBrowser->append("Input Value Exceeds Drill Deep Parameters\n please enter a value between 0.0 and 0.5 meters");
    }
    /* CHECK FOR isInDig AND Modify the dig and dump flags in fwd config*/
    //send command to hci or handle here, not sure
    /* can make separate control structure for digfwd rev and such if it is meant
     * to be ui controled as opposed to automated, other wise,
     * engage sequence here with simpler structure to check for desiredConfig == 0
     * since that is default for both dig and dump and allows for drive to occur
     * if it is a separate call using handleLocomotion as opposed to the configs
     * We may want to do Something else though not sure
     */
}

void MainWindow::digRev(float meters) {
    if(meters == 0) {
        if(meters <= 3.0) {
            drill((-1.0F*meters), drillFwd); //the reverse of fwd
        }
        else
            ui->consoleOutputTextBrowser->append("Input Value Exceeds Drill Deep Parameters\n please enter a value between 0.0 and 0.5 meters");
    }
}

void MainWindow::digEnd() { //Obsolete
    //Either send stop command from hci or do something with stopAll
    //stopAll may not affect itonly the motors but still motors
    drill(0.0F, drillEnd);
}

void MainWindow::bcktWdraw() { //bucket retract
    handleExcavationTranslationStop();
}

//The Rest of these may require they're own function
//if it cannot be set in reverse in the boolean check function

void MainWindow::bcktFwd() {

}

void MainWindow::bcktRev() {

}

// Helper for Drilling
void MainWindow::drillParameters(double depth) {
    //ui->drill_Depth->display(depth);
    drillValue = (float)(depth);
}

void MainWindow::armDrive() {
    if (m_desiredConfig == 0 && isInDig == true && isInDump == false) {
        handleExcavationArmDrive();
        m_digConfig = 0;
    }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dig mode\n please press enter dig configuration");
}

void MainWindow::armDig() {
    if (m_desiredConfig == 0 && isInDig == true && isInDump == false) {
        handleExcavationArmJog();
        m_digConfig = 1;
    }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dig mode\n please press enter dig configuration");
}

void MainWindow::armGTFO() {
    if (m_desiredConfig == 0 && isInDig == true && isInDump == false) {
        handleExcavationArmDig();
        m_digConfig = 2;
    }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dig mode\n please press enter dig configuration");
}

void MainWindow::armPrep() {
    if(m_desiredConfig == 0 && isInDig == true && isInDump == false) {
        handleExcavationArmStore();
        m_digConfig = 0; //Store but should still be able to drive
    }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dig mode\n please press enter dig configuration");
}

void MainWindow::dumpExtend() {
    if(isInDump == true && isInDig == false) {
        if(m_digConfig == 2) {
            if(m_dumpConfig != 1) {
                handleDepositionDumpDump();
                m_dumpConfig = 1;
            }
            else
                ui->consoleOutputTextBrowser->append("Currently already fully extended or something went wrong");
        }
        else
            ui->consoleOutputTextBrowser->append("Please tell the excavation arm to GTFO");
    }
    //else if(ui->slider_DepositionDump->value() == 100)  {
    //    m_dumpConfig = 1;
   // }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dump mode\n please press enter dump configuration");
}

void MainWindow::dumpRetract() {
    if(isInDump == true && isInDig == false) {
        if(m_digConfig == 2) {
            if(m_dumpConfig != 0) {
                handleDepositionDumpStore();
                m_dumpConfig = 0;
            }
            else
                ui->consoleOutputTextBrowser->append("Currently already fully stored or something went wrong");
        }
        else
            ui->consoleOutputTextBrowser->append("Please tell the excavation arm to GTFO");
    }
    else
        ui->consoleOutputTextBrowser->append("Currently not in dump mode\n please press enter dump configuration");
   // else if(ui->slider_DepositionDump->value() == -100)  {
   //     m_dumpConfig = 0;
   // }
}

void MainWindow::dumpConveyor(bool checked) {
    if(m_dumpConfig == 1) { //absolutely has to be on fully extended
        handleDepositionConveyor(checked);
    }
    else
        ui->consoleOutputTextBrowser->append("It is imperative you extend deposition before running the conveyor");
}

/* Anti Bucket Conveyor Sends negative Speed */
void MainWindow::inverseExcavationConveyer(bool checked) {
    if(m_digConfig == 1) {
        SpeedContolCommand msg;
        int speed = (-1)*(ui->slider_ExcavationConveyor->value());
        msg.set_rpm(checked ? speed : 0);
        msg.set_timeout(456);
        int msg_size = msg.ByteSize();
        void *msg_buff = malloc(msg_size);
        if (!msg_buff) {
            ui->consoleOutputTextBrowser->append("Failed to allocate message buffer.\nDetails: malloc(msg_size) returned: NULL\n");
            return;
        }
        msg.SerializeToArray(msg_buff, msg_size);

        AMQPExchange * ex = m_amqp->createExchange("amq.topic");
        ex->Declare("amq.topic", "topic", AMQP_DURABLE);
        ex->Publish((char*)msg_buff, msg_size, "motorcontrol.excavation.bucket_conveyor_rpm");

        free(msg_buff);
    }
    else
        ui->consoleOutputTextBrowser->append("Not in dig configuration. \n Please go through dig configuration Process");

}

void MainWindow::regularExcavationConveyer(bool checked) {
    if(m_digConfig == 1) {
        handleExcavationConveyor(checked);
    }
    else
        ui->consoleOutputTextBrowser->append("Not in dig configuration. \n Please go through dig configuration Process");

}

void MainWindow::handleA_KeyPress() {
    if(m_desiredConfig == 0) { //straight
        handleTankPivotLK();
    }
    else                       //turn or strafe
        handleLocomotionLeft();
}

void MainWindow::handleD_KeyPress() {
    if(m_desiredConfig == 0) { //straight
        handleTankPivotRK();
    }
    else                       //turn or strafe
        handleLocomotionRight();
}
