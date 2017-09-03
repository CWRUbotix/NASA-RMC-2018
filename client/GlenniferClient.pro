#-------------------------------------------------
#
# Project created by QtCreator 2016-10-17T21:21:14
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = GlenniferClient
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    connectiondialog.cpp \
    messages.pb.cc \
    consumerthread.cpp \
    speedometer.cpp \
    cameraone.cpp \
    doubleedit.cpp \
    intedit.cpp \
    drillslider.cpp \
    ledindicator.cpp

HEADERS  += mainwindow.h \
    connectiondialog.h \
    messages.pb.h \
    consumerthread.h \
    speedometer.h \
    cameraone.h \
    doubleedit.h \
    intedit.h \
    drillslider.h \
    ledindicator.h

FORMS    += mainwindow.ui \
    connectiondialog.ui \
    cameraone.ui

CONFIG += conan_basic_setup
include(conanbuildinfo.pri)

unix: LIBS += -L/usr/local/lib/ -lprotobuf
