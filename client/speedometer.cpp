/****************************************************************************
**
** Copyright (C) 2014 Eric Fesler - ESD Sprl
** All rights reserved.
**
** License Agreement
**
** This program is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
**
**
** Software Author: Eric Fesler <eric.fesler@gmail.com>
**
** Summary: Top class for the speedometer control Widget
**
****************************************************************************/

#include <QtWidgets>
#include "speedometer.h"

Speedometer::Speedometer(QWidget *parent) :
    QWidget(parent)
{
    m_power = 0.0;
    m_speed = 0.0;
    m_displayPowerPath = true;

    // The power gradient
    m_powerGradient =  QConicalGradient(0, 0, 180);
    m_powerGradient.setColorAt(0, Qt::red);
    m_powerGradient.setColorAt(0.375, Qt::yellow);
    m_powerGradient.setColorAt(0.75, Qt::green);
    m_unitTextColor = QColor(Qt::gray);
    m_speedTextColor = QColor(Qt::black);
    m_powerPathColor = QColor(Qt::gray);
}

void Speedometer::setSpeed(double speed)
{
    m_speed=speed;
    update();
}

void Speedometer::setSpeed(int speed)
{
   setSpeed((double)speed);
   update();
}

void Speedometer::setPower(double power)
{
    m_power=power;
    update();
}

void Speedometer::setPower(int power)
{
   setPower((double)power);
   update();
}

void Speedometer::setValues(double speed, double power)
{
    m_speed = speed;
    QPropertyAnimation *animation = new QPropertyAnimation(this, "power");
    animation->setDuration(1000);
    animation->setStartValue(m_power);
    animation->setEndValue(power);

    animation->start();
}

void Speedometer::setUnit(const QString &unit)
{
    m_unit = unit;
}

void Speedometer::setPowerGradient(const QConicalGradient &gradient)
{
    m_powerGradient = gradient;
}

void Speedometer::setDisplayPowerPath(bool displayPowerPath)
{
    m_displayPowerPath = displayPowerPath;
}

void Speedometer::setUnitTextColor(const QColor &color)
{
    m_unitTextColor = color;
}

void Speedometer::setSpeedTextColor(const QColor &color)
{
    m_speedTextColor = color;
}

void Speedometer::setPowerPathColor(const QColor &color)
{
    m_powerPathColor = color;
}


void Speedometer::paintEvent(QPaintEvent *)
{
    // Limits of external circle
    static const QPoint x1(0, -70);
    static const QPoint x2(0, -90);
    static const QPoint x3(-90,0);
    static const QPoint x4(-70,0);
    static const QRectF extRect(-90,-90,180,180);
    static const QRectF intRect(-70,-70,140,140);
    static const QRectF midRect(-80,-80,160,160);
    static const QRectF unitRect(-44,36,86,17);

    // Compute speed Strings
    int speedInt = (int)m_speed;
    int speedDec = (int)(m_speed * 10.0) - (speedInt * 10);
    QString s_SpeedInt = QString::number(speedInt).append('.');
    QString s_SpeedDec = QString::number(speedDec);

    // Compute power angle
    double powerAngle = m_power * 270.0 / 100.0;

     // The power path
    QPainterPath dummyPath;
    dummyPath.moveTo(x1);
    dummyPath.arcMoveTo(intRect, 90 - powerAngle);
    QPainterPath powerPath;
    powerPath.moveTo(x1);
    powerPath.lineTo(x2);
    powerPath.arcTo(extRect, 90, -1 * powerAngle);
    powerPath.lineTo(dummyPath.currentPosition());
    powerPath.arcTo(intRect, 90 - powerAngle, powerAngle);

    // Paint everything
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.translate(width() / 2, height() / 2);
    int side = qMin(width(), height());
    painter.scale(side / 200.0, side / 200.0);

    painter.save();
    painter.rotate(-135);

    // The external path
    if (m_displayPowerPath) {

        QPainterPath externalPath;
        externalPath.moveTo(x1);
        externalPath.lineTo(x2);
        externalPath.arcTo(extRect, 90, -270);
        externalPath.lineTo(x4);
        externalPath.arcTo(intRect, 180, 270);

        painter.setPen(m_powerPathColor);
        painter.drawPath(externalPath);
    }

    painter.setBrush(m_powerGradient);
    painter.setPen(Qt::NoPen);
    painter.drawPath(powerPath);
    painter.restore();

    // Draw unit
    painter.setPen(m_unitTextColor);
    QString fontFamily = this->font().family();
    QFont unitFont(fontFamily, 18);
    painter.setFont(unitFont);
    painter.drawText(unitRect, Qt::AlignCenter, m_unit);

    // Draw Speed
    QColor speedColor(0,0,0);
    QFont speedFont(fontFamily, 48);
    QFontMetrics fm1(speedFont);
    int speedWidth = fm1.width(s_SpeedInt);

    QFont speedDecFont(fontFamily, 23);
    QFontMetrics fm2(speedDecFont);
    int speedDecWidth = fm2.width(s_SpeedDec);

    int leftPos = -1 * (speedWidth + speedDecWidth) / 2.0 ;
    int leftDecPos = leftPos + speedWidth ;
    int topPos = 10;
    int topDecPos = 10;
    painter.setPen(m_speedTextColor);
    painter.setFont(speedFont);
    painter.drawText(leftPos, topPos, s_SpeedInt);
    painter.setFont(speedDecFont);
    painter.drawText(leftDecPos, topDecPos, s_SpeedDec);

}

