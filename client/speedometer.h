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
** Summary: Header for the speedometer control Widget
**
****************************************************************************/

#ifndef SPEEDOMETER_H
#define SPEEDOMETER_H

#include <QWidget>
#include <QtDesigner/QDesignerExportWidget>

class Speedometer : public QWidget
{
    Q_OBJECT
       Q_PROPERTY(double speed READ speed WRITE setSpeed);
       Q_PROPERTY(double power READ power WRITE setPower);
       Q_PROPERTY(QString unit READ unit WRITE setUnit);
       Q_PROPERTY(QConicalGradient powerGrandient READ powerGradient WRITE setPowerGradient);
       Q_PROPERTY(bool displayPowerPath READ displayPowerPath WRITE setDisplayPowerPath );
       Q_PROPERTY(QColor unitTextColor READ unitTextColor WRITE setUnitTextColor );
       Q_PROPERTY(QColor speedTextColor READ speedTextColor WRITE setSpeedTextColor );
       Q_PROPERTY(QColor powerPathColor READ powerPathColor WRITE setPowerPathColor );

public:
    explicit Speedometer(QWidget *parent = 0);

    /*!
      \return The current displayed value
    */
    double speed() const { return m_speed; }
    /*!
      \return The current displayed power
    */
    double power() const { return m_power; }

    QString unit() const { return m_unit; }

    void setUnit(const QString &unit);

    QConicalGradient powerGradient() const { return m_powerGradient; }

    void setPowerGradient(const QConicalGradient &gradient);

    bool displayPowerPath() const { return m_displayPowerPath; }

    void setDisplayPowerPath(bool displayPowerPath);

    QColor unitTextColor() const { return m_unitTextColor; }

    void setUnitTextColor(const QColor &color);

    QColor speedTextColor() const { return m_speedTextColor; }

    void setSpeedTextColor(const QColor &color);

    QColor powerPathColor() const { return m_powerPathColor; }

    void setPowerPathColor(const QColor &color);



public slots:
    /*!
      \slots This slot is used to set Speedometer speed
    */
    void setSpeed(double);
    /*!
      \slots This is an overloaded member function, provided for convenience.
    */
    void setSpeed(int);
    /*!
      \slots This slot is used to set Speedometer power in percengtage
    */
    void setPower(double);
    /*!
      \slots This is an overloaded member function, provided for convenience.
    */
    void setPower(int);
    /*!
     * \brief sets the speed and the power. The power will be set through a smooth animation
     * \param speed the speed to set
     * \param power the power to set
     */
    void setValues(double speed, double power);



protected:
    double m_speed;
    double m_power;
    QString m_unit;
    QConicalGradient m_powerGradient;
    bool m_displayPowerPath;
    QColor m_unitTextColor;
    QColor m_speedTextColor;
    QColor m_powerPathColor;

    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
};

#endif // SPEEDOMETER_H
