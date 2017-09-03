#ifndef DRILLSLIDER_H
#define DRILLSLIDER_H

#include <QAbstractItemModel>
#include <QSlider>
#include <qslider.h>

class DrillSlider : public QSlider
{
    Q_OBJECT

public:
    DrillSlider(QWidget *parent = Q_NULLPTR);
    //DrillSlider(qreal contents, QWidget *parent = Q_NULLPTR);
    //DrillSlider(qreal contents, qreal min, qreal max, int precision, QWidget *parent = Q_NULLPTR);
signals:
    void valueChanged(double value);
    //void valueEdited(qreal value); valued only slides
public slots:
    void changedValue(double value);
private slots:
    void handleValueChanged(double value);
    void handleSliderValue(int value);
    //void handleTextEdited(const QString &text);
private:
    int max = 300.0;
    int min = 0.0;
    double drillValue;
    int slideValue;
};

#endif // DRILLSLIDER_H
