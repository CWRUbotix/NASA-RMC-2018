#include "drillslider.h"

DrillSlider::DrillSlider(QWidget *parent)
    : QSlider(parent)
{
    connect(this, &QSlider::valueChanged, this, &DrillSlider::handleValueChanged);
}

void DrillSlider::changedValue(double value) {
    handleValueChanged(value);
}

//Other examples had a setValue that emitted the parameter but I don't think this will need it

void DrillSlider::handleSliderValue(int value) {
    slideValue = value;
    drillValue = (double)(value/10);
    emit valueChanged(drillValue);
}

void DrillSlider::handleValueChanged(double value) {
    slideValue = (int)(10*(value));
    drillValue = value;
    emit valueChanged(drillValue);
}
