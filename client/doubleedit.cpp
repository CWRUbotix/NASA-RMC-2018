#include "doubleedit.h"

#include <QLineEdit>
#include <QString>
#include <QDoubleValidator>

DoubleEdit::DoubleEdit(QWidget *parent) :
    DoubleEdit(0, parent)
{
}

DoubleEdit::DoubleEdit(qreal contents, QWidget *parent) :
    QLineEdit(QString::number(contents), parent)
{
    connect(this, &QLineEdit::textChanged, this, &DoubleEdit::handleTextChanged);
    connect(this, &QLineEdit::textEdited, this, &DoubleEdit::handleTextEdited);
    setValidator(new QDoubleValidator());
}

DoubleEdit::DoubleEdit(qreal contents, qreal min, qreal max, int precision, QWidget *parent) :
    DoubleEdit(contents, parent)
{
    setValidator(new QDoubleValidator(min, max, precision));
}

void DoubleEdit::setValue(qreal value) {
    m_value = value;
    emit valueChanged(value);
}

void DoubleEdit::handleTextChanged(const QString &text) {
    m_value = text.toDouble();
    emit valueChanged(m_value);
}

void DoubleEdit::handleTextEdited(const QString &text) {
    m_value = text.toDouble();
    emit valueEdited(m_value);
}
