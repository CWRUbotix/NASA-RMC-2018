#include "intedit.h"

#include <QLineEdit>
#include <QString>
#include <QIntValidator>

IntEdit::IntEdit(QWidget *parent) :
    IntEdit(0, parent)
{

}

IntEdit::IntEdit(int contents, QWidget *parent) :
    QLineEdit(QString::number(contents), parent)
{
    connect(this, &QLineEdit::textChanged, this, &IntEdit::handleTextChanged);
    connect(this, &QLineEdit::textEdited, this, &IntEdit::handleTextEdited);
    setValidator(new QIntValidator());
}

IntEdit::IntEdit(int contents, int min, int max, QWidget *parent) :
    IntEdit(contents, parent)
{
    setValidator(new QIntValidator(min, max));
}

void IntEdit::setValue(int value) {
    m_value = value;
    emit valueChanged(value);
}

void IntEdit::handleTextChanged(const QString &text) {
    m_value = text.toInt();
    emit valueChanged(m_value);
}

void IntEdit::handleTextEdited(const QString &text) {
    m_value = text.toInt();
    emit valueEdited(m_value);
}
