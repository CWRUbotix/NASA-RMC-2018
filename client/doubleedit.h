#ifndef DOUBLEEDIT_H
#define DOUBLEEDIT_H

#include <QObject>
#include <QWidget>
#include <QLineEdit>
#include <QString>

class DoubleEdit : public QLineEdit {
    Q_OBJECT
public:
    DoubleEdit(QWidget *parent = Q_NULLPTR);
    DoubleEdit(qreal contents, QWidget *parent = Q_NULLPTR);
    DoubleEdit(qreal contents, qreal min, qreal max, int precision, QWidget *parent = Q_NULLPTR);
signals:
    void valueChanged(qreal value);
    void valueEdited(qreal value);
public slots:
    void setValue(qreal value);
private slots:
    void handleTextChanged(const QString &text);
    void handleTextEdited(const QString &text);
private:
    qreal m_value;
};

#endif // DOUBLEEDIT_H
