#ifndef INTEDIT_H
#define INTEDIT_H

#include <QObject>
#include <QWidget>
#include <QLineEdit>
#include <QString>

class IntEdit : public QLineEdit {
    Q_OBJECT
public:
    IntEdit(QWidget *parent = Q_NULLPTR);
    IntEdit(int contents, QWidget *parent = Q_NULLPTR);
    IntEdit(int contents, int min, int max, QWidget *parent = Q_NULLPTR);
signals:
    void valueChanged(int value);
    void valueEdited(int value);
public slots:
    void setValue(int value);
private slots:
    void handleTextChanged(const QString &text);
    void handleTextEdited(const QString &text);
private:
    int m_value;
};

#endif // INTEDIT_H
