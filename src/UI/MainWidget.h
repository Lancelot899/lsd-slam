#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QTableWidget>

#include "PointCloudWidget.h"



class MainWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MainWidget(QWidget *parent = 0);

signals:

public slots:
    void stBtnCall();


private:
    //! show item
    PointCloudWidget   *PCWidget;
    QPushButton        *stButton;
    QPushButton        *edButton;
    QPushButton        *psButton;
    QTableWidget       *infoTable;

private:
    //! control item
    bool isRunning;
};

#endif // MAINWIDGET_H
