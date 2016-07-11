

#include <QGridLayout>
#include <QDebug>
#include <QFileDialog>

#include "MainWidget.h"

MainWidget::MainWidget(QWidget *parent) :
    QWidget(parent)
{
    this->setFixedSize(1600, 960);
    PCWidget = new PointCloudWidget;
    stButton = new QPushButton;
    edButton = new QPushButton;
    psButton = new QPushButton;
    infoTable = new QTableWidget;

    QGridLayout *mainLayout = new QGridLayout;
    this->setLayout(mainLayout);

    mainLayout->addWidget(PCWidget, 0, 0, 4, 7);
    mainLayout->addWidget(infoTable, 0, 7, 3, 3);
    mainLayout->addWidget(stButton, 3, 7, 1, 1);
    mainLayout->addWidget(edButton, 3, 8, 1, 1);
    mainLayout->addWidget(psButton, 3, 9, 1, 1);

    stButton->setText("start");
    edButton->setText("end");
    psButton->setText("pause");

    connect(stButton, SIGNAL(clicked()), this, SLOT(stBtnCall()));
}

void MainWidget::stBtnCall()
{
    QString file = QFileDialog::getOpenFileName(
                        this, "please input a video", QString(),
                        "video files(*.rm *.rmvb *.wmv *.avi *.mp4 *.3gp *.mkv)");
    isRunning = true;
}


