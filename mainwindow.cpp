#include <QIcon>
#include <QMessageBox>
#include <QFileDialog>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Create controller
    controller = new Controller;
    connectSignals();
    // Initialize flag
    isCameraConnected=false;
    // disable the play button
    ui->playButton->setEnabled(false);
    // disable the menu option for selecting target
    ui->actionSelect_target->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete controller;
    delete ui;
}

void MainWindow::connectSignals()
{
    connect(controller->captureThread, SIGNAL(videoFinished()), this, SLOT(on_playButton_clicked()));
    connect(ui->videoLabel, SIGNAL(roiSelected(QRect, QPoint)), controller->processingThread, SLOT(setTarget(QRect, QPoint)));
}

void MainWindow::on_playButton_clicked()
{
    if (!controller->captureThread->isPaused())
    { // pause it
        controller->pause();
        ui->playButton->setIcon(QIcon(":/playIcon"));
    }
    else
    { // play
        controller->play();
        ui->playButton->setIcon(QIcon(":/pauseIcon"));
    }
}

void MainWindow::on_actionLoad_Video_triggered()
{
    QString filename = QFileDialog::getOpenFileName(
            this,
            tr("Select Video to Open"),
            QDir::toNativeSeparators(QDir::homePath()),
            tr("Videos (*.avi)"));

    if (!filename.isEmpty())
    {
        if (controller->readVideo(filename))
        {
            this->startProcessing();
        }
        else
        {
            statusBar()->showMessage(tr("ERROR - Could not load ") + filename);
        }
    }
    else
    {
        statusBar()->showMessage(tr("ERROR - You have to select a file "));
    }
}

void MainWindow::on_actionConnect_to_camera_triggered()
{
    // Connect to camera
    if((isCameraConnected = controller->connectToCamera()))
    {
        this->startProcessing();
    }
    else
    {
        QMessageBox::warning(this,"ERROR:","Could not connect to camera.");
    }
}

void MainWindow::startProcessing()
{
    connect(controller->processingThread, SIGNAL(newProcessedFrame(QImage)), this, SLOT(updateVideoLabel(QImage)));
    connect(controller->processingThread->tracker_, SIGNAL(newCorrelationImage(QImage)), this, SLOT(updateOutputLabel(QImage)));
    connect(controller->processingThread->tracker_, SIGNAL(newFilterImage(QImage)), this, SLOT(updateFilterLabel(QImage)));
    connect(controller->processingThread->tracker_, SIGNAL(newInputImage(QImage)), this, SLOT(updateInputLabel(QImage)));

    // enable play btn and set the icon to pause
    ui->playButton->setEnabled(true);
    ui->playButton->setIcon(QIcon(":/pauseIcon"));
    // enable the menu option for selecting the target
    ui->actionSelect_target->setEnabled(true);
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::information(this,"About",QString("Created by Jeremie DERAY and Alberto QUINTERO DELGADO\nMaster Students - 2013"));
}

void MainWindow::updateVideoLabel(QImage img)
{
    // Display input image in inputlabel
    ui->videoLabel->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::updateOutputLabel(QImage img)
{
    // Display correlation image in outputlabel
    ui->outputLabel->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::updateFilterLabel(QImage img)
{
    // Display filter image in outputlabel
    ui->filterLabel->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::updateInputLabel(QImage img)
{
    // Display filter image in outputlabel
    ui->inputLabel->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::on_actionSelect_target_triggered()
{
    // Pause the video if it is on going
    if (!controller->captureThread->isPaused())
    {
//        on_playButton_clicked();
    }

    ui->videoLabel->selectROI();
}

void MainWindow::resizeEvent(QResizeEvent *ev)
{
    controller->captureThread->setSize(ui->videoLabel->size());
}
