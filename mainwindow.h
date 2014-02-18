#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include "controller.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void connectSignals();
    void resizeEvent(QResizeEvent *);

private slots:
    void on_playButton_clicked();
    void on_actionLoad_Video_triggered();
    void on_actionConnect_to_camera_triggered();
    void on_actionAbout_triggered();
    void updateVideoLabel(QImage);
    void updateOutputLabel(QImage);
    void updateFilterLabel(QImage);
    void updateInputLabel(QImage);

    void on_actionSelect_target_triggered();

private:
    void startProcessing();

    Ui::MainWindow *ui;
    Controller *controller;
    bool isCameraConnected;
};

#endif // MAINWINDOW_H
