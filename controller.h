#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "capturethread.h"
#include "imagebuffer.h"
#include "processingthread.h"
#include <QtGui>
#include <opencv/highgui.h>


class Controller : public QObject
{
    Q_OBJECT

public:
    Controller();
    ~Controller();

    ImageBuffer      *imageBuffer;
    ProcessingThread *processingThread;
    CaptureThread    *captureThread;

    bool connectToCamera();
    void stopThreads();
    void clearImageBuffers();
    bool readVideo(QString);
    void pause();
    void play();

public slots:
    void setTarget(QRect, QPoint);

signals:
    void newInputFrame(QImage);

private:
    int imageBufferSize;
    QRect target_;
    QPoint targetOrigen_;
};

#endif // CONTROLLER_H
