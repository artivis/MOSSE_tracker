#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include <QThread>
#include <QtGui>
#include "opencv/highgui.h"
#include "imagebuffer.h"

class CaptureThread : public QThread
{
    Q_OBJECT

public:
    CaptureThread(ImageBuffer *buffer);
    ~CaptureThread();

    void setInputMode(int m)    { QMutexLocker locker(&inputMutex); inputMode = m; }
    bool isCameraConnected()    { return cap.isOpened(); }
    int  getInputSourceWidth()  { return cap.get(CV_CAP_PROP_FRAME_WIDTH); }
    int  getInputSourceHeight() { return cap.get(CV_CAP_PROP_FRAME_HEIGHT); }
    void pause()                { QMutexLocker locker(&pauseMutex); paused = true; }
    void play()                 { QMutexLocker locker(&pauseMutex); paused = false; }
    bool isPaused()             { return paused; }
    void setSize(QSize size_)   { toResize = size_; }

    bool readVideo(QString fn);
    bool connectToCamera(int c);
    void stopCaptureThread();

private:
    ImageBuffer *inputBuffer;
    cv::VideoCapture cap;
    cv::Mat grabbedFrame;
    CImage currentImage;
    QMutex stoppedMutex;
    QMutex capMutex;
    QMutex inputMutex;
    QMutex pauseMutex;
    int inputMode;
    QSize toResize;

    volatile bool stopped;
    bool paused;

protected:
    void run();

signals:
    void videoFinished();
};

#endif // CAPTURETHREAD_H
