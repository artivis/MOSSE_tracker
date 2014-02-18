#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H
#include <QThread>
#include <QtGui>
#include <opencv/highgui.h>
#include "imagebuffer.h"
#include "cimage.h"
#include "tracker.h"

class ProcessingThread : public QThread
{
    Q_OBJECT

public:
    ProcessingThread(ImageBuffer *imageBuffer);
    ~ProcessingThread();

    void stopProcessingThread();
    int  getCurrentSizeOfBuffer();

    void setCurrentImage(const CImage &frame) { currentFrame = frame; }
    CImage getProcessedFrame()   const        { return processedFrame; }

    void pause()                              { QMutexLocker locker(&pauseMutex); paused = true; }
    void play()                               { QMutexLocker locker(&pauseMutex); paused = false; }
    bool isPaused()                           { return paused; }

    Tracker       *tracker_;

public slots:
    void setTarget(QRect rect, QPoint zeroP)  { targetRect_ = rect;
                                                targetOrigin_ = zeroP;
                                                targetSetted_ = true; }

private:
    ImageBuffer   *outputBuffer;
    volatile bool stopped;
    bool          paused;
    int           currentSizeOfBuffer;

    QMutex        stoppedMutex;
    QMutex        pauseMutex;

    CImage        currentFrame;
    CImage        processedFrame;

    bool          targetSetted_;
    QRect         targetRect_;
    QPoint        targetOrigin_;


protected:
    void run();

signals:
    void newProcessedFrame(const QImage &frame);
};

#endif // PROCESSINGTHREAD_H
