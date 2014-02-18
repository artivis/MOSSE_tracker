#include "controller.h"
#include "imagebuffer.h"
#include <QtGui>

Controller::Controller()
    : imageBufferSize(1)
{
    imageBuffer = new ImageBuffer(this, imageBufferSize);

    captureThread    = new CaptureThread(imageBuffer);
    processingThread = new ProcessingThread(imageBuffer);

    target_ = QRect();
    targetOrigen_ = QPoint();
}

Controller::~Controller()
{
    stopThreads();
    delete captureThread;
    delete processingThread;

    imageBuffer->clearBuffer();
    delete imageBuffer;
}

bool Controller::readVideo(QString filename)
{
    bool res = false;

    if (captureThread->isRunning())
    {
        captureThread->pause();
        processingThread->pause();
    }

    if ((res = captureThread->readVideo(filename)))
    {
        if (captureThread->isPaused())
        {
            captureThread->play();
            processingThread->play();
        }
        else
        {
            captureThread->start(QThread::HighPriority);
            processingThread->start(QThread::HighPriority);
        }
    }

    return res;
}

bool Controller::connectToCamera()
{
    bool isOpened    = false;

    if (captureThread->isRunning())
    {
        captureThread->pause();
        processingThread->pause();
    }

    if((isOpened = captureThread->connectToCamera(0)))
    {
        if (captureThread->isPaused())
        {
            captureThread->play();
            processingThread->play();
        }
        else
        {
            captureThread->start(QThread::HighPriority);
            processingThread->start(QThread::HighPriority);
        }
    }

    return isOpened;
}


void Controller::stopThreads()
{
    captureThread->stopCaptureThread();

    // Take one frame off a FULL queue to allow the capture thread to finish
    if(imageBuffer->getSizeOfImageBuffer() == imageBufferSize)
        imageBuffer->getFrame();

    if (!captureThread->wait(2000))
    {
        captureThread->terminate();
    }

    processingThread->stopProcessingThread();
    if (!processingThread->wait(2000))
    {
        processingThread->terminate();
    }
}

void Controller::clearImageBuffers()
{
    imageBuffer->clearBuffer();
}

void Controller::setTarget(QRect roi, QPoint origen)
{
    target_ = roi;
    targetOrigen_ = origen;
}

void Controller::play()
{
    captureThread->play();
    processingThread->play();
}

void Controller::pause()
{
    captureThread->pause();
    processingThread->pause();
}
