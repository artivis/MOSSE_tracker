#include <QDebug>
#include "capturethread.h"
#include "imagebuffer.h"
#include "cimage.h"
#include "config.h"

CaptureThread::CaptureThread(ImageBuffer *imageBuffer)
    : QThread()
    , inputBuffer(imageBuffer)
    , stopped(false)
    , paused(false)
    , currentImage(cv::Mat())
{
}

CaptureThread::~CaptureThread()
{
    if(cap.isOpened())
    {
        cap.release();
    }
}

void CaptureThread::run()
{
    while(1)
    {
        // Check if it is paused
        pauseMutex.lock();
        if (paused)
        {
            pauseMutex.unlock();
            sleep(3); // to avoid consuming CPU
            continue;
        }
        pauseMutex.unlock();

        /////////////////////////////////
        // Stop thread if stopped=TRUE //
        /////////////////////////////////
        stoppedMutex.lock();
        if (stopped)
        {
            stopped = false;
            stoppedMutex.unlock();
            break;
        }
        stoppedMutex.unlock();


        // Capture a frame
        capMutex.lock();
            cap >> grabbedFrame;
        capMutex.unlock();
        // resize the frame to fit in the UI frames
        cv::resize(grabbedFrame, grabbedFrame, cv::Size(toResize.width(), toResize.height()));
        // flip the image
        cv::flip(grabbedFrame, grabbedFrame, 1);

        // Create the CImage object
        currentImage = CImage(grabbedFrame.clone());

        // add the frame to the buffer
        inputBuffer->addFrame(currentImage);

        // if the video reached the end just repeat it
        inputMutex.lock();
        if (inputMode == INPUT_VIDEO)
        {
            inputMutex.unlock();
            capMutex.lock();
            if (cap.get(CV_CAP_PROP_FRAME_COUNT)-1 == cap.get(CV_CAP_PROP_POS_FRAMES))
            {
                cap.set(CV_CAP_PROP_POS_FRAMES, 0);
                emit videoFinished();
            }
            capMutex.unlock();
            msleep(1000/cap.get(CV_CAP_PROP_FPS));
        }
        else if (inputMode == INPUT_CAMERA)
        {
            inputMutex.unlock();
            msleep(35);
        }
        inputMutex.unlock();
    }
}

void CaptureThread::stopCaptureThread()
{
    stoppedMutex.lock();
    stopped=true;
    stoppedMutex.unlock();
}

bool CaptureThread::readVideo(QString fn)
{
    bool res = false;
    setInputMode(INPUT_VIDEO);
    capMutex.lock();
    if (cap.isOpened())
        cap.release();
        res = cap.open(fn.toStdString());
    capMutex.unlock();

    return res;
}

bool CaptureThread::connectToCamera(int c)
{
    bool res = false;
    setInputMode(INPUT_CAMERA);
    capMutex.lock();
    if (cap.isOpened())
        cap.release();
        res = cap.open(c);
    capMutex.unlock();

    return res;
}
