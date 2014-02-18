#include <QDebug>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "imagebuffer.h"
#include "processingthread.h"

ProcessingThread::ProcessingThread(ImageBuffer *imageBuffer)
    : QThread()
    , outputBuffer(imageBuffer)
    , stopped(false)
    , paused(false)
    , targetSetted_(false)
{
    tracker_ = new Tracker();
}

ProcessingThread::~ProcessingThread()
{
}

void ProcessingThread::run()
{
    while(1)
    {
        // Check if paused
        pauseMutex.lock();
        if (paused)
        {
            pauseMutex.unlock();
            sleep(3);
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
        /////////////////////////////////


        if (outputBuffer->getSizeOfImageBuffer() > 0)
        {
            currentFrame = outputBuffer->getFrame();
        } else continue;

        cv::Mat temp;
        currentFrame.getMat().copyTo(temp);

        if (targetSetted_)
        {
            // if the tracker is not initialized, then do it
            if (!tracker_->isInitialized())
            {
                tracker_->InitTracker(temp,
                                     cv::Rect(targetOrigin_.x(), targetOrigin_.y(), targetRect_.width(), targetRect_.height()));
            }

            /*
             * Do the tracking and then get the new position of the target to draw it
             */
            tracker_->Track(temp);
            cv::Rect targetPos = tracker_->getPosition();

            if (targetPos.x > 0 && targetPos.y > 0)
            { // It was found so, update the previous position
                // update the targetOrigin_ and targetRect_
                targetOrigin_.setX(targetPos.x);
                targetOrigin_.setY(targetPos.y);
                targetRect_.setWidth(targetPos.width);
                targetRect_.setHeight(targetPos.height);
            }

            // Set state of target
            int state = tracker_->getState();

            if (state == FOUND)
            {// draw blue rectangle
                cv::rectangle(temp,
                              cv::Rect(targetOrigin_.x(), targetOrigin_.y(), targetRect_.width(), targetRect_.height()),
                              cv::Scalar(255, 0, 0), 2);

            } else
            {// draw red rectangle with X
                cv::rectangle(temp,
                              cv::Rect(targetOrigin_.x(), targetOrigin_.y(), targetRect_.width(), targetRect_.height()),
                              cv::Scalar(0, 0, 255), 2);
                cv::line(temp,
                         cv::Point(targetOrigin_.x(), targetOrigin_.y()),
                         cv::Point(targetOrigin_.x()+targetRect_.width(), targetOrigin_.y()+targetRect_.height()),
                         cv::Scalar(0, 0, 255), 2);
                cv::line(temp,
                         cv::Point(targetOrigin_.x()+targetRect_.width(), targetOrigin_.y()),
                         cv::Point(targetOrigin_.x(), targetOrigin_.y()+targetRect_.height()),
                         cv::Scalar(0, 0, 255), 2);

            }

        }

        msleep(35);
        // Inform GUI thread of new frame (QImage)
        emit newProcessedFrame(CImage::getQImage(temp));
    }
}

void ProcessingThread::stopProcessingThread()
{
    stoppedMutex.lock();
    stopped = true;
    stoppedMutex.unlock();
}


int ProcessingThread::getCurrentSizeOfBuffer()
{
    return outputBuffer->getSizeOfImageBuffer();
}
