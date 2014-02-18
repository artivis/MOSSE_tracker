#include "imagebuffer.h"

ImageBuffer::ImageBuffer(QObject *parent, int s, bool fd)
    : QObject(parent)
    , bufferSize(s)
    , dropFrame(fd)
{
    // Semaphore initializations
    freeSlots    = new QSemaphore(bufferSize);
    usedSlots    = new QSemaphore(0);
    clearBuffer1 = new QSemaphore(1);
    clearBuffer2 = new QSemaphore(1);
}

CImage ImageBuffer::getFrame()
{
    // Acquire semaphores
    clearBuffer2->acquire();
    usedSlots->acquire();
    // Temporary data
//    CImage tempFrame(cv::Mat());
    // Take frame from queue
    imageQueueProtect.lock();
        CImage tempFrame = imageQueue.dequeue();
    imageQueueProtect.unlock();

    // Release semaphores
    freeSlots->release();
    clearBuffer2->release();
    // Return frame to caller
    return tempFrame;
}

void ImageBuffer::addFrame(const CImage &frame)
{
    // Acquire semaphore
    clearBuffer1->acquire();
    // If frame dropping is enabled, do not block if buffer is full
    if(dropFrame)
    {
        // Try and acquire semaphore to add frame
        if(freeSlots->tryAcquire())
        {
            // Add frame to queue
            imageQueueProtect.lock();
                imageQueue.enqueue(frame);
            imageQueueProtect.unlock();
            // Release semaphore
            usedSlots->release();
        }
    }
    // If buffer is full, wait on semaphore
    else
    {
        // Acquire semaphore
        freeSlots->acquire();
        // Add frame to queue
        imageQueueProtect.lock();
            imageQueue.enqueue(frame);
        imageQueueProtect.unlock();
        // Release semaphore
        usedSlots->release();
    }
    // Release semaphore
    clearBuffer1->release();
    emit newFrame();
}

void ImageBuffer::clearBuffer()
{
    // Check if buffer is not empty
    if(imageQueue.size()!=0)
    {
        // Stop adding frames to buffer
        clearBuffer1->acquire();
        // Stop taking frames from buffer
        clearBuffer2->acquire();
        // Release all remaining slots in queue
        freeSlots->release(imageQueue.size());
        // Acquire all queue slots
        freeSlots->acquire(bufferSize);
        // Reset usedSlots to zero
        usedSlots->acquire(imageQueue.size());
        // Clear buffer
        imageQueue.clear();
        // Release all slots
        freeSlots->release(bufferSize);
        // Allow getFrame() to resume
        clearBuffer2->release();
        // Allow addFrame() to resume
        clearBuffer1->release();
        qDebug() << "Image buffer successfully cleared.";
    }
    else
        qDebug() << "WARNING: Could not clear image buffer: already empty.";
}

int ImageBuffer::getSizeOfImageBuffer()
{
    return imageQueue.size();
}
