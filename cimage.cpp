#include "cimage.h"

CImage::CImage(const cv::Mat& mat)
{
    image_ = cv::Mat(mat);
}

CImage::CImage()
{
    image_ = cv::Mat();
}

CImage::~CImage()
{
}

QImage CImage::getQImage() const
{
    if (image_.empty()) return QImage();

    std::vector<uchar> data;
    cv::imencode("*.pbm", image_, data);

    QImage img;
    if (! img.loadFromData(data.data(), data.size()))
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }

    return img;
}

QImage CImage::getQImage(const cv::Mat &mat)
{
    return CImage(mat).getQImage();
}
