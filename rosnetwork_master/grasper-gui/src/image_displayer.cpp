#include "image_displayer.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <QImage>
#include <QPixmap>
#include <QPicture>
#include <QPainter>

using namespace cv;

ImageDisplayer::ImageDisplayer()
    : imageTransport(nodeHandle) {}

ImageDisplayer::~ImageDisplayer()
{
}

void ImageDisplayer::initImageDisplayer()
{
    sub = imageTransport.subscribe("camera/cameraFrames",
                                   10,
                                   &ImageDisplayer::handleIncomingImage,
                                   this);

    timer = new QTimer();
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(displayTimeout);
}

void ImageDisplayer::handleIncomingImage(const sensor_msgs::ImageConstPtr &image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        qDebug() << "cv_bridge exception: " << e.what();
        return;
    }
}

void ImageDisplayer::paint(QPainter *painter)
{
    if (cvPtr == nullptr)
    {
        qDebug() << "cvPtr is nullptr";
        return;
    }
    Mat img = cvPtr->image;
    cv::resize(img, img, Size(width(), height()), 0, 0, INTER_CUBIC);
    QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
    imdisplay = imdisplay.rgbSwapped();
    painter->drawPixmap(0, 0, QPixmap::fromImage(imdisplay));
}

void ImageDisplayer::displayImage()
{
}
