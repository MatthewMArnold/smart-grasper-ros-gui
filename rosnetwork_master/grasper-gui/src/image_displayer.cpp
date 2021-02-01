#include "image_displayer.hpp"

#include <QImage>
#include <QPainter>
#include <QPicture>
#include <QPixmap>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

#include "error_controller.hpp"

using namespace cv;

ImageDisplayer::ImageDisplayer() : m_imageTransport(m_nodeHandle) {}

ImageDisplayer::~ImageDisplayer() {}

void ImageDisplayer::initImageDisplayer()
{
    m_imageTransportSub = m_imageTransport.subscribe(
        "camera/cameraFrames",
        10,
        &ImageDisplayer::handleIncomingImage,
        this);

    m_imageDisplayTimer = new QTimer();
    QObject::connect(
        m_imageDisplayTimer,
        SIGNAL(timeout()),
        this,
        SLOT(update()),
        Qt::QueuedConnection);
    m_imageDisplayTimer->start(IMAGE_DISPLAY_FREQUENCY);

    m_imageNodeRunningTimeout = new QTimer();
    QObject::connect(
        m_imageNodeRunningTimeout,
        SIGNAL(timeout()),
        this,
        SLOT(imageNodeDisconnected()));
    QObject::connect(
        this,
        SIGNAL(imageNodeMsgReceived()),
        m_imageNodeRunningTimeout,
        SLOT(start()),
        Qt::QueuedConnection);
    m_imageNodeRunningTimeout->start(IMAGE_NODE_ERROR_TIMEOUT);
}

void ImageDisplayer::handleIncomingImage(const sensor_msgs::ImageConstPtr &image)
{
    //    qDebug() << "new image received";
    cv_bridge::CvImagePtr cv_ptr;
    emit imageNodeMsgReceived();
    setImageNodeConnected(true);
    try
    {
        m_cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        qDebug() << "cv_bridge exception: " << e.what();
        return;
    }
}

void ImageDisplayer::imageNodeDisconnected() { setImageNodeConnected(false); }

void ImageDisplayer::errorCleared(ErrorController::ErrorType type) { Q_UNUSED(type); }

void ImageDisplayer::paint(QPainter *painter)
{
    if (m_cvPtr == nullptr)
    {
        return;
    }
    Mat img = m_cvPtr->image;
    cv::resize(img, img, Size(width(), height()), 0, 0, INTER_CUBIC);
    QImage imdisplay((uchar *)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
    imdisplay = imdisplay.rgbSwapped();
    painter->drawPixmap(0, 0, QPixmap::fromImage(imdisplay));
}

void ImageDisplayer::setImageNodeConnected(bool connected)
{
    if (m_imageNodeRunning != connected)
    {
        m_imageNodeRunning = connected;
        if (m_imageNodeRunning)
        {
            qDebug() << "remove error";
            ErrorController::getInstance()->removeError(
                ErrorController::ErrorType::CAMERA_NODE_NOT_RUNNING);
        }
        else
        {
            ErrorController::getInstance()->addError(
                this,
                ErrorController::ErrorType::CAMERA_NODE_NOT_RUNNING);
        }
    }
}
