#ifndef IMAGE_DISPLAYER_HPP_
#define IMAGE_DISPLAYER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <QObject>
#include <QtQuick/QQuickPaintedItem>
#include <QTimer>

class ImageDisplayer : public QQuickPaintedItem
{
    Q_OBJECT

public:
    ImageDisplayer();

    ~ImageDisplayer();

    Q_INVOKABLE void initImageDisplayer();

    void paint(QPainter *painter) override;

    void handleIncomingImage(const sensor_msgs::ImageConstPtr &image);

public slots:
    void displayImage();

private:
    QTimer *timer;
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber sub;
    cv_bridge::CvImagePtr cvPtr;

    int displayTimeout = 100;
};

#endif  // IMAGE_DISPLAYER_HPP_
