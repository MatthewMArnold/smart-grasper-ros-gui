#ifndef IMAGE_DISPLAYER_HPP_
#define IMAGE_DISPLAYER_HPP_

#include <QObject>
#include <QTimer>
#include <QtQuick/QQuickPaintedItem>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "error_reporter.hpp"

class ImageDisplayer : public QQuickPaintedItem, public ErrorReporter
{
    Q_OBJECT

public:
    ImageDisplayer();

    ~ImageDisplayer();

    Q_INVOKABLE void initImageDisplayer();

    void paint(QPainter *painter) override;

    void handleIncomingImage(const sensor_msgs::ImageConstPtr &image);

    void errorCleared(ErrorController::ErrorType type) override;

public slots:
    void imageNodeDisconnected();
    void setImageNodeConnected(bool connected);

signals:
    void imageNodeMsgReceived();

private:
    static constexpr int IMAGE_DISPLAY_FREQUENCY = 100;
    static constexpr int IMAGE_NODE_ERROR_TIMEOUT = 1000;

    QTimer *m_imageDisplayTimer;

    ros::NodeHandle m_nodeHandle;
    image_transport::ImageTransport m_imageTransport;
    image_transport::Subscriber m_imageTransportSub;
    cv_bridge::CvImagePtr m_cvPtr;

    bool m_imageNodeRunning = true;
    QTimer *m_imageNodeRunningTimeout;
};

#endif  // IMAGE_DISPLAYER_HPP_
