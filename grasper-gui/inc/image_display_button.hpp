#ifndef IMAGE_DISPLAY_BUTTON_HPP_
#define IMAGE_DISPLAY_BUTTON_HPP_

#include <QObject>
#include <ros/ros.h>

class ImageDisplayButton : public QObject
{
    Q_OBJECT

public:
    ImageDisplayButton();
    virtual ~ImageDisplayButton() = default;

    void addConnections(QObject *root);

public slots:
    void captureImage();

private:
    ros::Publisher m_captureImagePub;
};

#endif  // IMAGE_DISPLAY_BUTTON_HPP_
