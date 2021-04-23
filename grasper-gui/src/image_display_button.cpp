#include "image_display_button.hpp"

#include <iostream>

#include <QDebug>
#include <grasper_msg/CameraRequestMessage.h>

#include "main_controller.hpp"

ImageDisplayButton::ImageDisplayButton()
{
    m_captureImagePub =
        MainController::getInstance()
            ->getNodeHandle()
            ->advertise<grasper_msg::CameraRequestMessage>("camera/requestCameraFrames", 1000);
}

void ImageDisplayButton::addConnections(QObject *root)
{
    // Connect capture image button to this object
    auto cameraButton = root->findChild<QObject *>("cameraButton");
    QObject::connect(
        cameraButton,
        SIGNAL(clicked()),
        this,
        SLOT(captureImage()),
        Qt::DirectConnection);
}

void ImageDisplayButton::captureImage()
{
    qDebug() << "capture image requested";
    grasper_msg::CameraRequestMessage cameraRequestMessage;
    // Capture an arbitrary number of frames
    cameraRequestMessage.cameraFramesToCapture = 5;
    cameraRequestMessage.cameraFrameCaptureRate = 100;
    // cameraRequestMessage. TODO
    m_captureImagePub.publish(cameraRequestMessage);
}
