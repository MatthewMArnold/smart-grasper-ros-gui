#include "force_controller_worker.hpp"

#include <sstream>

#include <QDebug>
#include <grasper_msg/MotorRequestMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "main_controller.hpp"

void ForceControllerWorker::msgCallback(
    const grasper_msg::MotorMessageFeedback &msg)
{
    setForceActual(msg.appliedForce);
}

void ForceControllerWorker::setForceDesired(double force)
{
    qDebug() << "force desired set to: " << force;
    if (m_forceDesired != force)
    {
        m_requestMutex.lock();
        m_forceDesired = force;
        bool squeeze = m_squeeze;
        bool measureForceRequest = m_measureForceRequest;
        m_requestMutex.unlock();
        sendMotorRequest(force, squeeze, measureForceRequest);
        emit onForceDesiredChanged(force);
    }
}

void ForceControllerWorker::setForceActual(double force)
{
    qDebug() << "actual force received: " << force;
    if (force != m_forceActual)
    {
        m_forceActual = force;
        if (m_measureForceRequest)
        {
            MainController::getInstance()->getRoot()->setProperty(
                "actualForce",
                QVariant(force));
            emit onForceActualChanged(m_forceActual);
        }
    }
}

void ForceControllerWorker::setSqueeze(bool squeeze)
{
    qDebug() << "set squeezed set to: " << squeeze;
    if (m_squeeze != squeeze)
    {
        m_requestMutex.lock();
        m_squeeze = squeeze;
        double forceDesired = m_forceDesired;
        bool measureForceRequest = m_measureForceRequest;
        m_requestMutex.unlock();
        sendMotorRequest(forceDesired, squeeze, measureForceRequest);
        emit onSqueezeChanged(squeeze);
    }
}

void ForceControllerWorker::setMeasureForceRequest(bool measureForceRequest)
{
    qDebug() << "force requested set to: " << measureForceRequest;
    if (m_measureForceRequest != measureForceRequest)
    {
        m_requestMutex.lock();
        m_measureForceRequest = measureForceRequest;
        double forceDesired = m_forceDesired = m_forceDesired;
        bool squeeze = m_squeeze;
        m_requestMutex.unlock();
        sendMotorRequest(forceDesired, squeeze, measureForceRequest);
        emit onMeasureForceRequestChanged(measureForceRequest);
    }
}

void ForceControllerWorker::addConnections(QObject *root)
{
    QObject::connect(
        root,
        SIGNAL(onDesiredForceChanged(double)),
        this,
        SLOT(setForceDesired(double)),
        Qt::DirectConnection);
    QObject::connect(
        root,
        SIGNAL(onMotorClosedRequested(bool)),
        this,
        SLOT(setSqueeze(bool)),
        Qt::DirectConnection);
    QObject::connect(
        root,
        SIGNAL(onForceRequestChanged(bool)),
        this,
        SLOT(setMeasureForceRequest(bool)),
        Qt::DirectConnection);
}

void ForceControllerWorker::sendMotorRequest(
    double force,
    bool enableMotorController,
    bool measureForce)
{
    grasper_msg::MotorRequestMessage request;
    request.appliedForce = force;
    request.enableMotorController = enableMotorController;
    request.measureForce = measureForce;

    m_motorRequestPub.publish(request);
}

void ForceControllerWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;

    m_motorRequestPub =
        n->advertise<grasper_msg::MotorRequestMessage>("serial/motor", 1000);
    m_motorMsgSubscriber = n->subscribe(
        "serial/motorFeedback",
        1000,
        &ForceControllerWorker::msgCallback,
        this);
    //    ros::Rate loopRate(10);

    // TODO add constant request polling in case messages fail to send.
    //    while (ros::ok())
    //    {
    //        ros::spinOnce();
    //        loopRate.sleep();
    //    }
}
