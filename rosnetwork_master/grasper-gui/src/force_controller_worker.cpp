#include "force_controller_worker.hpp"

#include <QDebug>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>

#include "main_controller.hpp"

void ForceControllerWorker::msgCallback(const grasper_msg::MotorMessageFeedback &msg)
{
//    ROS_INFO("hello world");
}

void ForceControllerWorker::testMsgCallback(const std_msgs::String &msg)
{
//    std::cout << msg << std::endl;
//    setForceActual(forceActual + 1);
}

void ForceControllerWorker::setForceDesired(double force)
{
    qDebug() << "force desired set to: " << force;
    if (m_forceDesired != force)
    {
        m_forceDesired = force;
        emit onForceDesiredChanged(m_forceDesired);
        // TODO send ROS message
    }
}

void ForceControllerWorker::setForceActual(double force)
{
    qDebug() << "actual force received: " << force;
    if (force != m_forceActual)
    {
        m_forceActual = force;
        MainController::getInstance()->getRoot()->setProperty("actualForce", QVariant(force));
        emit onForceActualChanged(m_forceActual);
    }
}

void ForceControllerWorker::setSqueeze(bool squeeze)
{
    qDebug() << "set squeezed set to: " << squeeze;
    if (m_squeeze != squeeze)
    {
        // TODO send ROS message to request motor squeeze.
        m_squeeze = squeeze;
        emit onSqueezeChanged(m_squeeze);
    }
}

void ForceControllerWorker::setMeasureForceRequest(bool measureForceRequest)
{
    qDebug() << "force requested set to: " << measureForceRequest;
    if (m_measureForceRequest != measureForceRequest)
    {
        // TODO send ROS message to request force measurement.
        m_measureForceRequest = measureForceRequest;
        emit onMeasureForceRequestChanged(m_measureForceRequest);
    }
}

void ForceControllerWorker::addConnections(QObject *root)
{
    QObject::connect(root, SIGNAL(onDesiredForceChanged(double)),
                     this, SLOT(setForceDesired(double)),
                     Qt::DirectConnection);
    QObject::connect(root, SIGNAL(onMotorClosedRequested(bool)),
                     this, SLOT(setSqueeze(bool)),
                     Qt::DirectConnection);
    QObject::connect(root, SIGNAL(onForceRequestChanged(bool)),
                     this, SLOT(setMeasureForceRequest(bool)),
                     Qt::DirectConnection);
}

void ForceControllerWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;

    ros::Publisher chatter_pub = n->advertise<std_msgs::String>("chatter", 1000);

    ros::Subscriber testSub = n->subscribe("chatter",
                                           1000,
                                           &ForceControllerWorker::testMsgCallback,
                                           this);
    ros::Subscriber motorMsgSubscriber = n->subscribe("serial/motorFeedback",
                                                      1000,
                                                      &ForceControllerWorker::msgCallback,
                                                      this);

    ros::Rate loopRate(10);

    int count = 0;
    while (ros::ok())
    {
        // TODO replace all this with real stuff eventually
        if (count % 10 == 1)
        {
            setForceActual(count);
        }

        std_msgs::String msg;
        std::stringstream ss;
        msg.data = ss.str();
        chatter_pub.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
        ++count;
    }
}
