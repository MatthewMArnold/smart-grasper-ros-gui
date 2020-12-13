#include "thermistor_worker.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QDebug>

#include "main_controller.hpp"

void ThermistorWorker::msgCallback(const std_msgs::String &msg)
{
    Q_UNUSED(msg);
}

void ThermistorWorker::addConnections(QObject *root)
{
    QObject::connect(root, SIGNAL(onTemperatureRequestChanged(bool)),
                     this, SLOT(setMeasureTemperature(bool)),
                     Qt::DirectConnection);
}

void ThermistorWorker::setTemperature(double temperature)
{
    qDebug() << "temperature received of: " << temperature;
    if (m_temperature != temperature)
    {
        m_temperature = temperature;
        MainController::getInstance()->getRoot()->setProperty("temperature", QVariant(m_temperature));
        emit onTemperatureChanged(m_temperature);
    }
}

void ThermistorWorker::setMeasureTemperature(bool measureTemperature)
{
    qDebug() << "measure temperature change requested of: " << measureTemperature;
    if (m_measureTemperature != measureTemperature)
    {
        m_measureTemperature = measureTemperature;
        MainController::getInstance()->setEnableTemperature(measureTemperature);
        emit onMeasureTemperatureChanged(m_measureTemperature);
    }
}

void ThermistorWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;

    Q_UNUSED(n);

    ros::Rate loopRate(10);

    int count = 0;
    while (ros::ok())
    {
        // TODO replace all this with real stuff eventually
        if (count % 10 == 1)
        {
            setTemperature(count);
        }
        ++count;

        ros::spinOnce();
        loopRate.sleep();
    }
}
