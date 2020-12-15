#include "thermistor_worker.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QDebug>

#include "main_controller.hpp"

void ThermistorWorker::msgCallback(const grasper_msg::ThermistorMessage &msg)
{
    double avgX = 0;
    for (int i = 0; i < 50; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= 50.0;
    setTemperature(avgX, static_cast<double>(msg.dataPoint[49].time));
}

void ThermistorWorker::addConnections(QObject *root)
{
    QObject::connect(root, SIGNAL(onTemperatureRequestChanged(bool)),
                     this, SLOT(setMeasureTemperature(bool)),
                     Qt::DirectConnection);
    QObject::connect(this, SIGNAL(onMeasureTemperatureChanged(bool)),
                     MainController::getInstance(), SLOT(setEnableTemperature(bool)));
    thermistorMsgSubscriber = MainController::getInstance()->getNodeHandle()->subscribe(
                "serial/thermistorData",
                1000,
                &ThermistorWorker::msgCallback,
                this);
}

void ThermistorWorker::setTemperature(double temperature, double time)
{
    Q_UNUSED(time);
    if (m_temperature != temperature)
    {
        m_temperature = temperature;
        if (m_measureTemperature)
        {
            MainController::getInstance()->getRoot()->setProperty("temperature", QVariant(QString::number(m_temperature, 'g', 2)));
            emit onTemperatureChanged(m_temperature);
        }
    }
}

void ThermistorWorker::setMeasureTemperature(bool measureTemperature)
{
    qDebug() << "measure temperature change requested of: " << measureTemperature;
    if (m_measureTemperature != measureTemperature)
    {
        m_measureTemperature = measureTemperature;
        emit onMeasureTemperatureChanged(m_measureTemperature);
    }
}

void ThermistorWorker::run() {}
