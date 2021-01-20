#include "pulseox_worker.hpp"

#include <QDebug>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "custom_plot_item.hpp"
#include "main_controller.hpp"

void PulseoxWorker::msgCallback(const grasper_msg::PulseOxRxMessage &msg)
{
    double avgX = 0;
    for (int i = 0; i < 50; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= 50.0;
    setOxygenLevel(avgX, static_cast<double>(msg.dataPoint[49].time));
}

void PulseoxWorker::initPulseOxGraph() {}

void PulseoxWorker::addConnections(QObject *root)
{
    QObject::connect(
        root,
        SIGNAL(onPulseOxRequestChanged(bool)),
        this,
        SLOT(setMeasurePulseox(bool)),
        Qt::DirectConnection);
    QObject::connect(
        this,
        SIGNAL(onMeasurePulseoxChanged(bool)),
        MainController::getInstance(),
        SLOT(setEnablePulseOx(bool)));
    m_pulseoxMsgSubscriber =
        MainController::getInstance()->getNodeHandle()->subscribe(
            "serial/pulseOxData",
            1000,
            &PulseoxWorker::msgCallback,
            this);
}

void PulseoxWorker::setOxygenLevel(double oxygenLevel, double time)
{
    if (m_oxygenLevel != oxygenLevel)
    {
        m_oxygenLevel = oxygenLevel;
        if (m_measurePulseox)
        {
            MainController::getInstance()->getRoot()->setProperty(
                "oxygen",
                QVariant(QString::number(m_oxygenLevel, 'g', 2)));
            emit onOxygenLevelChanged(m_oxygenLevel, time);
        }
    }
}

void PulseoxWorker::setMeasurePulseox(bool measurePulseox)
{
    if (m_measurePulseox != measurePulseox)
    {
        m_measurePulseox = measurePulseox;
        emit onMeasurePulseoxChanged(m_measurePulseox);
    }
}

void PulseoxWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;
}
