#include "pulseox_worker.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QDebug>

#include "main_controller.hpp"

void PulseoxWorker::msgCallback(const std_msgs::String &msg)
{
    Q_UNUSED(msg);
}

void PulseoxWorker::addConnections(QObject *root)
{
    QObject::connect(root, SIGNAL(onPulseOxRequestChanged(bool)),
                     this, SLOT(setMeasurePulseox(bool)),
                     Qt::DirectConnection);
}

void PulseoxWorker::setOxygenLevel(double oxygenLevel)
{
    qDebug() << "oxygen level received of: " << oxygenLevel;
    if (m_oxygenLevel != oxygenLevel)
    {
        m_oxygenLevel = oxygenLevel;
        MainController::getInstance()->getRoot()->setProperty("oxygen", QVariant(m_oxygenLevel));
        emit onOxygenLevelChanged(m_oxygenLevel);
    }
}

void PulseoxWorker::setMeasurePulseox(bool measurePulseox)
{
    qDebug() << "measure pulse ox change requested of: " << measurePulseox;
    if (m_measurePulseox != measurePulseox)
    {
        m_measurePulseox = measurePulseox;
        MainController::getInstance()->setEnablePulseOx(measurePulseox);
        emit onMeasurePulseoxChanged(m_measurePulseox);
    }
}

void PulseoxWorker::run()
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
            setOxygenLevel(count);
        }
        ++count;

        ros::spinOnce();
        loopRate.sleep();
    }
}
