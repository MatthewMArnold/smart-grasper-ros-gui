#include "pulseox_worker.hpp"

#include <QDebug>
#include <QQmlProperty>
#include <std_msgs/String.h>

#include "custom_plot_item.hpp"
#include "main_controller.hpp"
#include "value_updater.hpp"

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
    ValueUpdater *pulseoxMeasurement = qobject_cast<ValueUpdater *>(
        MainController::getInstance()
            ->getRoot()
            ->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsRightCol")
            ->findChild<QObject *>("oxygen")
            ->findChild<QObject *>("displayBorder")
            ->findChild<QObject *>("sensorReading"));

    if (pulseoxMeasurement)
    {
        QObject::connect(
            this,
            SIGNAL(oxygenLevelChanged(QString)),
            pulseoxMeasurement,
            SLOT(setValue(QString)),
            Qt::QueuedConnection);
    }
    else
    {
        qDebug() << "failed to find pulse measurement value to update";
    }

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
            emit oxygenLevelChangedWithTime(m_oxygenLevel, time);
            emit oxygenLevelChanged(QString::number(m_oxygenLevel, 'g', 2));
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
