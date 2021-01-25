#include "bioimpedance_worker.hpp"

#include <QDebug>

#include "main_controller.hpp"
#include "value_updater.hpp"

void BioimpedanceWorker::msgCallback(
    const grasper_msg::ImpedanceDataMessage &msg)
{
    double avgX = 0;
    for (int i = 0; i < 50; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= 50.0;
    setImpedance(avgX, static_cast<double>(msg.dataPoint[49].time));
}

void BioimpedanceWorker::addConnections(QObject *root)
{
    ValueUpdater *impedanceMeasurement = qobject_cast<ValueUpdater *>(
        MainController::getInstance()
            ->getRoot()
            ->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsLeftCol")
            ->findChild<QObject *>("impedance")
            ->findChild<QObject *>("displayBorder")
            ->findChild<QObject *>("sensorReading"));
    if (impedanceMeasurement)
    {
        QObject::connect(
            this,
            SIGNAL(impedanceChanged(QString)),
            impedanceMeasurement,
            SLOT(setValue(QString)),
            Qt::QueuedConnection);
    }
    else
    {
        qDebug()
            << "impedance measurement not found, can't be displayed properly";
    }

    QObject::connect(
        root,
        SIGNAL(onImpedanceRequestChanged(bool)),
        this,
        SLOT(setImpedanceRequested(bool)),
        Qt::DirectConnection);
    QObject::connect(
        this,
        SIGNAL(onImpedanceRequestedChanged(bool)),
        MainController::getInstance(),
        SLOT(setEnableImpedance(bool)));
    m_impedanceMsgSubscriber =
        MainController::getInstance()->getNodeHandle()->subscribe(
            "serial/impedanceData",
            1000,
            &BioimpedanceWorker::msgCallback,
            this);
}

void BioimpedanceWorker::setImpedance(double impedance, double time)
{
    Q_UNUSED(time);
    if (m_impedance != impedance)
    {
        m_impedance = impedance;
        if (m_impedanceRequested)
        {
            //            qDebug() << impedance;
            emit impedanceChanged(QString::number(m_impedance, 'g', 2));
        }
    }
}

void BioimpedanceWorker::setImpedanceRequested(bool impedanceRequested)
{
    if (m_impedanceRequested != impedanceRequested)
    {
        m_impedanceRequested = impedanceRequested;
        emit onImpedanceRequestedChanged(impedanceRequested);
    }
}

void BioimpedanceWorker::run() {}
