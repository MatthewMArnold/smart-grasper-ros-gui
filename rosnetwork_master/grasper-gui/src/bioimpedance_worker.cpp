#include "bioimpedance_worker.hpp"

#include <QDebug>

#include "main_controller.hpp"

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
    impedanceMsgSubscriber =
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
            qDebug() << impedance;
            MainController::getInstance()->getRoot()->setProperty(
                "impedance",
                QVariant(QString::number(impedance, 'g', 2)));
            emit onImpedanceChanged(impedance);
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
