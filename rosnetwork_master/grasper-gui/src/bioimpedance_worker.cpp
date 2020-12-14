#include "bioimpedance_worker.hpp"

#include "main_controller.hpp"

void BioimpedanceWorker::msgCallback(const grasper_msg::MotorMessageFeedback &msg)
{
    Q_UNUSED(msg);
}

void BioimpedanceWorker::addConnections(QObject *root)
{
    QObject::connect(root, SIGNAL(onImpedenceRequestChanged(bool)),
                     this, SLOT(setImpedanceRequested(bool)),
                     Qt::DirectConnection);
    QObject::connect(this, SIGNAL(onImpedanceRequestedChanged(bool)),
                     MainController::getInstance(), SLOT(setEnableImpedence(bool)));
}

void BioimpedanceWorker::setImpedance(double impedance)
{
    if (m_impedance != impedance)
    {
        m_impedance = impedance;
        emit onImpedanceChanged(impedance);
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

void BioimpedanceWorker::run()
{
}
