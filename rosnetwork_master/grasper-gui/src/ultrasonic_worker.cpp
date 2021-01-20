#include "ultrasonic_worker.hpp"

#include <QDebug>

#include "main_controller.hpp"

void UltrasonicWorker::msgCallback(
    const grasper_msg::UltrasonicDataMessage &msg)
{
    double avgX = 0;
    for (int i = 0; i < 50; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= 50.0;
    setVelocityOfSound(avgX, static_cast<double>(msg.dataPoint[49].time));
}

void UltrasonicWorker::addConnections(QObject *root)
{
    QObject::connect(
        root,
        SIGNAL(onVelocityOfSoundRequestChanged(bool)),
        this,
        SLOT(setMeasureVelocityOfSound(bool)),
        Qt::DirectConnection);
    QObject::connect(
        this,
        SIGNAL(onMeasureVelocityOfSoundChanged(bool)),
        MainController::getInstance(),
        SLOT(setEnableVelocityOfSound(bool)));
    m_ultrasonicMsgSubscriber =
        MainController::getInstance()->getNodeHandle()->subscribe(
            "serial/ultrasonicData",
            1000,
            &UltrasonicWorker::msgCallback,
            this);
}

void UltrasonicWorker::setVelocityOfSound(double velocityOfSound, double time)
{
    Q_UNUSED(time);
    if (m_velocityOfSound != velocityOfSound)
    {
        m_velocityOfSound = velocityOfSound;
        if (m_measureVelocityOfSound)
        {
            MainController::getInstance()->getRoot()->setProperty(
                "velocityOfSound",
                QVariant(QString::number(m_velocityOfSound, 'g', 2)));
            emit onVelocityOfSoundChanged(m_velocityOfSound);
        }
    }
}

void UltrasonicWorker::setMeasureVelocityOfSound(bool measureVelocityOfSound)
{
    qDebug() << "measure velocity of sound request: " << measureVelocityOfSound;
    if (m_measureVelocityOfSound != measureVelocityOfSound)
    {
        m_measureVelocityOfSound = measureVelocityOfSound;
        emit onMeasureVelocityOfSoundChanged(m_measureVelocityOfSound);
    }
}

void UltrasonicWorker::run() {}
