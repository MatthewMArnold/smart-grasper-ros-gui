#include "ultrasonic_worker.hpp"

#include <QDebug>

#include "main_controller.hpp"

void UltrasonicWorker::msgCallback(const std_msgs::String &msg)
{
    Q_UNUSED(msg);
}

void UltrasonicWorker::addConnections(QObject *root)
{
    QObject::connect(root, SIGNAL(onVelocityOfSoundRequestChanged(bool)),
                     this, SLOT(setMeasureVelocityOfSound(bool)),
                     Qt::DirectConnection);
}

void UltrasonicWorker::setVelocityOfSound(double velocityOfSound)
{
    qDebug() << "velocity of sound set to: " << velocityOfSound;
    if (m_velocityOfSound != velocityOfSound)
    {
        m_velocityOfSound = velocityOfSound;
        MainController::getInstance()->getRoot()->setProperty("velocityOfSound", QVariant(velocityOfSound));
        emit onVelocityOfSoundChanged(m_velocityOfSound);
    }
}

void UltrasonicWorker::setMeasureVelocityOfSound(bool measureVelocityOfSound)
{
    qDebug() << "measure velocity of sound request: " << measureVelocityOfSound;
    if (m_measureVelocityOfSound != measureVelocityOfSound)
    {
        m_measureVelocityOfSound = measureVelocityOfSound;
        MainController::getInstance()->setEnableVelocityOfSound(measureVelocityOfSound);
        emit onMeasureVelocityOfSoundChanged(m_measureVelocityOfSound);
    }
}

void UltrasonicWorker::run()
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
            setVelocityOfSound(count);
        }
        ++count;

        ros::spinOnce();
        loopRate.sleep();
    }
}
