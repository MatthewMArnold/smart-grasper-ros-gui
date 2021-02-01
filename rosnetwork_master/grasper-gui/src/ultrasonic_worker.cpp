#include "ultrasonic_worker.hpp"

#include <QDebug>

#include "custom_plot_item.hpp"
#include "main_controller.hpp"
#include "value_updater.hpp"

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
    ValueUpdater *ultrasonicMeasurement = qobject_cast<ValueUpdater *>(
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsLeftCol")
            ->findChild<QObject *>("velOfSound")
            ->findChild<QObject *>("displayBorder")
            ->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(onVelocityOfSoundChanged(QString)),
        ultrasonicMeasurement,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);

    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>(
            "pulsePlot"));
    QObject::connect(
        this,
        SIGNAL(onVelocityOfSoundChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject *ultrasonicRadio =
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsLeftCol")
            ->findChild<QObject *>("velOfSound");
    QObject::connect(
        ultrasonicRadio,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(graphVelOfSound(bool)),
        Qt::DirectConnection);

    QObject *velOfSoundSwitch = root->findChild<QObject *>("velOfSoundSwitch");
    QObject::connect(
        velOfSoundSwitch,
        SIGNAL(sliderToggled(bool, int)),
        this,
        SLOT(setMeasureVelocityOfSound(bool, int)),
        Qt::DirectConnection);

    QObject::connect(
        this,
        SIGNAL(onMeasureVelocityOfSoundChanged(bool, int)),
        MainController::getInstance(),
        SLOT(setEnableVelocityOfSound(bool, int)));

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
            emit onVelocityOfSoundChanged(
                QString::number(m_velocityOfSound, 'g', 2));
            QMutexLocker lock(&m_graphControlLock);
            if (m_graphControl)
            {
                emit onVelocityOfSoundChangedWithTime(m_velocityOfSound, time);
            }
        }
    }
}

void UltrasonicWorker::setMeasureVelocityOfSound(
    bool measureVelocityOfSound,
    int index)
{
    qDebug() << "measure velocity of sound request: " << measureVelocityOfSound;
    m_measureVelocityOfSound = measureVelocityOfSound;
    emit onMeasureVelocityOfSoundChanged(m_measureVelocityOfSound, index);
}

void UltrasonicWorker::graphVelOfSound(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
            MainController::getInstance()
                ->getRoot()
                ->findChild<QObject *>("homeScreen")
                ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("Velocity of Sound, (m/sec)");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_graphControl = selected;
}

void UltrasonicWorker::run() {}
