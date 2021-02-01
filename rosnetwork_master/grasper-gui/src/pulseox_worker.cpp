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
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsRightCol")
            ->findChild<QObject *>("oxygen")
            ->findChild<QObject *>("displayBorder")
            ->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(oxygenLevelChanged(QString)),
        pulseoxMeasurement,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);

    QObject *pulseoxRadio = root->findChild<QObject *>("homeScreen")
                                ->findChild<QObject *>("sensorReadingsRightCol")
                                ->findChild<QObject *>("oxygen");
    QObject::connect(
        pulseoxRadio,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(graphPulseox(bool)),
        Qt::DirectConnection);

    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>(
            "pulsePlot"));
    QObject::connect(
        this,
        SIGNAL(oxygenLevelChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject *pulseoxSwitch = root->findChild<QObject *>("pulseOxSwitch");
    QObject::connect(
        pulseoxSwitch,
        SIGNAL(sliderToggled(bool, int)),
        this,
        SLOT(setMeasurePulseox(bool, int)),
        Qt::DirectConnection);

    QObject::connect(
        this,
        SIGNAL(onMeasurePulseoxChanged(bool, int)),
        MainController::getInstance(),
        SLOT(setEnablePulseOx(bool, int)));

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
            emit oxygenLevelChanged(QString::number(m_oxygenLevel, 'g', 2));
            QMutexLocker lock(&m_graphControlLock);
            if (m_graphControl)
            {
                emit oxygenLevelChangedWithTime(m_oxygenLevel, time);
            }
        }
    }
}

void PulseoxWorker::graphPulseox(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
            MainController::getInstance()
                ->getRoot()
                ->findChild<QObject *>("homeScreen")
                ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("Oxygen level (SpO2%)");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_graphControl = selected;
}

void PulseoxWorker::setMeasurePulseox(bool measurePulseox, int index)
{
    m_measurePulseox = measurePulseox;
    qDebug() << "onmeasurepulseox " << index;
    emit onMeasurePulseoxChanged(m_measurePulseox, index);
}

void PulseoxWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;
}
