#include "bioimpedance_worker.hpp"

#include <QDebug>

#include "custom_plot_item.hpp"
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
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsLeftCol")
            ->findChild<QObject *>("impedance")
            ->findChild<QObject *>("displayBorder")
            ->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(impedanceChanged(QString)),
        impedanceMeasurement,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);
    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>(
            "pulsePlot"));
    QObject::connect(
        this,
        SIGNAL(impedanceChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject *impedanceRadioButton =
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsLeftCol")
            ->findChild<QObject *>("impedance");
    QObject::connect(
        impedanceRadioButton,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(graphImpedance(bool)),
        Qt::DirectConnection);

    QObject *bioimpedanceSwitch = root->findChild<QObject *>("impedanceSwitch");
    QObject::connect(
        bioimpedanceSwitch,
        SIGNAL(sliderToggled(bool, int)),
        this,
        SLOT(setImpedanceRequested(bool, int)),
        Qt::DirectConnection);

    QObject::connect(
        this,
        SIGNAL(onImpedanceRequestedChanged(bool, int)),
        MainController::getInstance(),
        SLOT(setEnableImpedance(bool, int)),
        Qt::DirectConnection);

    m_impedanceMsgSubscriber =
        MainController::getInstance()->getNodeHandle()->subscribe(
            "serial/impedanceData",
            1000,
            &BioimpedanceWorker::msgCallback,
            this);
}

void BioimpedanceWorker::graphImpedance(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
            MainController::getInstance()
                ->getRoot()
                ->findChild<QObject *>("homeScreen")
                ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("impedance");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_graphControl = selected;
}

void BioimpedanceWorker::setImpedance(double impedance, double time)
{
    if (m_impedance != impedance)
    {
        m_impedance = impedance;
        if (m_impedanceRequested)
        {
            emit impedanceChanged(QString::number(m_impedance, 'g', 2));
            QMutexLocker lock(&m_graphControlLock);
            if (m_graphControl)
            {
                emit impedanceChangedWithTime(m_impedance, time);
            }
        }
    }
}

void BioimpedanceWorker::setImpedanceRequested(
    bool impedanceRequested,
    int index)
{
    m_impedanceRequested = impedanceRequested;
    emit onImpedanceRequestedChanged(impedanceRequested, index);
}

void BioimpedanceWorker::run() {}
