#include "sensor_measurement.hpp"

#include "main_controller.hpp"

void SensorMeasurement::addConnections(
    ValueUpdater *measurementToUpdate,
    CustomPlotItem *graphToConnect,
    QObject *radioButton,
    QObject *onOffSwitch)
{
    QObject::connect(
        this,
        SIGNAL(onMeasurementChanged(QString)),
        measurementToUpdate,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);
    QObject::connect(
        this,
        SIGNAL(onMeasurementChangedWithTime(double, double)),
        graphToConnect,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);
    QObject::connect(
        radioButton,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(shouldGraphData(bool)),
        Qt::DirectConnection);
    QObject::connect(
        onOffSwitch,
        SIGNAL(sliderToggled(bool, int)),
        this,
        SLOT(measurementRequested(bool, int)),
        Qt::DirectConnection);
}

void SensorMeasurement::measurementRequested(bool mRequested, int index)
{
    m_measurementRequested = mRequested;
    emit onMeasurementRequestChanged(m_measurementRequested, index);
}

void SensorMeasurement::shouldGraphData(bool shouldGraph)
{
    if (shouldGraph)
    {
        CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
            MainController::getInstance()->getRoot()->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel(m_graphLabelY);
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_graphControl = shouldGraph;
}

bool SensorMeasurement::getGraphControl()
{
    QMutexLocker lock(&m_graphControlLock);
    return m_graphControl;
}

bool SensorMeasurement::getMeasurementRequested() { return m_measurementRequested; }
