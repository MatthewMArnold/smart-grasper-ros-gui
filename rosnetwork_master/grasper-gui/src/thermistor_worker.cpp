#include "thermistor_worker.hpp"

#include <QDebug>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "custom_plot_item.hpp"
#include "main_controller.hpp"
#include "value_updater.hpp"

void ThermistorWorker::msgCallback(const grasper_msg::ThermistorMessage &msg)
{
    double avgX = 0;
    for (int i = 0; i < 50; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= 50.0;
    setTemperature(avgX, static_cast<double>(msg.dataPoint[49].time));
}

void ThermistorWorker::addConnections(QObject *root)
{
    ValueUpdater *temperatureMeasurement = qobject_cast<ValueUpdater *>(
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsRightCol")
            ->findChild<QObject *>("temperature")
            ->findChild<QObject *>("displayBorder")
            ->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(temperatureChanged(QString)),
        temperatureMeasurement,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);

    QObject *temperatureRadio =
        root->findChild<QObject *>("homeScreen")
            ->findChild<QObject *>("sensorReadingsRightCol")
            ->findChild<QObject *>("temperature");
    QObject::connect(
        temperatureRadio,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(graphTemperature(bool)),
        Qt::DirectConnection);

    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>(
            "pulsePlot"));
    QObject::connect(
        this,
        SIGNAL(temperatureChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject *temperatureSwitch =
        root->findChild<QObject *>("temperatureSwitch");
    QObject::connect(
        temperatureSwitch,
        SIGNAL(sliderToggled(bool, int)),
        this,
        SLOT(setMeasureTemperature(bool, int)),
        Qt::DirectConnection);

    QObject::connect(
        this,
        SIGNAL(onMeasureTemperatureChanged(bool, int)),
        MainController::getInstance(),
        SLOT(setEnableTemperature(bool, int)));

    m_thermistorMsgSubscriber =
        MainController::getInstance()->getNodeHandle()->subscribe(
            "serial/thermistorData",
            1000,
            &ThermistorWorker::msgCallback,
            this);
}

void ThermistorWorker::setTemperature(double temperature, double time)
{
    if (m_temperature != temperature)
    {
        m_temperature = temperature;
        if (m_measureTemperature)
        {
            emit temperatureChanged(QString::number(m_temperature, 'g', 2));
            QMutexLocker lock(&m_graphControlLock);
            if (m_graphControl)
            {
                emit temperatureChangedWithTime(m_temperature, time);
            }
        }
    }
}

void ThermistorWorker::setMeasureTemperature(bool measureTemperature, int index)
{
    qDebug() << "measure temperature change requested of: "
             << measureTemperature;
    m_measureTemperature = measureTemperature;
    emit onMeasureTemperatureChanged(m_measureTemperature, index);
}

void ThermistorWorker::graphTemperature(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
            MainController::getInstance()
                ->getRoot()
                ->findChild<QObject *>("homeScreen")
                ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("Temperature, (C)");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_graphControl = selected;
}

void ThermistorWorker::run() {}
