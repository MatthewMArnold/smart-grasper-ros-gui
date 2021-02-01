#ifndef SENSOR_MEASUREMENT_HPP_
#define SENSOR_MEASUREMENT_HPP_

#include <QMutex>
#include <QObject>
#include <QString>
#include <ros/ros.h>

#include "custom_plot_item.hpp"
#include "value_updater.hpp"

class SensorMeasurement : public QObject
{
    Q_OBJECT

public:
    explicit SensorMeasurement(QString graphLabelY, bool graphControl = false)
        : m_graphControl(graphControl),
          m_graphLabelY(graphLabelY)
    {
    }
    ~SensorMeasurement() = default;

    void addConnections(
        ValueUpdater *measurementToUpdate,
        CustomPlotItem *graphToConnect,
        QObject *radioButton,
        QObject *onOffSwitch);

public slots:
    void shouldGraphData(bool);
    void measurementRequested(bool, int);

signals:
    void onMeasurementChanged(QString);
    void onMeasurementChangedWithTime(double, double);
    void onMeasurementRequestChanged(bool, int);

protected:
    static constexpr int ROS_QUEUE_SIZE = 1000;

    double m_currAvgValue = 0.0;
    ros::Subscriber m_measurementSubscriber;

    bool getGraphControl();
    bool getMeasurementRequested();

private:
    QMutex m_graphControlLock;
    bool m_graphControl;
    bool m_measurementRequested = false;
    QString m_graphLabelY;
};

#endif  // SENSOR_MEASUREMENT_HPP_
