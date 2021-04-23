#include "sensor_measurement_instances.hpp"

#include <cmath>

#include <QDebug>

#include "main_controller.hpp"

QString limitedNum(double num, int n)
{
    int i = rint(num * pow(10, n));
    if (i % 100)
    {
        return QString::number(num, 'f', 2);
    }
    else
    {
        return QString::number(i / 100, 'f', 2);
    }
}

ImpedanceMeasurement::ImpedanceMeasurement() : SensorMeasurement("Impedance  (\u2126)")
{
    m_measurementSubscriber = MainController::getInstance()->getNodeHandle()->subscribe(
        "serial/impedanceData",
        ROS_QUEUE_SIZE,
        &ImpedanceMeasurement::msgCallback,
        this);
}

void ImpedanceMeasurement::msgCallback(const grasper_msg::ImpedanceDataMessage &msg)
{
    constexpr int MSG_LEN = 50;
    double avgX = 0;
    for (int i = 0; i < MSG_LEN; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= static_cast<double>(MSG_LEN);
    if (m_currAvgValue != avgX)
    {
        m_currAvgValue = avgX;
        if (this->getMeasurementRequested())
        {
            emit onMeasurementChanged(limitedNum(m_currAvgValue));
        }
    }
    if (this->getMeasurementRequested())
    {
        bool gc = this->getGraphControl();
        if (gc)
        {
            emit onMeasurementChangedWithTime(m_currAvgValue, msg.dataPoint[MSG_LEN - 1].time);
        }
    }
}

PulseoxMeasurement::PulseoxMeasurement() : SensorMeasurement("Oxygen Level (%)", true)
{
    m_measurementSubscriber = MainController::getInstance()->getNodeHandle()->subscribe(
        "serial/pulseOxData",
        ROS_QUEUE_SIZE,
        &PulseoxMeasurement::msgCallback,
        this);
}

void PulseoxMeasurement::msgCallback(const grasper_msg::PulseOxRxMessage &msg)
{
    constexpr int MSG_LEN = 50;
    double avgX = 0;
    for (int i = 0; i < MSG_LEN; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= static_cast<double>(MSG_LEN);
    if (m_currAvgValue != avgX)
    {
        m_currAvgValue = avgX;
        if (this->getMeasurementRequested())
        {
            emit onMeasurementChanged(limitedNum(m_currAvgValue));
        }
    }
    if (this->getMeasurementRequested())
    {
        bool gc = this->getGraphControl();
        if (gc)
        {
            emit onMeasurementChangedWithTime(m_currAvgValue, msg.dataPoint[MSG_LEN - 1].time);
        }
    }
}

UltrasonicMeasurement::UltrasonicMeasurement() : SensorMeasurement("Velocity of Sound (m/sec)")
{
    m_measurementSubscriber = MainController::getInstance()->getNodeHandle()->subscribe(
        "serial/ultrasonicData",
        ROS_QUEUE_SIZE,
        &UltrasonicMeasurement::msgCallback,
        this);
}

void UltrasonicMeasurement::msgCallback(const grasper_msg::UltrasonicDataMessage &msg)
{
    constexpr int MSG_LEN = 100;
    double avgX = 0;
    for (int i = 0; i < MSG_LEN; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= static_cast<double>(MSG_LEN);
    if (m_currAvgValue != avgX)
    {
        m_currAvgValue = avgX;
        if (this->getMeasurementRequested())
        {
            emit onMeasurementChanged(limitedNum(m_currAvgValue));
        }
    }
    if (this->getMeasurementRequested())
    {
        bool gc = this->getGraphControl();
        if (gc)
        {
            emit onMeasurementChangedWithTime(m_currAvgValue, msg.dataPoint[MSG_LEN - 1].time);
        }
    }
}

ThermistorMeasurement::ThermistorMeasurement() : SensorMeasurement("Temperature (\u00B0C)")
{
    m_measurementSubscriber = MainController::getInstance()->getNodeHandle()->subscribe(
        "serial/thermistorData",
        ROS_QUEUE_SIZE,
        &ThermistorMeasurement::msgCallback,
        this);
}

void ThermistorMeasurement::msgCallback(const grasper_msg::ThermistorMessage &msg)
{
    if (m_currAvgValue != msg.dataPoint.data)
    {
        m_currAvgValue = msg.dataPoint.data;
        if (this->getMeasurementRequested())
        {
            emit onMeasurementChanged(limitedNum(m_currAvgValue));
        }
    }
    if (this->getMeasurementRequested())
    {
        if (this->getGraphControl())
        {
            emit onMeasurementChangedWithTime(m_currAvgValue, msg.dataPoint.time);
        }
    }
}

PhaseAngleMeasurement::PhaseAngleMeasurement() : SensorMeasurement("Phase Angle (degrees)")
{
    m_measurementSubscriber = MainController::getInstance()->getNodeHandle()->subscribe(
        "serial/phaseAngleData",
        ROS_QUEUE_SIZE,
        &PhaseAngleMeasurement::msgCallback,
        this);
}

void PhaseAngleMeasurement::msgCallback(const grasper_msg::PhaseAngleMessage &msg)
{
    constexpr int MSG_LEN = 50;
    double avgX = 0;
    for (int i = 0; i < MSG_LEN; i++)
    {
        avgX += msg.dataPoint[i].data;
    }
    avgX /= static_cast<double>(MSG_LEN);
    if (m_currAvgValue != avgX)
    {
        m_currAvgValue = avgX;
        if (this->getMeasurementRequested())
        {
            emit onMeasurementChanged(limitedNum(m_currAvgValue));
        }
    }
    if (this->getMeasurementRequested())
    {
        bool gc = this->getGraphControl();
        if (gc)
        {
            emit onMeasurementChangedWithTime(m_currAvgValue, msg.dataPoint[MSG_LEN - 1].time);
        }
    }
}
