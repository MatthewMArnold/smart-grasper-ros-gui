#ifndef SENSOR_MEASUREMENT_INSTANCES_HPP_
#define SENSOR_MEASUREMENT_INSTANCES_HPP_

#include <grasper_msg/ImpedanceDataMessage.h>
#include <grasper_msg/PulseOxRxMessage.h>
#include <grasper_msg/ThermistorMessage.h>
#include <grasper_msg/UltrasonicDataMessage.h>

#include "sensor_measurement.hpp"

class ImpedanceMeasurement : public SensorMeasurement
{
public:
    ImpedanceMeasurement();
    void msgCallback(const grasper_msg::ImpedanceDataMessage &msg);
};

class PulseoxMeasurement : public SensorMeasurement
{
public:
    PulseoxMeasurement();
    void msgCallback(const grasper_msg::PulseOxRxMessage &msg);
};

class UltrasonicMeasurement : public SensorMeasurement
{
public:
    UltrasonicMeasurement();
    void msgCallback(const grasper_msg::UltrasonicDataMessage &msg);
};

class ThermistorMeasurement : public SensorMeasurement
{
public:
    ThermistorMeasurement();
    void msgCallback(const grasper_msg::ThermistorMessage &msg);
};

#endif  // SENSOR_MEASUREMENT_INSTANCES_HPP_
