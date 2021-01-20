#ifndef THERMISTOR_WORKER_HPP
#define THERMISTOR_WORKER_HPP

#include <QApplication>
#include <QObject>
#include <QQmlApplicationEngine>
#include <QString>
#include <QThread>
#include <grasper_msg/ThermistorMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class ThermistorWorker : public QThread
{
    Q_OBJECT
    Q_PROPERTY(double temperature READ temperature NOTIFY onTemperatureChanged)
    Q_PROPERTY(bool measureTemperature READ measureTemperature NOTIFY
                   onMeasureTemperatureChanged WRITE setMeasureTemperature)

public:
    void msgCallback(const grasper_msg::ThermistorMessage &msg);
    void addConnections(QObject *root);

    double temperature() const { return m_temperature; }
    bool measureTemperature() const { return m_measureTemperature; }

public slots:
    void setTemperature(double temperature, double time);
    void setMeasureTemperature(bool measureTemperature);

signals:
    void onTemperatureChanged(double temperature);
    void onMeasureTemperatureChanged(bool measureTemperature);

private:
    double m_temperature;
    bool m_measureTemperature;
    ros::Subscriber m_thermistorMsgSubscriber;

    void run() override;
};

#endif  // THERMISTOR_WORKER_HPP
