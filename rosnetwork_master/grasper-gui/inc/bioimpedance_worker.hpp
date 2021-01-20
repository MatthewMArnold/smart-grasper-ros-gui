#ifndef BIOIMPEDANCE_WORKER_HPP
#define BIOIMPEDANCE_WORKER_HPP

#include <QApplication>
#include <QMutex>
#include <QQmlApplicationEngine>
#include <QString>
#include <QThread>
#include <grasper_msg/ImpedanceDataMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * Class that handles enabling and disabling the motor controller and
 * associated sensors, setting a desired force, and reading and
 * displaying actual force.
 */
class BioimpedanceWorker : public QThread
{
    Q_OBJECT
    Q_PROPERTY(double impedance READ impedance NOTIFY onImpedanceChanged)
    Q_PROPERTY(bool impedanceRequested READ impedanceRequested NOTIFY
                   onImpedanceRequestedChanged WRITE setImpedanceRequested)

public:
    void msgCallback(const grasper_msg::ImpedanceDataMessage &msg);
    void addConnections(QObject *root);

    double impedance() const { return m_impedance; }
    bool impedanceRequested() const { return m_impedanceRequested; }

public slots:
    void setImpedance(double impedance, double time);
    void setImpedanceRequested(bool impedanceRequested);

signals:
    void onImpedanceChanged(double impedance);
    void onImpedanceRequestedChanged(bool impedanceRequested);

private:
    double m_impedance;
    bool m_impedanceRequested;
    ros::Subscriber m_impedanceMsgSubscriber;

    void run() override;
};

#endif  // BIOIMPEDANCE_WORKER_HPP
