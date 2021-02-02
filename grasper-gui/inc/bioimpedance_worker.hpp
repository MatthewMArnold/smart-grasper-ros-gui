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

public:
    void msgCallback(const grasper_msg::ImpedanceDataMessage &msg);
    void addConnections(QObject *root);

    double impedance() const { return m_impedance; }
    bool impedanceRequested() const { return m_impedanceRequested; }

public slots:
    void setImpedance(double impedance, double time);
    void setImpedanceRequested(bool impedanceRequested, int index);
    void graphImpedance(bool);

signals:
    void impedanceChanged(QString);
    void onImpedanceRequestedChanged(bool impedanceRequested, int index);
    void impedanceChangedWithTime(double, double);

private:
    QMutex m_graphControlLock;
    bool m_graphControl = false;

    double m_impedance;
    bool m_impedanceRequested;
    ros::Subscriber m_impedanceMsgSubscriber;

    void run() override;
};

#endif  // BIOIMPEDANCE_WORKER_HPP
