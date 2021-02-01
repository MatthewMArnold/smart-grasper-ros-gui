#ifndef PULSEOX_WORKER_HPP
#define PULSEOX_WORKER_HPP

#include <QApplication>
#include <QMutex>
#include <QObject>
#include <QQmlApplicationEngine>
#include <QString>
#include <QThread>
#include <QVector>
#include <grasper_msg/PulseOxRxMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class PulseoxWorker : public QThread
{
    Q_OBJECT

public:
    void msgCallback(const grasper_msg::PulseOxRxMessage &msg);
    void initPulseOxGraph();
    void addConnections(QObject *root);

    double oxygenLevel() const { return m_oxygenLevel; }
    bool measurePulseox() const { return m_measurePulseox; }

public slots:
    void setOxygenLevel(double oxygenLevel, double time);
    void setMeasurePulseox(bool measurePulseox, int index);
    void graphPulseox(bool);

signals:
    void oxygenLevelChanged(QString oxygenLevel);
    void oxygenLevelChangedWithTime(double oxygenLevel, double time);
    void onMeasurePulseoxChanged(bool measurePulseox, int index);

private:
    double m_oxygenLevel;
    bool m_measurePulseox;
    ros::Subscriber m_pulseoxMsgSubscriber;
    QMutex m_graphControlLock;
    bool m_graphControl = false;

    QVector<double> m_pulseoxX;
    QVector<double> m_pulseoxY;

    void run() override;
};

#endif  // PULSEOX_WORKER_HPP
