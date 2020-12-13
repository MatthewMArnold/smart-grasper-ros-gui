#ifndef PULSEOX_WORKER_HPP
#define PULSEOX_WORKER_HPP

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QThread>
#include <QObject>
#include <QString>
#include <ros/ros.h>
#include <std_msgs/String.h>

class PulseoxWorker : public QThread
{
    Q_OBJECT
    Q_PROPERTY(double oxygenLevel
               READ oxygenLevel
               NOTIFY onOxygenLevelChanged
               WRITE setOxygenLevel);
    Q_PROPERTY(bool measurePulseox
               READ measurePulseox
               NOTIFY onMeasurePulseoxChanged
               WRITE setMeasurePulseox);

public:
    void msgCallback(const std_msgs::String &msg);
    void addConnections(QObject *root);

    double oxygenLevel() const { return m_oxygenLevel; }
    bool measurePulseox() const { return m_measurePulseox; }

public slots:
    void setOxygenLevel(double oxygenLevel);
    void setMeasurePulseox(bool measurePulseox);

signals:
    void onOxygenLevelChanged(double oxygenLevel);
    void onMeasurePulseoxChanged(bool measurePulseox);

private:
    double m_oxygenLevel;
    bool m_measurePulseox;

    void run() override;
};

#endif // PULSEOX_WORKER_HPP
