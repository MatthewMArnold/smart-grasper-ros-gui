#ifndef PULSEOX_WORKER_HPP
#define PULSEOX_WORKER_HPP

#include <QApplication>
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
    Q_PROPERTY(double oxygenLevel READ oxygenLevel)
    Q_PROPERTY(bool measurePulseox READ measurePulseox NOTIFY
                   onMeasurePulseoxChanged WRITE setMeasurePulseox)

public:
    void msgCallback(const grasper_msg::PulseOxRxMessage &msg);
    void initPulseOxGraph();
    void addConnections(QObject *root);

    double oxygenLevel() const { return m_oxygenLevel; }
    bool measurePulseox() const { return m_measurePulseox; }

public slots:
    void setOxygenLevel(double oxygenLevel, double time);
    void setMeasurePulseox(bool measurePulseox);

signals:
    void onOxygenLevelChanged(double oxygenLevel, double time);
    void onMeasurePulseoxChanged(bool measurePulseox);

private:
    double m_oxygenLevel;
    bool m_measurePulseox;
    ros::Subscriber m_pulseoxMsgSubscriber;

    QVector<double> m_pulseoxX;
    QVector<double> m_pulseoxY;

    void run() override;
};

#endif  // PULSEOX_WORKER_HPP
