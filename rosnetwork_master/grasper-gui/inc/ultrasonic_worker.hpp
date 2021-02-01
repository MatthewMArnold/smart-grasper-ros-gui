#ifndef ULTRASONIC_WORKER_HPP
#define ULTRASONIC_WORKER_HPP

#include <QApplication>
#include <QMutex>
#include <QObject>
#include <QQmlApplicationEngine>
#include <QString>
#include <QThread>
#include <grasper_msg/UltrasonicDataMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class UltrasonicWorker : public QThread
{
    Q_OBJECT

public:
    void msgCallback(const grasper_msg::UltrasonicDataMessage &msg);
    void addConnections(QObject *root);

    double velocityOfSound() const { return m_velocityOfSound; }
    bool measureVelocityOfSound() const { return m_measureVelocityOfSound; }

public slots:
    void setVelocityOfSound(double velocityOfSound, double time);
    void setMeasureVelocityOfSound(bool measureVelocityOfSound, int index);
    void graphVelOfSound(bool);

signals:
    void onVelocityOfSoundChanged(QString velocityOfSound);
    void onMeasureVelocityOfSoundChanged(bool measureVelocityOfSound, int index);
    void onVelocityOfSoundChangedWithTime(double, double);

private:
    double m_velocityOfSound;
    bool m_measureVelocityOfSound;
    ros::Subscriber m_ultrasonicMsgSubscriber;
    QMutex m_graphControlLock;
    bool m_graphControl = false;

    void run() override;
};

#endif  // ULTRASONIC_WORKER_HPP
