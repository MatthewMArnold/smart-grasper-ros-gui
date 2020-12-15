#ifndef ULTRASONIC_WORKER_HPP
#define ULTRASONIC_WORKER_HPP

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QThread>
#include <QObject>
#include <QString>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <grasper_msg/UltrasonicDataMessage.h>

class UltrasonicWorker : public QThread
{
    Q_OBJECT
    Q_PROPERTY(double velocityOfSound
               READ velocityOfSound
               NOTIFY onVelocityOfSoundChanged)
    Q_PROPERTY(bool measureVelocityOfSound
               READ measureVelocityOfSound
               NOTIFY onMeasureVelocityOfSoundChanged
               WRITE setMeasureVelocityOfSound)

public:
    void msgCallback(const grasper_msg::UltrasonicDataMessage &msg);
    void addConnections(QObject *root);

    double velocityOfSound() const { return m_velocityOfSound; }
    bool measureVelocityOfSound() const { return m_measureVelocityOfSound; }

public slots:
    void setVelocityOfSound(double velocityOfSound, double time);
    void setMeasureVelocityOfSound(bool measureVelocityOfSound);

signals:
    void onVelocityOfSoundChanged(double velocityOfSound);
    void onMeasureVelocityOfSoundChanged(bool measureVelocityOfSound);

private:
    double m_velocityOfSound;
    bool m_measureVelocityOfSound;
    ros::Subscriber ultrassonicMsgSubscriber;

    void run() override;
};

#endif // ULTRASONIC_WORKER_HPP
