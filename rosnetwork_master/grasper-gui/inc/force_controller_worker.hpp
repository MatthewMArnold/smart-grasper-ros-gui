#ifndef FORCE_SENSOR_HPP_
#define FORCE_SENSOR_HPP_

#include <QApplication>
#include <QMutex>
#include <QQmlApplicationEngine>
#include <QString>
#include <QThread>
#include <grasper_msg/MotorMessageFeedback.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * Class that handles enabling and disabling the motor controller and
 * associated sensors, setting a desired force, and reading and
 * displaying actual force.
 */
class ForceControllerWorker : public QThread
{
    Q_OBJECT

public:
    void msgCallback(const grasper_msg::MotorMessageFeedback &msg);
    void addConnections(QObject *root);

    double forceDesired() const { return m_forceDesired; }
    double forceActual() const { return m_forceActual; }
    bool squeeze() const { return m_squeeze; }
    bool measureForceRequest() const { return m_measureForceRequest; }

public slots:
    void setForceDesired(double force);
    void setForceActual(double force, double time);
    void setPositionActual(double position, double time);
    void setSqueeze(bool squeeze);
    void setMeasureForceRequest(bool measureForceRequest, int index);
    void graphForce(bool);
    void graphPosition(bool);

signals:
    void onForceDesiredChanged(double force);
    void onForceActualChanged(QString force);
    void onSqueezeChanged(bool squeeze);
    void onForceActualChangedWithTime(double, double);
    void onPositionActualChanged(QString);
    void onPositionActualChangedWithTime(double, double);

private:
    double m_forceDesired = 0.0;
    double m_forceActual = 0.0;
    double m_positionActual = 0.0;
    bool m_squeeze = false;
    bool m_measureForceRequest = false;
    ros::Publisher m_motorRequestPub;
    QMutex m_requestMutex;
    ros::Subscriber m_motorMsgSubscriber;

    QMutex m_graphControlLock;
    bool m_forceGraphControl = false;
    bool m_positionGraphControl = false;

    void run() override;

    void sendMotorRequest(double force, bool enableMotorController, bool measureForce);
};

#endif  // FORCE_CONTROLLER_WORKER_HPP_
