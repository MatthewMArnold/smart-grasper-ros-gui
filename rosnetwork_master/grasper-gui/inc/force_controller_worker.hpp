#ifndef FORCE_SENSOR_HPP_
#define FORCE_SENSOR_HPP_

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QThread>
#include <QString>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <grasper_msg/MotorMessageFeedback.h>

/**
 * Class that handles enabling and disabling the motor controller and
 * associated sensors, setting a desired force, and reading and
 * displaying actual force.
 */
class ForceControllerWorker : public QThread
{
    Q_OBJECT
    Q_PROPERTY(double forceDesired
               READ forceDesired
               NOTIFY onForceDesiredChanged
               WRITE setForceDesired);
    Q_PROPERTY(double forceActual
               READ forceActual
               NOTIFY onForceActualChanged
               WRITE setForceActual);
    Q_PROPERTY(bool squeeze
               READ squeeze
               NOTIFY onSqueezeChanged
               WRITE setSqueeze);
    Q_PROPERTY(bool measureForceRequest
               READ measureForceRequest
               NOTIFY onMeasureForceRequestChanged
               WRITE setMeasureForceRequest);

public:
    void msgCallback(const grasper_msg::MotorMessageFeedback &msg);
    void testMsgCallback(const std_msgs::String &msg);
    void addConnections(QObject *root);

    double forceDesired() const { return m_forceDesired; }
    double forceActual() const { return m_forceActual; }
    bool squeeze() const { return m_squeeze; }
    bool measureForceRequest() const { return m_measureForceRequest; }

public slots:
    void setForceDesired(double force);
    void setForceActual(double force);
    void setSqueeze(bool squeeze);
    void setMeasureForceRequest(bool measureForceRequest);

signals:
    void onForceDesiredChanged(QVariant force);
    void onForceActualChanged(QVariant force);
    void onSqueezeChanged(bool squeeze);
    void onMeasureForceRequestChanged(QVariant measureForceRequest);

private:
    double m_forceDesired = 0.0f;
    double m_forceActual = 10.0f;
    bool m_squeeze = false;
    bool m_measureForceRequest = false;

    void run() override;
};

#endif  // FORCE_CONTROLLER_WORKER_HPP_
