#ifndef MAIN_CONTROLLER_HPP_
#define MAIN_CONTROLLER_HPP_

#include <QApplication>
#include <QMutex>
#include <QQmlApplicationEngine>
#include <QThread>
#include <QTimer>
#include <grasper_msg/SensorRequestMessage.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "error_reporter.hpp"
#include "sensor_measurement_instances.hpp"

class ForceControllerWorker;

/**
 * Thread that publishes sensor requests to '/serial/sensorEnable'
 * whenever the main controller's sensor request message has changed.
 */
class SensorRequestWorker : public QThread
{
    Q_OBJECT
    void run() override;
};

/**
 * Main controller that sets up all worker threads that connect ros
 * publishers and the gui.
 */
class MainController : public QObject, public ErrorReporter
{
    Q_OBJECT
    Q_PROPERTY(bool teensyConnected READ teensyConnected WRITE setTeensyConnected)

public:
    static inline MainController *getInstance()
    {
        if (mainController == nullptr)
        {
            mainController = new MainController();
        }
        return mainController;
    }

    MainController(const MainController &) = delete;
    MainController &operator=(const MainController &) = delete;
    ~MainController();

    void initialize(QQmlApplicationEngine *engine);
    void addConnections(QObject *root);

    ros::NodeHandle *getNodeHandle() { return &m_nodeHandle; }

    QObject *getRoot() { return root; }

    grasper_msg::SensorRequestMessage getSensorRequestMsg();

    bool teensyConnected() const { return m_teensyConnected; }

    void teensyConnectedMsgCallback(const std_msgs::Bool &msg);

    void serialNodeConnectedMsgCallback(const std_msgs::Bool &msg);

    void errorCleared(ErrorController::ErrorType type) override { Q_UNUSED(type); }

public slots:
    void setEnablePulseOx(bool enabled, int index);
    void setEnableTemperature(bool enabled, int index);
    void setEnableVelocityOfSound(bool enabled, int index);
    void setEnableImpedance(bool enabled, int index);
    void setTeensyConnected(bool teensyConnected);
    void teensyDisconnected();
    void setSerialNodeRunning(bool serialNodeRunning);
    void serialNodeDisconnected();
    void graphMediator(double, double, bool);

signals:
    void teensyConnectedMsgReceived();
    void serialNodeRunningMsgReceived();
    void graphData(double, double);

private:
    static constexpr int TEENSY_TIMEOUT_MS = 1000;
    static constexpr int SEIRAL_CONNECTED_TIMEOUT_MS = 1000;

    static MainController *mainController;

    // Thread workers
    ForceControllerWorker *forceController;
    ThermistorMeasurement *thermistor;
    PulseoxMeasurement *pulseox;
    UltrasonicMeasurement *ultrasonic;
    ImpedanceMeasurement *bioimpedance;
    SensorRequestWorker *sensorRequest;

    // Threads
    QThread forceControllerThread;
    QThread sendSensorRequestThread;

    grasper_msg::SensorRequestMessage m_sensorRequestMessage;
    QMutex sensorRequestLock;

    ros::NodeHandle m_nodeHandle;
    QObject *root;

    bool m_teensyConnected = true;
    ros::Subscriber m_teensyConnectedSub;
    QTimer m_teensyConnectedTimeout;

    bool m_serialNodeRunning = true;
    ros::Subscriber m_serialRunningSub;
    QTimer m_serialNodeconnectedTimeout;

    MainController() = default;
};

#endif
