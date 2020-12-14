#ifndef MAIN_CONTROLLER_HPP_
#define MAIN_CONTROLLER_HPP_

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QThread>
#include <QMutex>
#include <ros/ros.h>
#include <grasper_msg/SensorRequestMessage.h>

class ForceControllerWorker;
class ThermistorWorker;
class PulseoxWorker;
class UltrasonicWorker;
class BioimpedanceWorker;

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
class MainController : public QObject
{
    Q_OBJECT

public:
    static inline MainController *getInstance()
    {
        if (mainController == nullptr) {
            mainController = new MainController();
        }
        return mainController;
    }

    MainController(const MainController &) = delete;
    MainController &operator=(const MainController &) = delete;
    ~MainController();

    void initialize(QQmlApplicationEngine *engine);
    void addConnections(QObject *root);

    ros::NodeHandle *getNodeHandle() { return &n; }

    QObject *getRoot() { return root; }

    grasper_msg::SensorRequestMessage getSensorRequestMsg();

public slots:
    void setEnablePulseOx(bool enabled);
    void setEnableTemperature(bool enabled);
    void setEnableVelocityOfSound(bool enabled);
    void setEnableImpedance(bool enabled);

private:
    static MainController *mainController;

    // Thread workers
    ForceControllerWorker *forceController;
    ThermistorWorker *thermistor;
    PulseoxWorker *pulseox;
    UltrasonicWorker *ultrasonic;
    BioimpedanceWorker *bioimpedance;
    SensorRequestWorker *sensorRequest;

    // Threads
    QThread forceControllerThread;
    QThread thermistorThread;
    QThread pulseoxThread;
    QThread ultrasonicThread;
    QThread bioimpedanceThread;
    QThread sendSensorRequestThread;

    grasper_msg::SensorRequestMessage sensorRequestMessage;
    QMutex sensorRequestLock;

    ros::NodeHandle n;
    QObject *root;

    MainController() = default;
};

#endif
