#include "main_controller.hpp"

#include <QQmlContext>
#include <QDebug>

#include "force_controller_worker.hpp"
#include "thermistor_worker.hpp"
#include "pulseox_worker.hpp"
#include "ultrasonic_worker.hpp"
#include "bioimpedance_worker.hpp"

void SensorRequestWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;

    ros::Publisher pub = n->advertise<grasper_msg::SensorRequestMessage>("serial/sensorEnable", 1000);

    ros::Rate loopRate(10);

    grasper_msg::SensorRequestMessage oldMsg;
    grasper_msg::SensorRequestMessage newMsg = MainController::getInstance()->getSensorRequestMsg();
    pub.publish(newMsg);
    while (ros::ok())
    {
        newMsg = MainController::getInstance()->getSensorRequestMsg();
        if (newMsg.enablePulseOx != oldMsg.enablePulseOx ||
                newMsg.enableTemperature != oldMsg.enableTemperature ||
                newMsg.enableVelocityOfSound != oldMsg.enableVelocityOfSound ||
                newMsg.enableImpedance != oldMsg.enableImpedance)
        {
            pub.publish(newMsg);
        }
        ros::spinOnce();
        loopRate.sleep();
        oldMsg = newMsg;
    }
}

MainController *MainController::mainController = nullptr;

MainController::~MainController()
{
    forceControllerThread.quit();
    forceControllerThread.wait();

    thermistorThread.quit();
    thermistorThread.wait();

    pulseoxThread.quit();
    pulseoxThread.wait();

    ultrasonicThread.quit();
    ultrasonicThread.wait();

    sendSensorRequestThread.quit();
    sendSensorRequestThread.wait();

    bioimpedanceThread.quit();
    bioimpedanceThread.wait();

    delete forceController;
    delete thermistor;
    delete pulseox;
    delete ultrasonic;
    delete bioimpedance;
    delete sensorRequest;
}

void MainController::initialize(QQmlApplicationEngine *engine)
{
    if (engine == nullptr) return;

    forceController = new ForceControllerWorker();
    thermistor = new ThermistorWorker();
    pulseox = new PulseoxWorker();
    ultrasonic = new UltrasonicWorker();
    bioimpedance = new BioimpedanceWorker();
    sensorRequest = new SensorRequestWorker();

    engine->rootContext()->setContextProperty("forceController", forceController);
}

void MainController::addConnections(QObject *root)
{
    if (root == nullptr) return;
    this->root = root;

    forceController->addConnections(root);
    thermistor->addConnections(root);
    ultrasonic->addConnections(root);
    pulseox->addConnections(root);
    bioimpedance->addConnections(root);

    connect(&forceControllerThread, &QThread::finished, forceController, &QObject::deleteLater);
    forceController->moveToThread(&forceControllerThread);
    forceController->start();

    connect(&thermistorThread, &QThread::finished, thermistor, &QObject::deleteLater);
    thermistor->moveToThread(&thermistorThread);
    thermistor->start();

    connect(&pulseoxThread, &QThread::finished, pulseox, &QObject::deleteLater);
    pulseox->moveToThread(&pulseoxThread);
    pulseox->start();

    connect(&ultrasonicThread, &QThread::finished, ultrasonic, &QObject::deleteLater);
    ultrasonic->moveToThread(&ultrasonicThread);
    ultrasonic->start();

    connect(&bioimpedanceThread, &QThread::finished, bioimpedance, &QObject::deleteLater);
    bioimpedance->moveToThread(&bioimpedanceThread);
    bioimpedance->start();

    connect(&sendSensorRequestThread, &QThread::finished, sensorRequest, &QObject::deleteLater);
    sensorRequest->moveToThread(&sendSensorRequestThread);
    sensorRequest->start();
}

void MainController::setEnablePulseOx(bool enabled)
{
    sensorRequestLock.lock();
    sensorRequestMessage.enablePulseOx = enabled;
    sensorRequestLock.unlock();
}

void MainController::setEnableTemperature(bool enabled)
{
    sensorRequestLock.lock();
    sensorRequestMessage.enableTemperature = enabled;
    sensorRequestLock.unlock();
}

void MainController::setEnableVelocityOfSound(bool enabled)
{
    sensorRequestLock.lock();
    sensorRequestMessage.enableVelocityOfSound = enabled;
    sensorRequestLock.unlock();
}

void MainController::setEnableImpedance(bool enabled)
{
    sensorRequestLock.lock();
    sensorRequestMessage.enableImpedance = enabled;
    sensorRequestLock.unlock();
}

grasper_msg::SensorRequestMessage MainController::getSensorRequestMsg()
{
    sensorRequestLock.lock();
    grasper_msg::SensorRequestMessage msg = sensorRequestMessage;
    sensorRequestLock.unlock();
    return msg;
}
