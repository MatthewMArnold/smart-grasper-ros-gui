#include "main_controller.hpp"

#include <QDebug>
#include <QQmlContext>

#include "custom_plot_item.hpp"
#include "error_controller.hpp"
#include "force_controller_worker.hpp"

void SensorRequestWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;

    ros::Publisher pub =
        n->advertise<grasper_msg::SensorRequestMessage>("serial/sensorEnable", 1000);

    ros::Rate loopRate(100);

    grasper_msg::SensorRequestMessage oldMsg;
    grasper_msg::SensorRequestMessage newMsg = MainController::getInstance()->getSensorRequestMsg();
    pub.publish(newMsg);
    while (ros::ok())
    {
        newMsg = MainController::getInstance()->getSensorRequestMsg();
        if (newMsg != oldMsg)
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

    sendSensorRequestThread.quit();
    sendSensorRequestThread.wait();

    delete forceController;
    delete thermistor;
    delete pulseox;
    delete ultrasonic;
    delete bioimpedance;
    delete sensorRequest;
    delete phaseAngle;
}

void MainController::teensyConnectedMsgCallback(const std_msgs::Bool &msg)
{
    emit teensyConnectedMsgReceived();
    if (msg.data) setTeensyConnected(true);
}

void MainController::serialNodeConnectedMsgCallback(const std_msgs::Bool &msg)
{
    emit serialNodeRunningMsgReceived();
    if (msg.data) setSerialNodeRunning(true);
}

void MainController::initialize(QQmlApplicationEngine *engine)
{
    if (engine == nullptr) return;

    forceController = new ForceControllerWorker();
    thermistor = new ThermistorMeasurement();
    pulseox = new PulseoxMeasurement();
    ultrasonic = new UltrasonicMeasurement();
    bioimpedance = new ImpedanceMeasurement();
    phaseAngle = new PhaseAngleMeasurement();
    sensorRequest = new SensorRequestWorker();

    engine->rootContext()->setContextProperty("forceController", forceController);

    m_teensyConnectedSub = m_nodeHandle.subscribe(
        "serial/mcuConnectedHandler",
        1000,
        &MainController::teensyConnectedMsgCallback,
        this);
    m_serialRunningSub = m_nodeHandle.subscribe(
        "serial/serialNodeRunning",
        1000,
        &MainController::serialNodeConnectedMsgCallback,
        this);
}

void MainController::addConnections(QObject *root)
{
    if (root == nullptr) return;
    this->root = root;

    forceController->addConnections(root);

    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>("pulsePlot"));

    QObject *thermistorRadioButton = root->findChild<QObject *>("temperature");
    ValueUpdater *thermistorMeasurement =
        qobject_cast<ValueUpdater *>(thermistorRadioButton->findChild<QObject *>("sensorReading"));
    thermistor->addConnections(
        thermistorMeasurement,
        graph,
        thermistorRadioButton,
        root->findChild<QObject *>("temperatureSwitch"));

    QObject *ultrasonicRadioButton = root->findChild<QObject *>("velOfSound");
    ValueUpdater *ultrasonicMeasurement =
        qobject_cast<ValueUpdater *>(ultrasonicRadioButton->findChild<QObject *>("sensorReading"));
    ultrasonic->addConnections(
        ultrasonicMeasurement,
        graph,
        ultrasonicRadioButton,
        root->findChild<QObject *>("velOfSoundSwitch"));

    QObject *pulseoxRadioButton = root->findChild<QObject *>("oxygen");
    ValueUpdater *pulseoxMeasurement =
        qobject_cast<ValueUpdater *>(pulseoxRadioButton->findChild<QObject *>("sensorReading"));
    pulseox->addConnections(
        pulseoxMeasurement,
        graph,
        pulseoxRadioButton,
        root->findChild<QObject *>("pulseOxSwitch"));

    QObject *impedanceRadioButton = root->findChild<QObject *>("impedance");
    ValueUpdater *impedanceMeasurement =
        qobject_cast<ValueUpdater *>(impedanceRadioButton->findChild<QObject *>("sensorReading"));
    bioimpedance->addConnections(
        impedanceMeasurement,
        graph,
        impedanceRadioButton,
        root->findChild<QObject *>("impedanceSwitch"));

    QObject *phaseAngleRadioButton = root->findChild<QObject *>("phaseAngle");
    ValueUpdater *phaseAngleMeasurement =
        qobject_cast<ValueUpdater *>(phaseAngleRadioButton->findChild<QObject *>("sensorReading"));
    phaseAngle->addConnections(
        phaseAngleMeasurement,
        graph,
        phaseAngleRadioButton,
        root->findChild<QObject *>("impedanceSwitch"));

    graph->initCustomPlot();

    connect(&forceControllerThread, &QThread::finished, forceController, &QObject::deleteLater);
    forceController->moveToThread(&forceControllerThread);
    forceController->start();

    QObject::connect(
        &m_teensyConnectedTimeout,
        SIGNAL(timeout()),
        this,
        SLOT(teensyDisconnected()));
    QObject::connect(
        this,
        SIGNAL(teensyConnectedMsgReceived()),
        &m_teensyConnectedTimeout,
        SLOT(start()),
        Qt::QueuedConnection);
    QObject::connect(
        &m_serialNodeconnectedTimeout,
        SIGNAL(timeout()),
        this,
        SLOT(serialNodeDisconnected()),
        Qt::DirectConnection);
    QObject::connect(
        this,
        SIGNAL(serialNodeRunningMsgReceived()),
        &m_serialNodeconnectedTimeout,
        SLOT(start()),
        Qt::QueuedConnection);

    m_teensyConnectedTimeout.start(TEENSY_TIMEOUT_MS);
    m_serialNodeconnectedTimeout.start(SEIRAL_CONNECTED_TIMEOUT_MS);
}

void MainController::teensyDisconnected() { setTeensyConnected(false); }

void MainController::serialNodeDisconnected() { setSerialNodeRunning(false); }

void MainController::setEnablePulseOx(bool enabled, int index)
{
    sensorRequestLock.lock();
    m_sensorRequestMessage.enablePulseOx = enabled;
    m_sensorRequestMessage.pulseoxIndex = index;
    sensorRequestLock.unlock();
}

void MainController::setEnableTemperature(bool enabled, int index)
{
    sensorRequestLock.lock();
    m_sensorRequestMessage.enableTemperature = enabled;
    m_sensorRequestMessage.temperatureIndex = index;
    sensorRequestLock.unlock();
}

void MainController::setEnableVelocityOfSound(bool enabled, int index)
{
    sensorRequestLock.lock();
    m_sensorRequestMessage.enableVelocityOfSound = enabled;
    m_sensorRequestMessage.velocityOfSoundIndex = index;
    sensorRequestLock.unlock();
}

void MainController::setEnableImpedance(bool enabled, int index)
{
    sensorRequestLock.lock();
    m_sensorRequestMessage.enableImpedance = enabled;
    m_sensorRequestMessage.impedanceIndex = index;
    sensorRequestLock.unlock();
}

void MainController::setTeensyConnected(bool teensyConnected)
{
    if (m_teensyConnected != teensyConnected)
    {
        m_teensyConnected = teensyConnected;

        if (m_teensyConnected)
        {
            qDebug() << "teensy connected";
            ErrorController::getInstance()->removeError(
                ErrorController::ErrorType::TEENSY_DISCONNECTED);
        }
        else
        {
            qDebug() << "teensy disconnected";
            ErrorController::getInstance()->addError(
                this,
                ErrorController::ErrorType::TEENSY_DISCONNECTED);
        }
    }
}

void MainController::setSerialNodeRunning(bool serialNodeRunning)
{
    if (m_serialNodeRunning != serialNodeRunning)
    {
        m_serialNodeRunning = serialNodeRunning;

        if (m_serialNodeRunning)
        {
            qDebug() << "serial node running";
            ErrorController::getInstance()->removeError(
                ErrorController::ErrorType::SERIAL_NODE_NOT_RUNNING);
        }
        else
        {
            qDebug() << "serial node not running";
            ErrorController::getInstance()->addError(
                this,
                ErrorController::ErrorType::SERIAL_NODE_NOT_RUNNING);
        }
    }
}

grasper_msg::SensorRequestMessage MainController::getSensorRequestMsg()
{
    sensorRequestLock.lock();
    grasper_msg::SensorRequestMessage msg = m_sensorRequestMessage;
    sensorRequestLock.unlock();
    return msg;
}

void MainController::graphMediator(double y, double x, bool hasControl)
{
    qDebug() << "received msg " << y << ", " << x;
    if (hasControl)
    {
        emit graphData(y, x);
    }
}
