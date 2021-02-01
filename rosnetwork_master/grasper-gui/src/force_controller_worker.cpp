#include "force_controller_worker.hpp"

#include <sstream>

#include <QDebug>
#include <grasper_msg/MotorRequestMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "custom_plot_item.hpp"
#include "main_controller.hpp"
#include "value_updater.hpp"

void ForceControllerWorker::msgCallback(
    const grasper_msg::MotorMessageFeedback &msg)
{
    setForceActual(msg.appliedForce);  // todo time
}

void ForceControllerWorker::setForceDesired(double force)
{
    if (m_forceDesired != force)
    {
        m_requestMutex.lock();
        m_forceDesired = force;
        bool squeeze = m_squeeze;
        bool measureForceRequest = m_measureForceRequest;
        m_requestMutex.unlock();
        sendMotorRequest(force, squeeze, measureForceRequest);
        emit onForceDesiredChanged(force);
    }
}

void ForceControllerWorker::setForceActual(double force)
{
    if (force != m_forceActual)
    {
        m_forceActual = force;
        if (m_measureForceRequest)
        {
            emit onForceActualChanged(QString::number(m_forceActual, 'g', 2));
            QMutexLocker lock(&m_graphControlLock);
            if (m_graphControl)
            {
                onForceActualChangedWithTime(m_forceActual, 1);  // todo time
            }
        }
    }
}

void ForceControllerWorker::setSqueeze(bool squeeze)
{
    qDebug() << "set squeezed set to: " << squeeze;
    if (m_squeeze != squeeze)
    {
        m_requestMutex.lock();
        m_squeeze = squeeze;
        double forceDesired = m_forceDesired;
        bool measureForceRequest = m_measureForceRequest;
        m_requestMutex.unlock();
        sendMotorRequest(forceDesired, squeeze, measureForceRequest);
        emit onSqueezeChanged(squeeze);
    }
}

void ForceControllerWorker::setMeasureForceRequest(
    bool measureForceRequest,
    int index)
{
    Q_UNUSED(index);
    qDebug() << "force requested set to: " << measureForceRequest;
    m_requestMutex.lock();
    m_measureForceRequest = measureForceRequest;
    double forceDesired = m_forceDesired = m_forceDesired;
    bool squeeze = m_squeeze;
    m_requestMutex.unlock();
    sendMotorRequest(forceDesired, squeeze, measureForceRequest);
}

void ForceControllerWorker::addConnections(QObject *root)
{
    ValueUpdater *forceMeasurement = qobject_cast<ValueUpdater *>(
        root->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(onForceActualChanged(QString)),
        forceMeasurement,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);

    QObject *forceRadio = root->findChild<QObject *>("homeScreen")
                              ->findChild<QObject *>("sensorReadingsLeftCol")
                              ->findChild<QObject *>("actualForce");
    QObject::connect(
        forceRadio,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(graphForce(bool)),
        Qt::DirectConnection);

    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>(
            "pulsePlot"));
    QObject::connect(
        this,
        SIGNAL(onForceActualChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject *measureForceSwitch =
        root->findChild<QObject *>("forceMeasurementSwitch");
    QObject::connect(
        measureForceSwitch,
        SIGNAL(sliderToggled(bool, int)),
        this,
        SLOT(setMeasureForceRequest(bool, int)),
        Qt::DirectConnection);

    QObject::connect(
        root,
        SIGNAL(onDesiredForceChanged(double)),
        this,
        SLOT(setForceDesired(double)),
        Qt::DirectConnection);

    QObject::connect(
        root,
        SIGNAL(onMotorClosedRequested(bool)),
        this,
        SLOT(setSqueeze(bool)),
        Qt::DirectConnection);
}

void ForceControllerWorker::sendMotorRequest(
    double force,
    bool enableMotorController,
    bool measureForce)
{
    grasper_msg::MotorRequestMessage request;
    request.appliedForce = force;
    request.enableMotorController = enableMotorController;
    request.measureForce = measureForce;

    m_motorRequestPub.publish(request);
}

void ForceControllerWorker::run()
{
    ros::NodeHandle *n = MainController::getInstance()->getNodeHandle();

    if (n == nullptr) return;

    m_motorRequestPub =
        n->advertise<grasper_msg::MotorRequestMessage>("serial/motor", 1000);
    m_motorMsgSubscriber = n->subscribe(
        "serial/motorFeedback",
        1000,
        &ForceControllerWorker::msgCallback,
        this);
    //    ros::Rate loopRate(10);

    // TODO add constant request polling in case messages fail to send.
    //    while (ros::ok())
    //    {
    //        ros::spinOnce();
    //        loopRate.sleep();
    //    }
}

void ForceControllerWorker::graphForce(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
            MainController::getInstance()
                ->getRoot()
                ->findChild<QObject *>("homeScreen")
                ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("force (N)");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_graphControl = selected;
}
