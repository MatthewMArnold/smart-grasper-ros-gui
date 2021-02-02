#include "force_controller_worker.hpp"

#include <sstream>

#include <QDebug>
#include <grasper_msg/MotorRequestMessage.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "custom_plot_item.hpp"
#include "main_controller.hpp"
#include "value_updater.hpp"

void ForceControllerWorker::msgCallback(const grasper_msg::MotorMessageFeedback &msg)
{
    setForceActual(msg.appliedForce, msg.time);  // todo time
    setPositionActual(msg.jawPos, msg.time);
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

void ForceControllerWorker::setForceActual(double force, double time)
{
    if (force != m_forceActual)
    {
        m_forceActual = force;
        if (m_measureForceRequest)
        {
            emit onForceActualChanged(QString::number(m_forceActual, 'g', 2));
            QMutexLocker lock(&m_graphControlLock);
            if (m_forceGraphControl)
            {
                onForceActualChangedWithTime(m_forceActual, time);
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

void ForceControllerWorker::setMeasureForceRequest(bool measureForceRequest, int index)
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
        root->findChild<QObject *>("actualForce")->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(onForceActualChanged(QString)),
        forceMeasurement,
        SLOT(setValue(QString)),
        Qt::QueuedConnection);

    ValueUpdater *positionMeasurement = qobject_cast<ValueUpdater *>(
        root->findChild<QObject *>("jawPosition")->findChild<QObject *>("sensorReading"));
    QObject::connect(
        this,
        SIGNAL(onPositionActualChanged(QString)),
        positionMeasurement,
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

    QObject *positionRadio = root->findChild<QObject *>("jawPosition");
    QObject::connect(
        positionRadio,
        SIGNAL(onSelected(bool)),
        this,
        SLOT(graphPosition(bool)),
        Qt::DirectConnection);

    CustomPlotItem *graph = qobject_cast<CustomPlotItem *>(
        MainController::getInstance()->getRoot()->findChild<QObject *>("pulsePlot"));
    QObject::connect(
        this,
        SIGNAL(onForceActualChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject::connect(
        this,
        SIGNAL(onPositionActualChangedWithTime(double, double)),
        graph,
        SLOT(graphData(double, double)),
        Qt::DirectConnection);

    QObject *measureForceSwitch = root->findChild<QObject *>("forceMeasurementSwitch");
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

    m_motorRequestPub = n->advertise<grasper_msg::MotorRequestMessage>("serial/motor", 1000);
    m_motorMsgSubscriber =
        n->subscribe("serial/motorFeedback", 1000, &ForceControllerWorker::msgCallback, this);
}

void ForceControllerWorker::graphForce(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph =
            qobject_cast<CustomPlotItem *>(MainController::getInstance()
                                               ->getRoot()
                                               ->findChild<QObject *>("homeScreen")
                                               ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("Force, N");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_forceGraphControl = selected;
}

void ForceControllerWorker::graphPosition(bool selected)
{
    if (selected)
    {
        CustomPlotItem *graph =
            qobject_cast<CustomPlotItem *>(MainController::getInstance()
                                               ->getRoot()
                                               ->findChild<QObject *>("homeScreen")
                                               ->findChild<QObject *>("pulsePlot"));
        graph->setYAxisLabel("Position, mm");
        graph->initCustomPlot();
    }
    QMutexLocker lock(&m_graphControlLock);
    m_positionGraphControl = selected;
}

void ForceControllerWorker::setPositionActual(double position, double time)
{
    if (position != m_positionActual)
    {
        m_positionActual = position;
        if (m_measureForceRequest)
        {
            emit onPositionActualChanged(QString::number(m_positionActual, 'g', 2));
        }
    }
    QMutexLocker lock(&m_graphControlLock);
    if (m_positionGraphControl)
    {
        emit onPositionActualChangedWithTime(m_positionActual, time);
    }
}
