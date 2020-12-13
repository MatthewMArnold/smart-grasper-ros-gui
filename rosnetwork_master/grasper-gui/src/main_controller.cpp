#include "main_controller.hpp"

#include <QQmlContext>

#include "force_controller_worker.hpp"
#include "thermistor_worker.hpp"

MainController *MainController::mainController = nullptr;

MainController::~MainController()
{
    forceControllerThread.quit();
    forceControllerThread.wait();

    thermistorThread.quit();
    thermistorThread.wait();

    delete forceController;
    delete thermistor;
}

void MainController::initialize(QQmlApplicationEngine *engine)
{
    if (engine == nullptr) return;
    forceController = new ForceControllerWorker();
    thermistor = new ThermistorWorker();

    engine->rootContext()->setContextProperty("forceController", forceController);
}

void MainController::addConnections(QObject *root)
{
    if (root == nullptr) return;
    this->root = root;

    forceController->addConnections(root);
    thermistor->addConnections(root);

    connect(&forceControllerThread, &QThread::finished, forceController, &QObject::deleteLater);
    forceController->moveToThread(&forceControllerThread);
    forceController->start();

    connect(&thermistorThread, &QThread::finished, thermistor, &QObject::deleteLater);
    thermistor->moveToThread(&thermistorThread);
    thermistor->start();
}
