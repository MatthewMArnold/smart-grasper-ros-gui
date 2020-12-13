#ifndef MAIN_CONTROLLER_HPP_
#define MAIN_CONTROLLER_HPP_

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QThread>
#include <ros/ros.h>

class ForceControllerWorker;
class ThermistorWorker;

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

public slots:
signals:

private:
    static MainController *mainController;

    // Thread workers
    ForceControllerWorker *forceController;
    ThermistorWorker *thermistor;

    // Threads
    QThread forceControllerThread;
    QThread thermistorThread;

    ros::NodeHandle n;
    QObject *root;

    MainController() {}
};

#endif
