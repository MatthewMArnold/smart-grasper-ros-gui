#include <QApplication>
#include <QQmlApplicationEngine>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "main_controller.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasper_qt_gui");
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine appEngine;
    MainController::getInstance()->initialize(&appEngine);
    appEngine.load(QUrl("qrc:main.qml"));
    MainController::getInstance()->addConnections(appEngine.rootObjects().first());
    return app.exec();
}