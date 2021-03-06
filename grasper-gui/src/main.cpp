#include <QApplication>
#include <QQmlApplicationEngine>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "custom_plot_item.hpp"
#include "image_displayer.hpp"
#include "main_controller.hpp"
#include "value_updater.hpp"

int main(int argc, char *argv[])
{
    qmlRegisterType<CustomPlotItem>("CustomPlot", 1, 0, "CustomPlotItem");
    qmlRegisterType<ImageDisplayer>("ImageDisplayer", 1, 0, "ImageDisplayer");
    qmlRegisterType<ValueUpdater>("ValueUpdater", 1, 0, "ValueUpdater");
    ros::init(argc, argv, "grasper_qt_gui");
    QApplication app(argc, argv);
    QQmlApplicationEngine appEngine;
    ErrorController::getInstance()->initialize();
    MainController::getInstance()->initialize(&appEngine);
    appEngine.load(QUrl("qrc:main.qml"));
    MainController::getInstance()->addConnections(appEngine.rootObjects().first());
    ErrorController::getInstance()->addConnections(appEngine.rootObjects().first());
    return app.exec();
}
