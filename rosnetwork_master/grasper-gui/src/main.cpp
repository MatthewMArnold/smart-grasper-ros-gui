#include <QApplication>
#include <QQmlApplicationEngine>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "main_controller.hpp"
#include "image_displayer.hpp"
#include "custom_plot_item.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include <QTimer>
#include <QImage>
#include <QPixmap>

using namespace cv;

int main(int argc, char *argv[])
{
    qmlRegisterType<CustomPlotItem>("CustomPlot", 1, 0, "CustomPlotItem");
    qmlRegisterType<ImageDisplayer>("ImageDisplayer", 1, 0, "ImageDisplayer");
    ros::init(argc, argv, "grasper_qt_gui");
    QApplication app(argc, argv);
    QQmlApplicationEngine appEngine;
    MainController::getInstance()->initialize(&appEngine);
    appEngine.load(QUrl("qrc:main.qml"));
    MainController::getInstance()->addConnections(appEngine.rootObjects().first());
    return app.exec();
}


 using namespace cv;
// MainWindow::MainWindow(QWidget *parent) :
//     QMainWindow(parent),
//     ui(new Ui::MainWindow)
// {
//     ui->setupUi(this);
//     Timer = new QTimer(this);
//     connect(Timer, SIGNAL(timeout()), this, SLOT(DisplayImage()));
//     Timer->start();
// }
// MainWindow::~MainWindow()
// {
//     delete ui;
// }
void displayImage(){
     Mat img;
     img = imread("E:/Bravelearn/Logo/mainlogo.png");
     cv::resize(img, img, Size(512, 384), 0, 0, INTER_LINEAR);
     cv::cvtColor(img,img,CV_BGR2RGB);
     QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
//     ui->display_image->setPixmap(QPixmap::fromImage(imdisplay));
 }
