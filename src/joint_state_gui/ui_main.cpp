#include "joint_monitor_widget.h"
#include <QApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
//    QCoreApplication::setAttribute(Qt::AA_DontUseNativeMenuBar); //fix for menubar notshowing in ubuntu
    QApplication a(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("xbot2_gui");
    JointMonitorWidget w(0, nullptr, nullptr, node);
    w.show();
    return a.exec();
}
