#include "joint_monitor_widget.h"
#include <QApplication>
#include <QTimer>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "xbot_gui");
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QApplication a(argc, argv);
    JointMonitorWidget w;
    w.show();
    return a.exec();
}
