#include "joint_monitor_widget.h"
#include <QApplication>
#include <QTimer>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    for(auto k : QStyleFactory::keys())
    {
        std::cout << k.toStdString() << std::endl;
    }

    ros::init(argc, argv, "monitor_gui_node");
    QApplication a(argc, argv);
    JointMonitorWidget w;
    w.show();
    return a.exec();
}
