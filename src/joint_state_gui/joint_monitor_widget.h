#ifndef JOINT_MONITOR_WIDGET_H
#define JOINT_MONITOR_WIDGET_H

#include <QWidget>
#include <QTimer>

#include <ros/ros.h>
#include <xbot_msgs/JointState.h>
#include <urdf_parser/urdf_parser.h>

#include "bar_plot_widget.h"
#include "joint_state_widget.h"
#include "robot_monitoring/joint_sliders/sliders_widget_mainview.h"
#include "../chart/chart.h"


class JointMonitorWidget : public QWidget
{

public:

    explicit JointMonitorWidget(QWidget *parent = nullptr);


    BarPlotWidget * barplot_wid;
    JointStateWidget * jstate_wid;

private:

    ChartWidget * _chart;
    QTimer * _timer;
    ros::Subscriber _jstate_sub;
    bool _valid_msg_recv;
    std::vector<std::string> _jnames;
    urdf::ModelInterfaceSharedPtr _urdf;

    cartesio_gui::SlidersWidgetMainView * _sliders;

    void on_timer_event();
    void on_jstate_recv(xbot_msgs::JointStateConstPtr msg);

    std::map<std::string, int> _jidmap;

};

#endif // JOINT_MONITOR_WIDGET_H
