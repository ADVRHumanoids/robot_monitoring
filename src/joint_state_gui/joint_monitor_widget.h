#ifndef JOINT_MONITOR_WIDGET_H
#define JOINT_MONITOR_WIDGET_H

#include <QWidget>
#include <QTimer>
#include <QMainWindow>
#include <QStatusBar>
#include <QMenuBar>

#include <ros/ros.h>
#include <xbot_msgs/JointState.h>
#include <xbot_msgs/CustomState.h>
#include <xbot_msgs/Fault.h>
#include <urdf_parser/urdf_parser.h>

#include "xbot2_wid.h"
#include "bar_plot_widget.h"
#include "joint_state_widget.h"
#include "../joint_sliders/sliders_widget_mainview.h"
#include "top_right_tab.h"
#include "../qcustomplot/qcustom_chart.h"

/**
 * @brief The JointMonitorWidget class is the main widget for the
 * xbot2-gui.
 */
class JointMonitorWidget : public QMainWindow
{

public:

    explicit JointMonitorWidget(int argc = 0,
                                char ** argv = nullptr,
                                QWidget *parent = nullptr);

    /**
     * @brief barplot_wid is the bar plot for the joint state
     */
    BarPlotWidget * barplot_wid;

    /**
     * @brief jstate_wid is a widget showing the full state for
     * a single joint
     */
    JointStateWidget * jstate_wid;

    /**
     * @brief _chart is a live plot widget
     */
    QCustomChart * _chart;

    /**
     * @brief _sliders is a slider-based commander for the robot
     * joints, organized in a chain-wise fashion
     */
    cartesio_gui::SlidersWidgetMainView * _sliders;

    /**
     * @brief _xbot2
     */
    XBot2Widget * _xbot2;

    /**
     * @brief _status_bar
     */
    QStatusBar * _status_bar;

    /**
     * @brief _menu_bar
     */
    QMenuBar * _menu_bar;

    /**
     * @brief _xbot2_status
     */
    XBot2StatusWidget * _xbot2_status;

    /**
     * @brief _tr_tab
     */
    TopRightTab * _tr_tab;

    ~JointMonitorWidget();

private:

    XBot::Ui::Context::Ptr _ctx;

    QTimer * _timer;
    ros::Subscriber _jstate_sub, _aux_sub, _fault_sub;
    bool _valid_msg_recv;
    bool _widget_started;
    std::vector<std::string> _jnames;
    urdf::ModelInterfaceSharedPtr _urdf;

    void create_menu();
    void save_default_cfg();

    void on_timer_event();
    void on_jstate_recv(xbot_msgs::JointStateConstPtr msg);
    void on_fault_recv(xbot_msgs::FaultConstPtr msg);
    void on_aux_recv(xbot_msgs::CustomStateConstPtr msg);



    std::map<std::string, int> _jidmap;

};

#endif // JOINT_MONITOR_WIDGET_H
