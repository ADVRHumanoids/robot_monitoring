#ifndef JOINT_MONITOR_WIDGET_H
#define JOINT_MONITOR_WIDGET_H

#include <QWidget>
#include <QTimer>
#include <QMainWindow>
#include <QStatusBar>
#include <QMenuBar>

#include <rclcpp/rclcpp.hpp>
#include <xbot_msgs/msg/joint_state.hpp>
#include <xbot_msgs/msg/custom_state.hpp>
#include <xbot_msgs/msg/fault.hpp>
#include <urdf_parser/urdf_parser.h>

#include "xbot2_wid.h"
#include "bar_plot_widget.h"
#include "joint_state_widget.h"
#include "../joint_sliders/sliders_widget_mainview.h"
#include "top_right_tab.h"
#include "../qcustomplot/qcustom_chart.h"

using namespace std::placeholders;

/**
 * @brief The JointMonitorWidget class is the main widget for the
 * xbot2-gui.
 */
class JointMonitorWidget : public QMainWindow
{

public:

    explicit JointMonitorWidget(int argc = 0,
                                char ** argv = nullptr,
                                QWidget *parent = nullptr, 
                                rclcpp::Node::SharedPtr node = nullptr);

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

    rclcpp::Node::SharedPtr _node;

    XBot::Ui::Context::Ptr _ctx;

    QTimer * _timer;
    rclcpp::Subscription<xbot_msgs::msg::JointState>::SharedPtr _jstate_sub;
    rclcpp::Subscription<xbot_msgs::msg::CustomState>::SharedPtr _aux_sub;
    rclcpp::Subscription<xbot_msgs::msg::Fault>::SharedPtr _fault_sub;
    bool _valid_msg_recv;
    bool _widget_started;
    std::vector<std::string> _jnames;
    urdf::ModelInterfaceSharedPtr _urdf;

    void create_menu();
    void save_default_cfg();

    void on_timer_event();
    void on_jstate_recv(const xbot_msgs::msg::JointState & msg);
    void on_fault_recv(const xbot_msgs::msg::Fault & msg);
    void on_aux_recv(const xbot_msgs::msg::CustomState &msg);

    std::map<std::string, int> _jidmap;

};

#endif // JOINT_MONITOR_WIDGET_H
