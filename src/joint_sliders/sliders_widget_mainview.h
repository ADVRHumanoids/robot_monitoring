#ifndef IMPEDANCEWIDGETMAINVIEW_H
#define IMPEDANCEWIDGETMAINVIEW_H

#include "sliders_widget.h"
#include <rclcpp/rclcpp.hpp>
#include <xbot2_interface/xbotinterface2.h>
#include <xbot_msgs/msg/joint_state.hpp>
#include <xbot_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace cartesio_gui
{

class SlidersWidgetMainView : public QWidget
{


public:
    
    struct Options
    {
        std::string message_type;
        bool enable_velocity_tab;
        bool enable_effort_tab;

        std::string ns;
        std::string joint_state_topic;
        std::string command_topic;
        
        Options();
    };

    explicit SlidersWidgetMainView(Options opt = Options(),
                                   QWidget * parent = nullptr,
                                   rclcpp::Node::SharedPtr node = nullptr);

    void contextMenuEvent(QContextMenuEvent * event) override;

    void makeJointVisible(QString jointname);
    
    ~SlidersWidgetMainView();

private:

    void make_publisher();
    void try_construct();
    void construct();
    
    XBot::XBotInterface::Ptr _robot;

    std::map<std::string, cartesio_gui::SlidersWidget *> _wid_p_map, _wid_k_map, _wid_d_map;
    QStackedWidget * _wid_stack;
    QComboBox * _chain_select;
    QStatusBar * _status;

    bool _load_success;

    Options _opt;

    bool sense();

    void set_msg_type();
    void set_js_topic_name();
    void set_cmd_topic_name();
    void set_ros_namespace();
    void on_reload();
    void on_disable_enable();

    void pos_callback(std::string, double value);
    void vel_callback(std::string, double value);
    void tau_callback(std::string, double value);
    void k_callback(std::string, double value);
    void d_callback(std::string, double value);

    void print_status_msg(QString msg);

    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<xbot_msgs::msg::JointCommand>::SharedPtr _pub_xbot;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_ros;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _ros_js_sub;
    rclcpp::Subscription<xbot_msgs::msg::JointState>::SharedPtr _xbot_js_sub;
    sensor_msgs::msg::JointState _ros_js;
    xbot_msgs::msg::JointState _xbot_js;
};

}




#endif // IMPEDANCEWIDGET_H

