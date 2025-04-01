#ifndef XBOT2_WID_H
#define XBOT2_WID_H

#include <QWidget>
#include <functional>
#include <QProgressBar>
#include <QLabel>
#include <QTimer>

#include "xbot2_plugin_wid.h"
#include "xbot2_status_wid.h"

#include <rclcpp/rclcpp.hpp>
#include <xbot_msgs/msg/statistics2.hpp>
#include <xbot_msgs/msg/joint_device_info.hpp>

class XBot2Widget : public QWidget
{

    Q_OBJECT

public:

    explicit XBot2Widget(QMainWindow * mw, QWidget * parent, 
        rclcpp::Node::SharedPtr node);

    void update();

signals:

private:

    rclcpp::Node::SharedPtr _node;
    XBot2StatusWidget * _status_wid;
    std::map<std::string, XBot2PluginWidget*> _pl_map;
    rclcpp::Subscription<xbot_msgs::msg::Statistics2>::SharedPtr _stats_sub;
    rclcpp::Subscription<xbot_msgs::msg::JointDeviceInfo>::SharedPtr _jdinfo_sub;
    //rclcpp::Subscription<rosgraph_msgs::msg::log>::SharedPtr _stderr_sub;

};

#endif // XBOT2_WID_H
