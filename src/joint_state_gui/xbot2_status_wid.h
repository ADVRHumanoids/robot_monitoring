#ifndef XBOT2_STATUS_WID_H
#define XBOT2_STATUS_WID_H

#include <QWidget>
#include <QLCDNumber>
#include <QPushButton>
#include <QLabel>
#include <QMainWindow>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <xbot_msgs/srv/start_process.hpp>
#include <xbot_msgs/srv/stop_process.hpp>

using namespace std::chrono_literals;

class XBot2StatusWidget : public QWidget
{

    Q_OBJECT

public:

    XBot2StatusWidget(QMainWindow * mw,
                      QWidget * parent,
                      rclcpp::Node::SharedPtr node);

    void update();

signals:

    void xbot2Started();

private:
    
    rclcpp::Node::SharedPtr _node;
    void handleStatusLabel();

    rclcpp::Time _last_status_recv;

    QMainWindow * _mw;
    QLCDNumber * _lcd;
    QLabel * _status_label;
    QPushButton * _cmd_button;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _status_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _vbatt_sub;
    
    rclcpp::Client<xbot_msgs::srv::StartProcess>::SharedPtr _srv_start;
    rclcpp::Client<xbot_msgs::srv::StopProcess>::SharedPtr _srv_stop;

    std::string _hw_type;
};

#endif // XBOT2_STATUS_WID_H
