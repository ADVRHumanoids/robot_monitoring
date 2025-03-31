#include "sliders_widget_mainview.h"

#include <QApplication>

#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/ros2/config_from_param.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("impedance_widget_main");
    
    cartesio_gui::SlidersWidgetMainView::Options options;

    node->declare_parameter("message_type", "sensor_msgs");
    node->declare_parameter("enable_velocity_tab", true);
    node->declare_parameter("enable_effort_tab", true);

    options.message_type = node->get_parameter("message_type").as_string();
    options.enable_velocity_tab = node->get_parameter("enable_velocity_tab").as_bool();
    options.enable_effort_tab = node->get_parameter("enable_effort_tab").as_bool();

    QApplication a(argc, argv);
    cartesio_gui::SlidersWidgetMainView main_view(options, nullptr, node);
    main_view.show();
    return a.exec();
}

