#include <robot_monitoring_tools/joint_sliders/sliders_widget_mainview.h>

#include <QApplication>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "impedance_widget_main");
    
    ros::NodeHandle nhpr("~");
    cartesio_gui::SlidersWidgetMainView::Options options;
    options.message_type = nhpr.param<std::string>("message_type", "sensor_msgs");
    options.enable_velocity_tab = nhpr.param("enable_velocity_tab", true);
    options.enable_effort_tab = nhpr.param("enable_effort_tab", true);

    
    QApplication a(argc, argv);
    cartesio_gui::SlidersWidgetMainView main_view(options);
    main_view.show();
    return a.exec();
}

