#ifndef IMPEDANCEWIDGETMAINVIEW_H
#define IMPEDANCEWIDGETMAINVIEW_H

#include "sliders_widget.h"
#include <XBotInterface/XBotInterface.h>
#include <ros/ros.h>

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
                                   QWidget * parent = nullptr);

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

    ros::NodeHandle _nh;
    ros::Publisher _pub;
};

}




#endif // IMPEDANCEWIDGET_H

