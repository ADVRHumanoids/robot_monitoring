#include <pluginlib/class_list_macros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

#include <robot_monitoring_tools/joint_sliders/sliders_widget_mainview.h>

namespace cartesio_rqt
{

class SlidersWidgetRqt : public rqt_gui_cpp::Plugin
{
//     Q_OBJECT
    
public:
    
    SlidersWidgetRqt();
    
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    
    virtual void shutdownPlugin(){}
    
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
                              qt_gui_cpp::Settings& instance_settings) const {}
                              
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings) {}

    // Comment in to signal that the plugin has a way to configure it
    //bool hasConfiguration() const;
    //void triggerConfiguration();
    
    
private:
    
    QWidget* widget_;

    
};


SlidersWidgetRqt::SlidersWidgetRqt()
{
    setObjectName("XBotSlider");
}

void SlidersWidgetRqt::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    
    cartesio_gui::SlidersWidgetMainView::Options options;
    options.message_type = "xbot_msgs";
    options.enable_velocity_tab = false;
    options.enable_effort_tab = false;
    options.joint_state_topic = "xbotcore/joint_states";
    options.command_topic = "xbotcore/command";
    
    widget_ = new cartesio_gui::SlidersWidgetMainView(options);
    widget_->setWindowTitle("Joint Sliders");
    
    // add widget to the user interface
    context.addWidget(widget_);
}

}


PLUGINLIB_EXPORT_CLASS(cartesio_rqt::SlidersWidgetRqt, rqt_gui_cpp::Plugin)
