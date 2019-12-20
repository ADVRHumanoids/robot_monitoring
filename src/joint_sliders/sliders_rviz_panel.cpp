#include <robot_monitoring_tools/joint_sliders/sliders_widget_mainview.h>

#include <ros/ros.h>
#include <rviz/panel.h>



class SlidersRvizPanel : public rviz::Panel 
{
    
public:
    
    SlidersRvizPanel(QWidget * parent = nullptr);
    
private:
    
    
};

SlidersRvizPanel::SlidersRvizPanel(QWidget * parent):
    rviz::Panel(parent)
{
    
    auto * wid = new cartesio_gui::SlidersWidgetMainView;
    
    auto * vbox = new QVBoxLayout(parent);
    vbox->addWidget(wid);
    setLayout(vbox);
    
}



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SlidersRvizPanel, rviz::Panel)

