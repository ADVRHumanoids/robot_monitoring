#ifndef FT_WIDGET_H
#define FT_WIDGET_H

#include "robot_monitoring/custom_qt_widget.h"

#include <QProgressBar>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>

#include <ros/ros.h>

namespace XBot { namespace Ui {


class GripperWidget : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "gripper_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    bool init(Args&) override;

    QString name() override;

    void update() override;

    bool loadConfig(const YAML::Node &cfg);

    bool saveConfig(YAML::Node &cfg);

    ~GripperWidget() override;

private:

    static void setProgressBarValue(QProgressBar * pb, double value, double max);

    ros::Publisher getCommandPublisher();

    ros::NodeHandle _nh;
    std::vector<ros::Subscriber> _subs;
    std::vector<QProgressBar*> _force, _torque;
    QComboBox * _select_gripper;
    std::map<QString, QString> _link_name_map;
    std::map<QString, ros::Publisher> _pub_map;
    QTimer * _pub_timer;


};

}}

#endif // Ft_WIDGET_H
