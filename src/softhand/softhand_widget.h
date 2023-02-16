#ifndef FT_WIDGET_H
#define FT_WIDGET_H

#include "robot_monitoring/custom_qt_widget.h"

#include <QProgressBar>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>

#include <ros/ros.h>

namespace XBot { namespace Ui {


class SofthandWidget : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "softhand_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    bool init(Args&) override;

    QString name() override;

    void update() override;

    bool loadConfig(const YAML::Node &cfg);

    bool saveConfig(YAML::Node &cfg);

    ~SofthandWidget() override;

private:

//    void clearLayout(QLayout *layout);
    static void setProgressBarValue(QProgressBar * pb, double value, double max);

    ros::NodeHandle _nh;
    std::vector<ros::Subscriber> _subs;
    std::vector<QProgressBar*> _force, _torque;
    QComboBox * _select_softhand;
    std::map<QString, QString> _link_name_map;
    std::map<QString, ros::Publisher> _pub_map;


};

}}

#endif // Ft_WIDGET_H
