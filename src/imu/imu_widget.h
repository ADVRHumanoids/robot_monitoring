#ifndef IMU_WIDGET_H
#define IMU_WIDGET_H

#include "../plugin/custom_qt_widget.h"

#include <QProgressBar>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>

#include <ros/ros.h>

namespace XBot { namespace Ui {

class ImuWidget : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "imu_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    ImuWidget();

    bool init(Args&) override;

    QString name() override;

    void update() override;

    ~ImuWidget() override;

private:

    static void setProgressBarValue(QProgressBar * pb, double value, double max);

    ros::NodeHandle _nh;
    std::vector<ros::Subscriber> _subs;
    std::vector<QProgressBar*> _omega, _linacc, _rot;
    QComboBox * _select_imu;
    std::map<QString, QString> _link_name_map;

};

}}

#endif // IMU_WIDGET_H
