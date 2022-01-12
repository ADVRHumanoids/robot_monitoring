#ifndef Aux_WIDGET_H
#define Aux_WIDGET_H

#include "robot_monitoring/custom_qt_widget.h"

#include <QPushButton>
#include <QCheckBox>
#include <QTimer>

#include <ros/ros.h>

namespace XBot { namespace Ui {

class AuxWidget : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "aux_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    AuxWidget();

    bool init(Args&) override;

    QString name() override;

    void update() override;

    ~AuxWidget() override;

private:

    void on_btn_pressed();

    ros::NodeHandle _nh;

    ros::ServiceClient _set_aux_srv;

    std::vector<QCheckBox*> _aux_check_box;

};

}}

#endif // Aux_WIDGET_H
