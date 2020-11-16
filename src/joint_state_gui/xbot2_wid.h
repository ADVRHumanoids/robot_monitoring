#ifndef XBOT2_WID_H
#define XBOT2_WID_H

#include <QWidget>
#include <functional>
#include <QProgressBar>
#include <QLabel>
#include <QTimer>

#include "xbot2_plugin_wid.h"
#include "xbot2_status_wid.h"

#include <ros/ros.h>

class XBot2Widget : public QWidget
{

    Q_OBJECT

public:

    explicit XBot2Widget(QWidget * parent = nullptr);

    void update();

signals:

private:

    ros::NodeHandle _nh;
    XBot2StatusWidget * _status_wid;
    std::map<std::string, XBot2PluginWidget*> _pl_map;
    ros::Subscriber _stats_sub, _jdinfo_sub;

};

#endif // XBOT2_WID_H
