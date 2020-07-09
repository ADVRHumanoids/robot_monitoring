#ifndef XBOT2_WID_H
#define XBOT2_WID_H

#include <QWidget>
#include <functional>
#include <QProgressBar>
#include <QLabel>
#include <QTimer>

#include "xbot2_plugin_wid.h"

#include <ros/ros.h>

class XBot2Widget : public QWidget
{

    Q_OBJECT

public:

    explicit XBot2Widget(QWidget * parent = nullptr);

signals:

private:

    ros::NodeHandle _nh;
    std::map<std::string, XBot2PluginWidget*> _pl_map;
    ros::Subscriber _stats_sub, _jdinfo_sub;

};

#endif // XBOT2_WID_H
