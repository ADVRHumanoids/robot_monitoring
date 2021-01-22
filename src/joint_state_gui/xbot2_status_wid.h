#ifndef XBOT2_STATUS_WID_H
#define XBOT2_STATUS_WID_H

#include <QWidget>
#include <QLCDNumber>
#include <QPushButton>
#include <QLabel>
#include <ros/ros.h>

class XBot2StatusWidget : public QWidget
{

    Q_OBJECT

public:

    XBot2StatusWidget(QWidget * parent = nullptr);

    void update();

signals:

    void xbot2Started();

private:

    void handleStatusLabel();

    ros::Time _last_status_recv;

    QLCDNumber * _lcd;
    QLabel * _status_label;
    QPushButton * _cmd_button;

    ros::NodeHandle _nh;
    ros::Subscriber _status_sub, _vbatt_sub;
    ros::ServiceClient _srv_start, _srv_stop;
};

#endif // XBOT2_STATUS_WID_H
