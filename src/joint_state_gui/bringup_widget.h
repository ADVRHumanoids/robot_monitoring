#ifndef BRINGUP_WIDGET_H
#define BRINGUP_WIDGET_H

#include <QDialog>
#include <QPushButton>
#include <QTextEdit>
#include <ros/ros.h>

class bringup_widget : public QDialog
{

public:

    bringup_widget(QWidget * parent = nullptr);

private:

    void labelOk(QString name);
    void labelNok(QString name);
    void labelText(QString name, QString text);
    void writeText(QString text);

    void bringup();
    bool wait_service(ros::ServiceClient& s);
    bool get_status(ros::ServiceClient& s);
    bool check_services();
    bool check_status();
    bool start_ecat();
    bool wait_slaves(int& nslaves);
    bool start_xbot();

    QPushButton * _startBtn;

    ros::NodeHandle _nh;
    ros::ServiceClient _ecat_status, _ecat_start, _ecat_get_slaves;
    ros::ServiceClient _xbot_status, _xbot_start;

    QTextEdit * _text;

};

#endif // BRINGUP_WIDGET_H
