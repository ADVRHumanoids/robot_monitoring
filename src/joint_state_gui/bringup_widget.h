#ifndef BRINGUP_WIDGET_H
#define BRINGUP_WIDGET_H

#include <QDialog>
#include <QPushButton>
#include <QTextEdit>
#include <ros/ros.h>
#include <QThread>

class BringupThread : public QThread
{

    Q_OBJECT

public:

    BringupThread();

signals:

    void writeText(const QString&);
    void labelOk(QString name);
    void labelNok(QString name);
    void labelText(QString name, QString text);
    void resultReady(bool);

private:

    void run() override;

    void bringup();
    bool wait_service(ros::ServiceClient& s);
    bool get_status(ros::ServiceClient& s);
    bool check_services();
    bool check_status();
    bool start_ecat();
    bool wait_slaves(int& nslaves);
    bool start_xbot();

    ros::NodeHandle _nh;
    ros::ServiceClient _ecat_status, _ecat_start, _ecat_get_slaves;
    ros::ServiceClient _xbot_status, _xbot_start;

};

class BringupWidget : public QDialog
{

public:

    BringupWidget(QWidget * parent = nullptr);

private:

    void start_worker();
    void stop_worker();

    void writeText(QString text);
    void labelOk(QString name);
    void labelNok(QString name);
    void labelText(QString name, QString text);

    BringupThread * _worker;
    QPushButton * _startBtn;
    QTextEdit * _text;
    bool _worker_success;



};

#endif // BRINGUP_WIDGET_H
