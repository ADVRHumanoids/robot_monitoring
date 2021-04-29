#ifndef BRINGUP_WIDGET_H
#define BRINGUP_WIDGET_H

#include <QDialog>
#include <QPushButton>
#include <QTextEdit>
#include <ros/ros.h>
#include <QThread>
#include <atomic>
#include <ros/callback_queue.h>

class BringupThread : public QThread
{

    Q_OBJECT

public:

    BringupThread();
    QString hw;
    std::atomic_bool ok;

signals:

    void writeText(const QString&);
    void labelOk(QString name);
    void labelWarn(QString name);
    void labelNok(QString name);
    void labelText(QString name, QString text);
    void resultReady(bool);

private:

    void run() override;

    void bringup();
    bool wait_service(ros::ServiceClient& s);
    bool get_status(ros::ServiceClient& s);
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

    BringupWidget(QString hwtype,
                  QWidget * parent = nullptr);

private:

    void start_worker();
    void stop_worker();

    void writeText(QString text);
    void labelOk(QString name);
    void labelWarn(QString name);
    void labelNok(QString name);
    void labelText(QString name, QString text);

    QString _hw;
    BringupThread * _worker;
    QPushButton * _startBtn;
    QTextEdit * _text;
    bool _worker_success;

    ros::CallbackQueue _cbq;
    ros::NodeHandle _nh;
    ros::Subscriber _stderr_sub;
    QTimer * _timer;



};

#endif // BRINGUP_WIDGET_H
