#include "bringup_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>
#include <QDir>
#include <QPushButton>
#include <QLabel>
#include <QCoreApplication>

#include <iostream>

#include <std_srvs/Trigger.h>
#include <ec_srvs/GetSlaveInfo.h>

#include <yaml-cpp/yaml.h>

void bringup_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{

    bringup_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/bringup.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;

}

}


bringup_widget::bringup_widget(QWidget * parent):
    QDialog(parent), _nh(""), _finished(false)
{
    auto wid = LoadUiFile(this);
    auto l = new QHBoxLayout;
    l->addWidget(wid);
    setLayout(l);
    wid->setMinimumWidth(400);

    setWindowTitle("Robot bringup procedure");

    // close button
    auto okBtn = findChild<QPushButton*>("okBtn");
    auto okBtnClicked = [this]()
    {
        _finished = true;
        reject();
    };
    connect(okBtn, &QPushButton::released, okBtnClicked);

    // start button
    _startBtn = findChild<QPushButton*>("startBtn");
    connect(_startBtn, &QPushButton::released, this, 
            [this]()
            {
                if(_startBtn->text() == "Start")
                {
                    _startBtn->setText("Abort");
                    bringup();
                    _startBtn->setText("Start");
                }
                else
                {
                    _finished = true;
                }
            });

    // text area
    _text = findChild<QTextEdit*>("textEdit");

    // ros services
    _ecat_start = _nh.serviceClient<std_srvs::Trigger>("ecat/d/start");
    _ecat_status = _nh.serviceClient<std_srvs::Trigger>("ecat/d/status");
    _ecat_get_slaves = _nh.serviceClient<ec_srvs::GetSlaveInfo>("ec_client/get_slaves_description");
    _xbot_start = _nh.serviceClient<std_srvs::Trigger>("xbotcore/d/start");
    _xbot_status = _nh.serviceClient<std_srvs::Trigger>("xbotcore/d/status");

}

void bringup_widget::labelOk(QString name)
{
    auto label = findChild<QLabel*>(name);
    label->setStyleSheet(
                "color: green;");
}

void bringup_widget::labelNok(QString name)
{
    auto label = findChild<QLabel*>(name);
    label->setStyleSheet(
                "color: red; "
                "font-weight: bold");
}

void bringup_widget::labelText(QString name, QString text)
{
    auto label = findChild<QLabel*>(name);
    label->setText(text);
}

void bringup_widget::writeText(QString text)
{
    _text->moveCursor(QTextCursor::End);
    _text->insertPlainText(text);
    std::cout << text.toStdString();
    repaint();
    QCoreApplication::processEvents();
}

void bringup_widget::bringup()
{
    _text->clear();

    // services existance
    if(!wait_service(_ecat_start) ||
            !wait_service(_ecat_status) ||
            !wait_service(_xbot_start) ||
            !wait_service(_xbot_status) ||
            !wait_service(_ecat_get_slaves)
            )
    {
        labelText("servicesOk", "Failed");
        labelNok("servicesLabel");
        labelNok("servicesOk");
        return;
    }

    labelText("servicesOk", "Ok");
    labelOk("servicesLabel");
    labelOk("servicesOk");

    // check currently running
    if(!check_status())
    {
        labelText("checkOk", "Failed");
        labelNok("checkLabel");
        labelNok("checkOk");
        return;
    }

    labelText("checkOk", "Ok");
    labelOk("checkLabel");
    labelOk("checkOk");

    // start ecat
    if(!start_ecat())
    {
        labelText("ecatOk", "Failed");
        labelNok("ecatLabel");
        labelNok("ecatOk");
        return;
    }

    labelText("ecatOk", "Ok");
    labelOk("ecatLabel");
    labelOk("ecatOk");

    // slaves descr
    int nslaves = -1;
    if(!wait_slaves(nslaves))
    {
        labelText("slaveOk", "Failed");
        labelNok("slaveLabel");
        labelNok("slaveOk");
        return;
    }

    labelText("slaveOk", QString("Ok (%1 slaves)").arg(nslaves));
    labelOk("slaveLabel");
    labelOk("slaveOk");

    // xbot2
    if(!start_xbot())
    {
        labelText("xbot2Ok", "Failed");
        labelNok("xbot2Label");
        labelNok("xbot2Ok");
        return;
    }

    labelText("xbot2Ok", "Ok");
    labelOk("xbot2Label");
    labelOk("xbot2Ok");

    accept();

    return;
}

bool bringup_widget::wait_service(ros::ServiceClient& s)
{
    writeText(QString(">> waiting for service '%1'..").arg(s.getService().c_str()));
    if(s.waitForExistence(ros::Duration(1.0)))
    {
        writeText("..ok \n");
    }
    else
    {
        writeText("..failed, check connection with robot pc \n");
        return false;
    }

    return true;
}


bool bringup_widget::check_services()
{
    
}

bool bringup_widget::check_status()
{
    std_srvs::Trigger srv;

    writeText(">> checking ecat master is not running..");

    if(!_ecat_status.call(srv))
    {
        writeText("..service failed \n");
        return false;
    }

    if(srv.response.success)
    {
        writeText("..ecat master is running, shutdown first \n");
        return false;
    }

    writeText("..ok \n");

    writeText(">> checking xbot2 is not running..");

    if(!_xbot_status.call(srv))
    {
        writeText("..service failed \n");
        return false;
    }

    if(srv.response.success)
    {
        writeText("..xbot2 is running, shutdown first \n");
        return false;
    }

    writeText("..ok \n");

    return true;
}

bool bringup_widget::start_ecat()
{
    std_srvs::Trigger srv;

    writeText(">> starting ecat master..");

    if(!_ecat_start.call(srv))
    {
        writeText("..service failed \n");
        return false;
    }

    if(!srv.response.success)
    {
        writeText("..failed \n");
        return false;
    }

    writeText("..ok \n");

    return true;

}

bool bringup_widget::wait_slaves(int& nslaves)
{
    ec_srvs::GetSlaveInfo srv;

    int attempts = 30;
    bool descr_ok = false;

    while(!descr_ok && attempts--)
    {
        writeText(">> querying slave description..");

        if(!_ecat_get_slaves.call(srv))
        {
            writeText("..service failed \n");
            return false;
        }

        if(_finished)
        {
            writeText("..canceled by user \n");
            return false;
        }

        if(srv.response.cmd_info.status != "FAULT")
        {
            descr_ok = true;
        }
        else
        {
            sleep(1);
            writeText(QString::fromStdString(srv.response.cmd_info.fault_info) + "\n");
        }
        
    }

    if(!descr_ok)
    {
        writeText("..failed after n = 30 attempts \n");
        return false;
    }

    try
    {
        auto y = YAML::Load(srv.response.cmd_info.msg);
        nslaves = y.size();
    }
    catch(YAML::Exception& e)
    {
        writeText("..parsing failed \n");
        return false;
    }

    writeText("..ok \n");

    return true;
}

bool bringup_widget::start_xbot()
{
    std_srvs::Trigger srv;

    writeText(">> starting xbot2..");

    if(!_xbot_start.call(srv))
    {
        writeText("..service failed \n");
        return false;
    }

    if(!srv.response.success)
    {
        writeText("..failed \n");
        return false;
    }

    writeText("..ok \n");

    return true;
}
