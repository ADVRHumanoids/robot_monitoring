#include "xbot2_status_wid.h"
#include "bringup_widget.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>
#include <QMessageBox>
#include <QMenu>
#include <QInputDialog>
#include <QMenuBar>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <xbot_msgs/GetPluginList.h>
#include <xbot_msgs/StartProcess.h>
#include <xbot_msgs/StopProcess.h>

void xbot2_status_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    xbot2_status_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/xbot2_status.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

QWidget * LoadUiFileBringup(QWidget * parent)
{
    xbot2_status_widget_qrc_init();
    QUiLoader loader;

    QFile file(":/bringup.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}

XBot2StatusWidget::XBot2StatusWidget(QMainWindow * mw,
                                     QWidget* parent):
    QWidget (parent),
    _nh("xbotcore"),
    _mw(mw)
{
    auto layout = new QVBoxLayout;
    layout->addWidget(LoadUiFile(this));
    layout->setMargin(0);
    setLayout(layout);

    // status label
    _status_label = findChild<QLabel*>("statusLabelXbot");
    _status_label->setToolTip("xbot2 process status from topic 'xbotcore/status'");
    _status_label->setText("Inactive");

    auto on_status_recv = [this](const std_msgs::StringConstPtr& msg)
    {
        _status_label->setText(QString::fromStdString(msg->data));
        _last_status_recv = ros::Time::now();
        handleStatusLabel();

    };

    _status_sub = _nh.subscribe<std_msgs::String>("status", 1, on_status_recv);

    // cmd button
    _cmd_button = findChild<QPushButton*>("cmdBtn");
    _cmd_button->setToolTip("Start/stop xbot2 process "
                            "(needs xbot2-launcher daemon running)");

    _srv_start = _nh.serviceClient<xbot_msgs::StartProcess>("d/start");
    _srv_stop = _nh.serviceClient<xbot_msgs::StopProcess>("d/stop");

    connect(_cmd_button, &QPushButton::released,
            [this]()
    {
        if(_cmd_button->text() == "Start")
        {
            xbot_msgs::StartProcess srv;

            if(!_hw_type.empty())
            {
                srv.request.args.push_back("--hw");
                srv.request.args.push_back(_hw_type);
            }

            if(_hw_type == "sim" || _hw_type == "gz")
            {
                srv.request.args.push_back("--simtime");
            }

            if(!_srv_start.waitForExistence(ros::Duration(1.0)))
            {
                QMessageBox msgBox;
                msgBox.setText("Start service is offline, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;
            }

            if(!_srv_start.call(srv))
            {
                QMessageBox msgBox;
                msgBox.setText("Start service failed, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;
            }

            if(!srv.response.success)
            {
                QMessageBox msgBox;
                msgBox.setText("Start service returned false: " +
                               QString::fromStdString(srv.response.message));
                msgBox.exec();
                return;
            }

            emit xbot2Started();

        }
        else if(_cmd_button->text() == "Stop")
        {
            xbot_msgs::StopProcess srv;

            if(!_srv_stop.waitForExistence(ros::Duration(1.0)))
            {
                QMessageBox msgBox;
                msgBox.setText("Stop service is offline, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;
            }

            if(!_srv_stop.call(srv))
            {
                QMessageBox msgBox;
                msgBox.setText("Stop service failed, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;
            }

            if(!srv.response.success)
            {
                QMessageBox msgBox;
                msgBox.setText("Stop service returned false: " +
                               QString::fromStdString(srv.response.message));
                msgBox.exec();
                return;
            }
        }
    });

    // kill btn
    auto kill_btn = findChild<QPushButton*>("killBtn");
    connect(kill_btn, &QPushButton::released,
            [this]()
    {
        auto cli = _nh.serviceClient<std_srvs::Trigger>("d/kill");

        std_srvs::Trigger srv;
        if(!cli.waitForExistence(ros::Duration(1.0)))
        {
            QMessageBox msgBox;
            msgBox.setText("Kill service is offline, make sure "
                           "xbot2-launcher daemon is up and running");
            msgBox.exec();
            return;
        }

        if(!cli.call(srv))
        {
            QMessageBox msgBox;
            msgBox.setText("Kill service failed, make sure "
                           "xbot2-launcher daemon is up and running");
            msgBox.exec();
            return;
        }

        if(!srv.response.success)
        {
            QMessageBox msgBox;
            msgBox.setText("Kill service returned false: \n" +
                           QString::fromStdString(srv.response.message));
            msgBox.exec();
            return;
        }

    });

    // vbatt lcd
    auto on_vbatt_recv = [this](const std_msgs::Float32ConstPtr& msg)
    {
        if(msg->data > 100)
        {
            _lcd->display("H1");
        }
        else if (msg->data < 0) {
            _lcd->display("L0");
        }
        else {
            _lcd->display(QString("%1").arg(msg->data, 0, 'f', 1));
        }
    };

    _vbatt_sub = _nh.subscribe<std_msgs::Float32>("vbatt", 1, on_vbatt_recv);

    _lcd = findChild<QLCDNumber*>("voltLcd");
    _lcd->setStyleSheet("border: 0px");
    _lcd->display("00");
    _lcd->setToolTip("Battery voltage from topic 'xbotcore/vbatt'");

    findChild<QLabel*>("voltLabel")->setStyleSheet("font-size: 24px");

    // bringup
    auto bringupBtn = findChild<QPushButton*>("bringupBtn");
    auto bringupBtnClicked = [this]()
    {
        auto bringupWid = new BringupWidget(QString::fromStdString(_hw_type),
                                            this);

        if(bringupWid->exec() == QDialog::Accepted)
        {
            emit xbot2Started();
        }

    };
    connect(bringupBtn, &QPushButton::released, bringupBtnClicked);

    // shutdown
    auto shutdownBtn = findChild<QPushButton*>("shutdownBtn");
    auto shutdownBtnClicked = [this]()
    {
        xbot_msgs::StopProcess srv_xbot;
        srv_xbot.request.signum = 0;
        ros::service::call("/xbotcore/d/stop", srv_xbot);
        std_srvs::Trigger srv_ec;
        ros::service::call("/ecat/d/kill", srv_ec);
    };
    connect(shutdownBtn, &QPushButton::released, shutdownBtnClicked);

    // select hw type with menu entry
    setupHwtypeMenu();

    handleStatusLabel();
}

void XBot2StatusWidget::setupHwtypeMenu()
{
    auto fileMenu = _mw->menuBar()->findChild<QMenu*>("File");

    auto * load_action = new QAction("Select hw profile", this);
    load_action->setStatusTip("Selects the hal hardware type for xbot2 (e.g. dummy, sim, ec, ...)");

    connect(load_action, &QAction::triggered,
            [this]()
    {
        QList<QString> items;

        auto cli = _nh.serviceClient<xbot_msgs::GetPluginList>("d/get_hw_types");

        xbot_msgs::GetPluginList srv;
        if(!cli.waitForExistence(ros::Duration(1.0)))
        {
            QMessageBox msgBox;
            msgBox.setText("get_hw_types service is offline, make sure "
                           "xbot2-launcher daemon is up and running");
            msgBox.exec();
            return;
        }

        if(!cli.call(srv))
        {
            QMessageBox msgBox;
            msgBox.setText("get_hw_types service failed, make sure "
                           "xbot2-launcher daemon is up and running");
            msgBox.exec();
            return;
        }

        items.append("auto");
        std::transform(srv.response.plugins.begin(),
                       srv.response.plugins.end(),
                       std::back_inserter(items),
                       QString::fromStdString);


        auto i = QInputDialog::getItem(this,
                                       "Select profile",
                                       "Profiles",
                                       items);

        if(i == "auto")
        {
            _hw_type.clear();
        }

        if(!i.isEmpty())
        {
            _hw_type = i.toStdString();
        }

    });

    fileMenu->addAction(load_action);

}

void XBot2StatusWidget::update()
{
    if(ros::Time::now() - _last_status_recv > ros::Duration(0.5))
    {
        _status_label->setText("Inactive");
        handleStatusLabel();
    }
}

void XBot2StatusWidget::handleStatusLabel()
{
    if(_status_label->text() == "Running")
    {
        _status_label->setStyleSheet(
                    "font-size: 18px; "
                    "background-color: #99ff99;"
                    "border-radius: 4px;"
                    "color: green");

        _cmd_button->setText("Stop");
    }
    else if(_status_label->text() == "Initializing")
    {
        _status_label->setStyleSheet(
                    "font-size: 18px; "
                    "background-color: #ff9248;"
                    "border-radius: 4px;"
                    "color: black");

        _cmd_button->setText("Stop");
    }
    else {
        _status_label->setStyleSheet(
                    "font-size: 18px; "
                    "background-color: #D3D3D3;"
                    "border-radius: 4px;"
                    "color: gray");

        _cmd_button->setText("Start");
    }
}
