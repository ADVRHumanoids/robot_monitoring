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

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <xbot_msgs/GetPluginList.h>

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

XBot2StatusWidget::XBot2StatusWidget(QWidget* parent):
    QWidget (parent),
    _nh("xbotcore")
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

    _srv_start = _nh.serviceClient<std_srvs::Trigger>("d/start");
    _srv_stop = _nh.serviceClient<std_srvs::Trigger>("d/stop");

    connect(_cmd_button, &QPushButton::released,
            [this]()
    {
        if(_cmd_button->text() == "Start")
        {
            std_srvs::Trigger srv;
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
            std_srvs::Trigger srv;
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
        auto bringupWid = new bringup_widget(this);

//        auto geom = bringupWid->geometry();
//        geom.setTopLeft(event->globalPos());
//        _load_plugin_wid->setGeometry(geom);

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
        std_srvs::Trigger srv;
        ros::service::call("/xbotcore/d/stop", srv);
        ros::service::call("/ecat/d/kill", srv);
    };
    connect(shutdownBtn, &QPushButton::released, shutdownBtnClicked);

    handleStatusLabel();
}

void XBot2StatusWidget::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);

    auto * load_action = new QAction("Select profile", this);

    connect(load_action, &QAction::triggered,
            [this, event]()
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

        items.append("Auto");
        std::transform(srv.response.plugins.begin(),
                       srv.response.plugins.end(),
                       std::back_inserter(items),
                       QString::fromStdString);


        auto i = QInputDialog::getItem(this,
                                       "Select profile",
                                       "Profiles",
                                       items);

        if(!i.isEmpty())
        {
            _hw_type = i.toStdString();
        }

    });

    menu.addAction(load_action);

    menu.exec(event->globalPos());
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
