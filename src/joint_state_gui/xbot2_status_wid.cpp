#include "xbot2_status_wid.h"
//#include "bringup_widget.h"

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

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <xbot_msgs/srv/get_plugin_list.hpp>
#include <xbot_msgs/srv/start_process.hpp>
#include <xbot_msgs/srv/stop_process.hpp>

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

// QWidget * LoadUiFileBringup(QWidget * parent)
// {
//     xbot2_status_widget_qrc_init();
//     QUiLoader loader;

//     QFile file(":/bringup.ui");
//     file.open(QFile::ReadOnly);

//     QWidget *formWidget = loader.load(&file, parent);
//     file.close();

//     return formWidget;
// }

}

XBot2StatusWidget::XBot2StatusWidget(QMainWindow * mw,
                                     QWidget* parent,
                                     rclcpp::Node::SharedPtr node) :
    QWidget (parent),
    _node(node),
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

    auto on_status_recv = [this](const std_msgs::msg::String& msg)
    {
        _status_label->setText(QString::fromStdString(msg.data));
        _last_status_recv = _node->get_clock()->now();
        handleStatusLabel();

    };

    _status_sub = _node->create_subscription<std_msgs::msg::String>("/xbotcore/status", 1, on_status_recv);

    // cmd button
    _cmd_button = findChild<QPushButton*>("cmdBtn");
    _cmd_button->setToolTip("Start/stop xbot2 process "
                            "(needs xbot2-launcher daemon running)");

    _srv_start = _node->create_client<xbot_msgs::srv::StartProcess>("d/start");
    _srv_stop = _node->create_client<xbot_msgs::srv::StopProcess>("d/stop");

    connect(_cmd_button, &QPushButton::released,
            [this]()
    {
        if(_cmd_button->text() == "Start")
        {
            auto request = std::make_shared<xbot_msgs::srv::StartProcess::Request>();

            if(!_hw_type.empty())
            {
                request->args.push_back("--hw");
                request->args.push_back(_hw_type);
            }

            if(_hw_type == "sim" || _hw_type == "gz")
            {
                request->args.push_back("--simtime");
            }

            if(!_srv_start->wait_for_service(1s))
            {
                QMessageBox msgBox;
                msgBox.setText("Start service is offline, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;
            }

            auto result = _srv_start->async_send_request(request);

            if (rclcpp::spin_until_future_complete(_node, result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                QMessageBox msgBox;
                msgBox.setText("Start service failed, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;


            } else if (!result.get()->success)
            {
                QMessageBox msgBox;
                msgBox.setText("Start service returned false: " +
                               QString::fromStdString(result.get()->message));
                msgBox.exec();
                return;
            }

            emit xbot2Started();

        }
        else if(_cmd_button->text() == "Stop")
        {
            auto request = std::make_shared<xbot_msgs::srv::StopProcess::Request>();

            if(!_srv_stop->wait_for_service(1s))
            {
                QMessageBox msgBox;
                msgBox.setText("Stop service is offline, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;
            }

            auto result = _srv_stop->async_send_request(request);

            if (rclcpp::spin_until_future_complete(_node, result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                QMessageBox msgBox;
                msgBox.setText("Stop service failed, make sure "
                               "xbot2-launcher daemon is up and running");
                msgBox.exec();
                return;

            } else if (!result.get()->success) {

                QMessageBox msgBox;
                msgBox.setText("Stop service returned false: " +
                               QString::fromStdString(result.get()->message));
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
        auto cli = _node->create_client<std_srvs::srv::Trigger>("d/kill");

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        if(!cli->wait_for_service(1s))
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.setText("Kill service is offline, make sure "
                           "xbot2-launcher daemon is up and running");
            msgBox.exec();
            return;
        }

        auto result = cli->async_send_request(request);

        if(rclcpp::spin_until_future_complete(_node, result) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.setText("Kill service failed, make sure "
                           "xbot2-launcher daemon is up and running");
            msgBox.exec();
            return;

        } else if(!result.get()->success)
        {
            QMessageBox msgBox;
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.setText("Kill service returned false: \n" +
                           QString::fromStdString(result.get()->message));
            msgBox.exec();
            return;
        }

    });

    // vbatt lcd
    auto on_vbatt_recv = [this](const std_msgs::msg::Float32& msg)
    {
        if(msg.data > 100)
        {
            _lcd->display("H1");
        }
        else if (msg.data < 0) {
            _lcd->display("L0");
        }
        else {
            _lcd->display(QString("%1").arg(msg.data, 0, 'f', 1));
        }
    };

    _vbatt_sub = _node->create_subscription<std_msgs::msg::Float32>("/xbotcore/vbatt", 1, on_vbatt_recv);

    _lcd = findChild<QLCDNumber*>("voltLcd");
    _lcd->setStyleSheet("border: 0px");
    _lcd->display("00");
    _lcd->setToolTip("Battery voltage from topic 'xbotcore/vbatt'");

    findChild<QLabel*>("voltLabel")->setStyleSheet("font-size: 24px");

    // bringup
    // auto bringupBtn = findChild<QPushButton*>("bringupBtn");
    // auto bringupBtnClicked = [this]()
    // {
    //     auto bringupWid = new BringupWidget(QString::fromStdString(_hw_type),
    //                                         this);

    //     if(bringupWid->exec() == QDialog::Accepted)
    //     {
    //         emit xbot2Started();
    //     }

    // };
    // connect(bringupBtn, &QPushButton::released, bringupBtnClicked);

    // shutdown
    auto shutdownBtn = findChild<QPushButton*>("shutdownBtn");
    auto shutdownBtnClicked = [this]()
    {
        rclcpp::Client<xbot_msgs::srv::StopProcess>::SharedPtr cli_stop =
            _node->create_client<xbot_msgs::srv::StopProcess>("/xbotcore/d/stop");

        auto request_stop = std::make_shared<xbot_msgs::srv::StopProcess::Request>();
        request_stop->signum = 0;

        while (!cli_stop->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_stop = cli_stop->async_send_request(request_stop);
        QString xb_str;

        if (rclcpp::spin_until_future_complete(_node, result_stop) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result_stop.get()->success) {
                xb_str = "<span style=\"color: green;\">xbot2 was stopped succesfully: </span>";
            } else {
                xb_str = "<span style=\"color: red;\">xbot2 could not be stopped: </span>";
            }
            
        } else {
            xb_str = "<span style=\"color: red;\">service failed: </span>";
        }

        xb_str += QString::fromStdString(result_stop.get()->message);


        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_kill =
            _node->create_client<std_srvs::srv::Trigger>("/ecat/d/kill");

        auto request_kill = std::make_shared<std_srvs::srv::Trigger::Request>();

        while (!cli_kill->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_kill = cli_kill->async_send_request(request_kill);
        QString ec_str;

        if (rclcpp::spin_until_future_complete(_node, result_kill) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result_kill.get()->success) {
                ec_str = "<span style=\"color: green;\">ecat master was stopped succesfully: </span>";
            } else {
                ec_str = "<span style=\"color: red;\">ecat master could not be stopped: </span>";
            }
            
        } else {
            ec_str = "<span style=\"color: red;\">service failed: </span>";
        }

        ec_str += QString::fromStdString(result_kill.get()->message);

        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Information);
        msgBox.setText("Shutdown request completed");
        msgBox.setInformativeText(xb_str + "<br><br>" + ec_str);
        msgBox.exec();


    };
    connect(shutdownBtn, &QPushButton::released, shutdownBtnClicked);

    // select hw type with menu entry
    handleStatusLabel();
}

void XBot2StatusWidget::update()
{
    if( _node->get_clock()->now() - _last_status_recv > rclcpp::Duration(0, 0.5e9))
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
        _cmd_button->setEnabled(true);
    }
    else if(_status_label->text() == "Initializing")
    {
        _status_label->setStyleSheet(
                    "font-size: 18px; "
                    "background-color: #ff9248;"
                    "border-radius: 4px;"
                    "color: black");

        _cmd_button->setText("Stop");
        _cmd_button->setEnabled(true);
    }
    else {
        _status_label->setStyleSheet(
                    "font-size: 18px; "
                    "background-color: #D3D3D3;"
                    "border-radius: 4px;"
                    "color: gray");

        _cmd_button->setEnabled(false);
    }
}
