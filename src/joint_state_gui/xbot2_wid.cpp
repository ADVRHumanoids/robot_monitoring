#include "xbot2_wid.h"

#include <xbot_msgs/msg/statistics2.hpp>
#include <xbot_msgs/srv/get_plugin_list.hpp>
#include <xbot_msgs/msg/lifecycle_event.hpp>
#include <xbot_msgs/srv/set_control_mask.hpp>
#include <xbot_msgs/msg/joint_device_info.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
//#include <rosgraph_msgs/Log.h>

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>
#include <QCheckBox>
#include <QRadioButton>
#include <QGroupBox>
#include <QTextEdit>
#include <QDateTime>
#include <QCoreApplication>

void xbot2_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    xbot2_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/xbot2.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

QWidget * LoadConsoleUiFile(QWidget * parent)
{
    QUiLoader loader;

    QFile file(":/console.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}

class ClickableConsoleWidget : public QWidget
{

public:

    ClickableConsoleWidget(QWidget * parent) : QWidget(parent)
    {
        _console = LoadConsoleUiFile(this);
        auto l = new QVBoxLayout;
        l->setMargin(0);
        l->addWidget(_console);
        setLayout(l);
    }

    // QWidget interface
protected:

    QWidget * _console;

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if(event->button() == Qt::LeftButton)
        {
            setStyleSheet("");
        }
    }
};

XBot2Widget::XBot2Widget(QMainWindow * mw, QWidget * parent, rclcpp::Node::SharedPtr node) :
    QWidget(parent),
    _node(std::move(node))
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto lay = new QVBoxLayout(this);
    lay->addWidget(ui);
    setLayout(lay);

    /* Add status widget */
    _status_wid = new XBot2StatusWidget(mw, nullptr, _node);

    auto status_box = findChild<QGroupBox*>("statusBox");
    auto status_layout = new QVBoxLayout;
    status_layout->setMargin(0);
    status_layout->addWidget(_status_wid);
    status_box->setLayout(status_layout);


    /* Connect filter buttons */
    auto enableFilterCheck = findChild<QCheckBox*>("enableFilter");

    auto enable_filt_srv = _node->create_client<std_srvs::srv::SetBool>(
        "enable_joint_filter");

    connect(enableFilterCheck, &QCheckBox::clicked,
        [enable_filt_srv, &_node = _node](bool checked) mutable
        {
            auto srv_data = std::make_shared<std_srvs::srv::SetBool::Request>();
            srv_data->data = checked;
            auto result = enable_filt_srv->async_send_request(srv_data);
            // Wait for the result.
            if (rclcpp::spin_until_future_complete(_node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
            } else {
                RCLCPP_ERROR(_node->get_logger(), "Failed to call service enable_joint_filter");
            }
        });

    auto safeBtn = findChild<QRadioButton*>("safeBtn");
    auto filt_safe_srv = _node->create_client<std_srvs::srv::Trigger>(
        "set_filter_profile_safe");

    auto mediumBtn = findChild<QRadioButton*>("mediumBtn");
    auto filt_mid_srv = _node->create_client<std_srvs::srv::Trigger>(
        "set_filter_profile_medium");

    auto fastBtn = findChild<QRadioButton*>("fastBtn");
    auto filt_fast_srv = _node->create_client<std_srvs::srv::Trigger>(
        "set_filter_profile_fast");

    safeBtn->setEnabled(false);
    mediumBtn->setEnabled(false);
    fastBtn->setEnabled(false);
    connect(safeBtn, &QRadioButton::clicked,
        [filt_safe_srv, &_node = _node](bool checked) mutable
        {
            if(!checked) return;

            auto srv_data = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = filt_safe_srv->async_send_request(srv_data);
            if (rclcpp::spin_until_future_complete(_node, result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_filter_profile_safe");
            }
        });

    connect(mediumBtn, &QRadioButton::clicked,
        [filt_mid_srv, &_node = _node](bool checked) mutable
        {
            if(!checked) return;

            auto srv_data = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = filt_mid_srv->async_send_request(srv_data);
            if (rclcpp::spin_until_future_complete(_node, result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_filter_profile_medium");
            }
        });

    connect(fastBtn, &QRadioButton::clicked,
        [filt_fast_srv, &_node = _node](bool checked) mutable
        {
            if(!checked) return;

            auto srv_data = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result = filt_fast_srv->async_send_request(srv_data);
            if (rclcpp::spin_until_future_complete(_node, result) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(_node->get_logger(), "Failed to call service set_filter_profile_fast");
            }
        });


    /* Connect joint disable button */
    auto disableEnableBtn = findChild<QPushButton*>("disableEnableBtn");

    auto setmask_srv = _node->create_client<xbot_msgs::srv::SetControlMask>(
        "/joint_master/set_control_mask");
    //setmask_srv.waitForExistence();

    connect(disableEnableBtn, &QPushButton::released,
        [disableEnableBtn, setmask_srv, &_node = _node]() mutable
        {
            auto srv_data = std::make_shared<xbot_msgs::srv::SetControlMask::Request>();

            if(disableEnableBtn->text().replace('&', "") == "Disable device")
            {
                srv_data->ctrl_mask = 0;
                auto result = setmask_srv->async_send_request(srv_data);
                if (rclcpp::spin_until_future_complete(_node, result) ==
                    rclcpp::FutureReturnCode::SUCCESS &&
                    result.get()->success)
                {
                    // disableEnableBtn->setText("Enable device");
                } 
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disable the device");
                }
            }
            else if(disableEnableBtn->text().replace('&', "") == "Enable device")
            {
                srv_data->ctrl_mask = 31;
                auto result = setmask_srv->async_send_request(srv_data);
                if (rclcpp::spin_until_future_complete(_node, result) ==
                    rclcpp::FutureReturnCode::SUCCESS &&
                    result.get()->success)
                {
                    // disableEnableBtn->setText("Disable device");
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable the device");
                }
            }
            else {
                disableEnableBtn->setEnabled(false);

                srv_data->ctrl_mask = 31;
                auto result = setmask_srv->async_send_request(srv_data);
                if (rclcpp::spin_until_future_complete(_node, result) ==
                    rclcpp::FutureReturnCode::SUCCESS &&
                    result.get()->success)
                {
                    // disableEnableBtn->setText("Disable device");
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable the device");
                }
            }
        });

    /* Add plugins */
    auto pluginsLayout = findChild<QVBoxLayout *>("pluginsLayout");
    auto client = _node->create_client<xbot_msgs::srv::GetPluginList>("/xbotcore/get_plugin_list");
    auto request = std::make_shared<xbot_msgs::srv::GetPluginList::Request>();

    auto srv_data = client->async_send_request(request);
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
          throw std::runtime_error("Service interrupted while waiting for get_plugin_list");
        }
        RCLCPP_INFO(_node->get_logger(), "service /xbotcore/get_plugin_list not available, waiting again...");
      }

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(_node, srv_data) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to call service get_plugin_list");
        throw std::runtime_error("Failed to call service get_plugin_list");
    }

    std::vector<std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>>> switch_srvs;
    std::vector<std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>>> abort_srvs;    
    auto plugin_names = srv_data.get()->plugins; //keep auto here for sake Christ!

    for(const auto& plname : plugin_names)
    {
        auto pl = new XBot2PluginWidget(QString::fromStdString(plname), this);
        pl->setMaximumWidth(500);
        pluginsLayout->addWidget(pl);

        _pl_map[plname] = pl;

        // connect buttons
        auto switch_srv = _node->create_client<std_srvs::srv::SetBool>(
            "/xbotcore/" + plname + "/switch");

        switch_srvs.push_back(switch_srv);

        while (!switch_srv->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service switch_srv. Exiting.");
                throw(std::runtime_error("Interrupted while waiting for the service switch_srv. Exiting."));
            }
            RCLCPP_INFO(_node->get_logger(), "service %s/switch not available, waiting again...", plname.c_str());
        }

        connect(pl, &XBot2PluginWidget::startStopPressed,
            [switch_srv, &_node = _node](bool start) mutable
            {
                auto srv_data = std::make_shared<std_srvs::srv::SetBool::Request>();
                srv_data->data = start;
                auto result = switch_srv->async_send_request(srv_data);
                if (rclcpp::spin_until_future_complete(_node, result) !=
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(_node->get_logger(), "Failed to call service switch");
                }
            }
        );

        auto abort_srv = _node->create_client<std_srvs::srv::Trigger>(
             "/xbotcore/" + plname + "/abort");

        abort_srvs.push_back(abort_srv);

        while (!abort_srv->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service abort_srv. Exiting.");
                throw(std::runtime_error("Interrupted while waiting for the service abort_srv. Exiting."));
            }
            RCLCPP_INFO(_node->get_logger(),  "service %s/abort not available, waiting again...", plname.c_str());
        }

        connect(pl, &XBot2PluginWidget::abortPressed,
                [abort_srv, &_node = _node]() mutable
                {
                    auto srv_data = std::make_shared<std_srvs::srv::Trigger::Request>();
                    auto result = abort_srv->async_send_request(srv_data);
                    if (rclcpp::spin_until_future_complete(_node, result) !=
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_ERROR(_node->get_logger(), "Failed to call service abort");
                    }
                });
    }

    pluginsLayout->setAlignment(Qt::AlignTop);

    /* Stop all */
    auto stopAllBtn = findChild<QPushButton*>("stopAllBtn");
    connect(stopAllBtn, &QPushButton::released,
        [switch_srvs, &_node = _node]() mutable
        {
            for(auto& srv : switch_srvs)
            {
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = false;
                auto result = srv->async_send_request(request);
                if (rclcpp::spin_until_future_complete(_node, result) != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(_node->get_logger(), "Failed to call service stopAll");
                }
            }
        });

    /* Abort all */
    auto abortAllBtn = findChild<QPushButton*>("abortAllBtn");
    connect(abortAllBtn, &QPushButton::released,
        [abort_srvs, &_node = _node]() mutable
        {
            for(auto& srv : abort_srvs)
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto result = srv->async_send_request(request);
                if (rclcpp::spin_until_future_complete(_node, result) != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(_node->get_logger(), "Failed to call service abortAll");
                }
            }
        });

    /* Listen to statistics */
    using namespace std::chrono;
    //using clock = std::chrono::high_resolution_clock;
    using namespace std::chrono_literals;
    static auto last_load_upd = _node->get_clock()->now();
    auto on_stats_recv = 

    _stats_sub = _node->create_subscription<xbot_msgs::msg::Statistics2>(
        "/xbotcore/statistics",
        1,
        [this](const xbot_msgs::msg::Statistics2::SharedPtr msg)
        {
            auto now = _node->get_clock()->now();
            bool load_upd_done = false;

            for(auto task_stats : msg->task_stats)
            {
                auto it = _pl_map.find(task_stats.name);

                if(it == _pl_map.end())
                {
                    continue;
                }

                it->second->setStatus(QString::fromStdString(
                    task_stats.state));

                if(now < last_load_upd + 1s) continue;

                auto th_name = task_stats.thread;

                auto th_it = std::find_if(msg->thread_stats.begin(),
                                        msg->thread_stats.end(),
                                        [th_name](const auto& item)
                                        {
                                            return item.name == th_name;
                                        });

                if(th_it != msg->thread_stats.end())
                {
                    double load = task_stats.run_time /
                                th_it->expected_period;

                    it->second->setLoad(load, task_stats.run_time*1000);
                }

                load_upd_done = true;

            }

            if(load_upd_done) last_load_upd = now;
        }
    );


    _jdinfo_sub = _node->create_subscription<xbot_msgs::msg::JointDeviceInfo>(
        "/xbotcore/joint_device_info",
        1,
        [enableFilterCheck, disableEnableBtn, safeBtn, mediumBtn, fastBtn] (const xbot_msgs::msg::JointDeviceInfo::SharedPtr msg)
        {
            if(msg->filter_active &&
                !enableFilterCheck->isChecked())
            {
                enableFilterCheck->setChecked(true);
            }

            if(!msg->filter_active &&
                enableFilterCheck->isChecked())
            {
                enableFilterCheck->setChecked(false);
            }

            safeBtn->setEnabled(msg->filter_active);
            mediumBtn->setEnabled(msg->filter_active);
            fastBtn->setEnabled(msg->filter_active);

            if(msg->mask == 0 &&
                disableEnableBtn->text().replace('&', "") == "Disable device")
            {
                disableEnableBtn->setText("Enable device");
                disableEnableBtn->setStyleSheet(
                    "background-color: red;"
                    "color: white;");
            }
            else if(msg->mask > 0 &&
                    disableEnableBtn->text().replace('&', "") == "Enable device")
            {
                disableEnableBtn->setText("Disable device");
                disableEnableBtn->setStyleSheet("");
            }

            if(msg->filter_cutoff_hz < 3.0)
            {
                if(!safeBtn->isChecked())
                    safeBtn->setChecked(true);
            }
            else if(msg->filter_cutoff_hz < 10.0)
            {
                if(!mediumBtn->isChecked())
                    mediumBtn->setChecked(true);
            }
            else if(!fastBtn->isChecked())
            {
                fastBtn->setChecked(true);
            }
        }
    );

    /* Error messages */
    auto btmRowLayout = findChild<QHBoxLayout*>("btmRowLayout");
    auto console_wid = new ClickableConsoleWidget(this);
    console_wid->setStatusTip("Click on the error message console "
"title to acknowledge the error");
    btmRowLayout->addWidget(console_wid);
    btmRowLayout->setStretch(0, 0);
    btmRowLayout->setStretch(1, 1);

    // auto console = findChild<QTextEdit*>("textEdit");

    // auto stderr_cb = [console, console_wid](rosgraph_msgs::LogConstPtr msg)
    // {
    //     QDateTime stamp;
    //     stamp.setMSecsSinceEpoch(msg->header.stamp.toSec()*1000);

    //     console->moveCursor(QTextCursor::End);
    //     console->insertPlainText(QString("[%1]").arg(stamp.toString("hh:mm:ss")));
    //     console->insertPlainText("[xbot2]");
    //     console->insertPlainText(QString::fromStdString(msg->msg));
    //     console->insertPlainText("\n");

    //     console_wid->setStyleSheet("background-color: #ff4500;");

    //     QCoreApplication::processEvents();

    // };


    //_stderr_sub = _node.subscription<rosgraph_msgs::msg::log>("d/stderr", 100, stderr_cb);

}

void XBot2Widget::update()
{
    _status_wid->update();
}
