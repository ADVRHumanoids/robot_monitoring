#include "xbot2_wid.h"

#include <xbot_msgs/Statistics2.h>
#include <xbot_msgs/GetPluginList.h>
#include <xbot_msgs/LifecycleEvent.h>
#include <xbot_msgs/SetControlMask.h>
#include <xbot_msgs/JointDeviceInfo.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

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

void xbot2_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {



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

}

XBot2Widget::XBot2Widget(QWidget * parent) :
    QWidget(parent),
    _nh("xbotcore")
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto lay = new QVBoxLayout(this);
    lay->addWidget(ui);
    setLayout(lay);

    /* Add status widget */
    _status_wid = new XBot2StatusWidget;
    auto status_box = findChild<QGroupBox*>("statusBox");
    auto status_layout = new QVBoxLayout;
    status_layout->setMargin(0);
    status_layout->addWidget(_status_wid);
    status_box->setLayout(status_layout);


    /* Connect filter buttons */
    auto enableFilterCheck = findChild<QCheckBox*>("enableFilter");

    auto enable_filt_srv = _nh.serviceClient<std_srvs::SetBool>(
        "enable_joint_filter");

    connect(enableFilterCheck, &QCheckBox::clicked,
            [enable_filt_srv](bool checked) mutable
            {
                std_srvs::SetBool srv_data;
                srv_data.request.data = checked;
                enable_filt_srv.call(srv_data);
            });

    auto safeBtn = findChild<QRadioButton*>("safeBtn");
    auto filt_safe_srv = _nh.serviceClient<std_srvs::Trigger>(
        "set_filter_profile_safe");

    auto mediumBtn = findChild<QRadioButton*>("mediumBtn");
    auto filt_mid_srv = _nh.serviceClient<std_srvs::Trigger>(
        "set_filter_profile_medium");

    auto fastBtn = findChild<QRadioButton*>("fastBtn");
    auto filt_fast_srv = _nh.serviceClient<std_srvs::Trigger>(
        "set_filter_profile_fast");

    safeBtn->setEnabled(false);
    mediumBtn->setEnabled(false);
    fastBtn->setEnabled(false);

    connect(safeBtn, &QRadioButton::clicked,
            [filt_safe_srv](bool checked) mutable
            {
                if(!checked) return;

                std_srvs::Trigger srv_data;
                filt_safe_srv.call(srv_data);
            });

    connect(mediumBtn, &QRadioButton::clicked,
            [filt_mid_srv](bool checked) mutable
            {
                if(!checked) return;

                std_srvs::Trigger srv_data;
                filt_mid_srv.call(srv_data);
            });

    connect(fastBtn, &QRadioButton::clicked,
            [filt_fast_srv](bool checked) mutable
            {
                if(!checked) return;

                std_srvs::Trigger srv_data;
                filt_fast_srv.call(srv_data);
            });


    /* Connect joint disable button */
    auto disableEnableBtn = findChild<QPushButton*>("disableEnableBtn");

    auto setmask_srv = _nh.serviceClient<xbot_msgs::SetControlMask>(
        "joint_master/set_control_mask");

    setmask_srv.waitForExistence();

    connect(disableEnableBtn, &QPushButton::released,
            [disableEnableBtn, setmask_srv]() mutable
            {
                xbot_msgs::SetControlMask srv_data;

                if(disableEnableBtn->text() == "Disable device")
                {
                    srv_data.request.ctrl_mask = 0;
                    if(setmask_srv.call(srv_data) &&
                        srv_data.response.success)
                    {
//                        disableEnableBtn->setText("Enable device");
                    }
                }
                else if(disableEnableBtn->text() == "Enable device")
                {
                    srv_data.request.ctrl_mask = 31;
                    if(setmask_srv.call(srv_data) &&
                        srv_data.response.success)
                    {
//                        disableEnableBtn->setText("Disable device");
                    }
                }
                else {
                    disableEnableBtn->setEnabled(false);
                }
            });


    /* Add plugins */
    auto pluginsLayout = findChild<QVBoxLayout *>("pluginsLayout");
    auto srv = _nh.serviceClient<xbot_msgs::GetPluginList>("get_plugin_list");
    xbot_msgs::GetPluginList srv_data;
    srv.call(srv_data);

    std::vector<ros::ServiceClient> switch_srvs;
    std::vector<ros::ServiceClient> abort_srvs;
    for(auto plname : srv_data.response.plugins)
    {
        auto pl = new XBot2PluginWidget(QString::fromStdString(plname), this);
        pl->setMaximumWidth(500);
        pluginsLayout->addWidget(pl);

        _pl_map[plname] = pl;

        // connect buttons
        auto switch_srv = _nh.serviceClient<std_srvs::SetBool>(
            plname + "/switch");

        switch_srvs.push_back(switch_srv);

        switch_srv.waitForExistence();

        connect(pl, &XBot2PluginWidget::startStopPressed,
                [switch_srv](bool start) mutable
                {
                    std_srvs::SetBool srv_data;
                    srv_data.request.data = start;
                    switch_srv.call(srv_data);
                });

        auto abort_srv = _nh.serviceClient<std_srvs::Trigger>(
            plname + "/abort");

        abort_srvs.push_back(abort_srv);

        abort_srv.waitForExistence();

        connect(pl, &XBot2PluginWidget::abortPressed,
                [abort_srv]() mutable
                {
                    std_srvs::Trigger srv_data;
                    abort_srv.call(srv_data);
                });
    }

    pluginsLayout->setAlignment(Qt::AlignTop);

    /* Stop all */
    auto stopAllBtn = findChild<QPushButton*>("stopAllBtn");
    connect(stopAllBtn, &QPushButton::released,
            [switch_srvs]() mutable
            {
                std_srvs::SetBool srv_data;
                srv_data.request.data = false;
                for(auto& srv : switch_srvs)
                {
                    srv.call(srv_data);
                }
            });

    /* Abort all */
    auto abortAllBtn = findChild<QPushButton*>("abortAllBtn");
    connect(abortAllBtn, &QPushButton::released,
            [abort_srvs]() mutable
            {
                std_srvs::Trigger srv_data;
                for(auto& srv : abort_srvs)
                {
                    srv.call(srv_data);
                }
            });

    /* Listen to statistics */
    using namespace std::chrono;
    using clock = std::chrono::high_resolution_clock;
    using namespace std::chrono_literals;
    static auto last_load_upd = clock::now();
    auto on_stats_recv = [this](xbot_msgs::Statistics2ConstPtr msg)
    {
        auto now = clock::now();
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
    };

    _stats_sub = _nh.subscribe<xbot_msgs::Statistics2>("statistics",
                                                      1,
                                                      on_stats_recv);

    /* Listen to joint dev info */
    auto on_jdinfo_recv =
        [enableFilterCheck,
         disableEnableBtn,
         safeBtn,
         mediumBtn,
         fastBtn]
        (const xbot_msgs::JointDeviceInfoConstPtr& msg)
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
            disableEnableBtn->text() == "Disable device")
        {
            disableEnableBtn->setText("Enable device");
            disableEnableBtn->setStyleSheet(
                "background-color: red;"
                "color: white;");
        }
        else if(msg->mask > 0 &&
                 disableEnableBtn->text() == "Enable device")
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
    };

    _jdinfo_sub = _nh.subscribe<xbot_msgs::JointDeviceInfo>(
        "joint_device_info",
        1,
        on_jdinfo_recv);

}

void XBot2Widget::update()
{
    _status_wid->update();
}
