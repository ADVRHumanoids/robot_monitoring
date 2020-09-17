#include "joint_monitor_widget.h"
#include "bar_plot_widget.h"
#include "xbot2_wid.h"
#include <QHBoxLayout>

#include <urdf_parser/urdf_parser.h>



JointMonitorWidget::JointMonitorWidget(QWidget *parent) :
    QWidget(parent),
    _valid_msg_recv(false),
    _widget_started(false)
{

    ros::NodeHandle nh("xbotcore");
    _jstate_sub = nh.subscribe("joint_states", 10, &JointMonitorWidget::on_jstate_recv, this);
    _fault_sub = nh.subscribe("fault", 10, &JointMonitorWidget::on_fault_recv, this);

    std::string urdf_str;
    nh.getParam("robot_description", urdf_str);

    if(urdf_str.empty())
    {
        throw std::runtime_error("Unable to read robot_description from parameter server");
    }

    _urdf = urdf::parseURDF(urdf_str);

    while(!_valid_msg_recv)
    {
        ros::spinOnce();
        usleep(1000);
    }

    std::string jidmap_str = nh.param<std::string>("robot_description_joint_id_map", "");
    if(!jidmap_str.empty())
    {
        try
        {
            auto jidmap_yaml = YAML::Load(jidmap_str);
            for(auto p: jidmap_yaml["joint_map"])
            {
                _jidmap[p.second.as<std::string>()] = p.first.as<int>();
            }
        }
        catch(std::exception& e)
        {
            fprintf(stderr, "Unable to get joint IDs: %s \n", e.what());
        }
    }

    cartesio_gui::SlidersWidgetMainView::Options opt;
    opt.message_type = "xbot_msgs";
    opt.ns = "xbotcore";
    _sliders = new cartesio_gui::SlidersWidgetMainView(opt);
    _sliders->setFixedWidth(400);

    jstate_wid = new JointStateWidget(this);
    jstate_wid->setJointName(QString::fromStdString(_jnames[0]), 0);

    barplot_wid = new BarPlotWidget(_jnames, this);

    auto on_joint_clicked = [this](std::string jname)
    {
        auto wid = barplot_wid->wid_map.at(jstate_wid->getJointName().toStdString());
        wid->setInactive();
        int jid = _jidmap.count(jname) > 0 ? _jidmap.at(jname) : 0;
        jstate_wid->setJointName(QString::fromStdString(jname), jid);
        wid = barplot_wid->wid_map.at(jname);
        wid->setActive();
        _sliders->makeJointVisible(QString::fromStdString(jname));
    };

    on_joint_clicked(_jnames[0]);

    for(auto p: barplot_wid->wid_map)
    {
        connect(p.second, &JointBarWidget::doubleLeftClicked,
                [p, on_joint_clicked](){ on_joint_clicked(p.first); });

        connect(p.second, &JointBarWidget::doubleRightClicked,
                [p, this]()
                {

                    _chart->addSeries(p.second->getJointName() +
                                      "/" +
                                      QString::fromStdString(barplot_wid->getFieldShortType()));
                });
    }



    _chart = new ChartWidget;
    _chart->setMinimumSize(640, 400);

    auto layout = new QHBoxLayout(this);

    auto vlayout_left = new QVBoxLayout(this);
    vlayout_left->addWidget(new XBot2Widget(this));
    vlayout_left->addWidget(barplot_wid);
    layout->addLayout(vlayout_left);


    auto vlayout = new QVBoxLayout(this);
    auto hlayout = new QHBoxLayout(this);
    hlayout->addWidget(jstate_wid);
    hlayout->addWidget(_sliders);

    vlayout->addLayout(hlayout);
    vlayout->addWidget(_chart);

    layout->addLayout(vlayout);
    setLayout(layout);

    connect(jstate_wid, &JointStateWidget::plotAdded,
            [this](QString plot)
            {
                _chart->addSeries(jstate_wid->getJointName() + "/" + plot);
            });


    _timer = new QTimer(this);
    _timer->setInterval(40);
    connect(_timer, &QTimer::timeout,
            this, &JointMonitorWidget::on_timer_event);
    _timer->start();

    _widget_started = true;


}

void JointMonitorWidget::on_timer_event()
{
    ros::spinOnce();

    jstate_wid->updateStatus();
    for(auto& p : barplot_wid->wid_map)
    {
        p.second->updateStatus();
    }
}

void JointMonitorWidget::on_jstate_recv(xbot_msgs::JointStateConstPtr msg)
{
    if(!_valid_msg_recv)
    {
        _jnames = msg->name;
        _valid_msg_recv = true;
        return;
    }

    if(!_widget_started)
    {
        return;
    }

    static auto t0 = msg->header.stamp;
    auto now = msg->header.stamp;

    for(int i = 0; i < msg->name.size(); i++)
    {

        double k = msg->stiffness[i];
        double d = msg->damping[i];
        double qerr = msg->position_reference[i] - msg->link_position[i];
        double dqerr = msg->velocity_reference[i] - msg->link_velocity[i];
        double tauref_imp = k*qerr + d*dqerr + msg->effort_reference[i];

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/link_pos",
                         (now - t0).toSec(),
                         msg->link_position[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/motor_pos",
                         (now - t0).toSec(),
                         msg->motor_position[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/pos_ref",
                         (now - t0).toSec(),
                         msg->position_reference[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/link_vel",
                         (now - t0).toSec(),
                         msg->link_velocity[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/motor_vel",
                         (now - t0).toSec(),
                         msg->motor_velocity[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/vel_ref",
                         (now - t0).toSec(),
                         msg->velocity_reference[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/torque",
                         (now - t0).toSec(),
                         msg->effort[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/torque_ffwd",
                         (now - t0).toSec(),
                         msg->effort_reference[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/torque_imp",
                         (now - t0).toSec(),
                         tauref_imp);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/current",
                         (now - t0).toSec(),
                         msg->aux[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/driver_temp",
                         (now - t0).toSec(),
                         msg->temperature_driver[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/motor_temp",
                         (now - t0).toSec(),
                         msg->temperature_motor[i]);




        if(msg->name[i] == jstate_wid->getJointName().toStdString())
        {
            jstate_wid->tor->setValue(msg->effort[i]);
            jstate_wid->torref->setValue(msg->effort_reference[i]);
            jstate_wid->velref->setValue(msg->velocity_reference[i]);
            jstate_wid->posref->setValue(msg->position_reference[i]);
            jstate_wid->motopos->setValue(msg->motor_position[i]);
            jstate_wid->motovel->setValue(msg->motor_velocity[i]);
            jstate_wid->linkpos->setValue(msg->link_position[i]);
            jstate_wid->linkvel->setValue(msg->link_velocity[i]);
            jstate_wid->current->setValue(msg->aux[i]);
            jstate_wid->stiffness->setValue(msg->stiffness[i]);
            jstate_wid->damping->setValue(msg->damping[i]);
            jstate_wid->mototemp->setValue(msg->temperature_motor[i]);
            jstate_wid->drivertemp->setValue(msg->temperature_driver[i]);


            jstate_wid->torref_imp->setValue(tauref_imp);

        }

        if(barplot_wid->getFieldType() == "Temperature")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(std::max(msg->temperature_motor[i],
                                   msg->temperature_driver[i]));
            wid->setRange(30, 90);

        }
        else if(barplot_wid->getFieldType() == "Current")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(std::fabs(msg->aux[i]), msg->aux[i]);
            wid->setRange(0, 60);
        }
        else if(barplot_wid->getFieldType() == "Torque")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(std::fabs(msg->effort[i]), msg->effort[i]);
            double taumax = _urdf->getJoint(msg->name[i])->limits->effort;
            wid->setRange(0, taumax);

        }
        else if(barplot_wid->getFieldType() == "Torque tracking error")
        {
            double k = msg->stiffness[i];
            double d = msg->damping[i];
            double qerr = msg->position_reference[i] - msg->link_position[i];
            double dqerr = msg->velocity_reference[i] - msg->link_velocity[i];
            double tauref_imp = k*qerr + d*dqerr + msg->effort_reference[i];
            double tau_err = tauref_imp - msg->effort[i];

            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(std::fabs(tau_err), tau_err);
            wid->setRange(0, 15.0);

        }
        else if(barplot_wid->getFieldType() == "Link position")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(msg->link_position[i]);
            double qmin = _urdf->getJoint(msg->name[i])->limits->lower;
            double qmax = _urdf->getJoint(msg->name[i])->limits->upper;
            wid->setRange(qmin, qmax);
        }
        else if(barplot_wid->getFieldType() == "Motor position")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(msg->motor_position[i]);
            double qmin = _urdf->getJoint(msg->name[i])->limits->lower;
            double qmax = _urdf->getJoint(msg->name[i])->limits->upper;
            wid->setRange(qmin, qmax);
        }
        else if(barplot_wid->getFieldType() == "Link velocity")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            double vel = msg->link_velocity[i];
            wid->setValue(std::fabs(vel), vel);
            double qdmax = _urdf->getJoint(msg->name[i])->limits->velocity;
            wid->setRange(0, qdmax);
        }
        else if(barplot_wid->getFieldType() == "Motor velocity")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            double vel = msg->motor_velocity[i];
            wid->setValue(std::fabs(vel), vel);
            double qdmax = _urdf->getJoint(msg->name[i])->limits->velocity;
            wid->setRange(0, qdmax);
        }
        else if(barplot_wid->getFieldType() == "Stiffness")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(msg->stiffness[i]);
            wid->setRange(0, 5000);
        }
        else if(barplot_wid->getFieldType() == "Damping")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(msg->damping[i]);
            wid->setRange(0, 100);
        }
    }


}

void JointMonitorWidget::on_fault_recv(xbot_msgs::FaultConstPtr msg)
{
    if(!_widget_started)
    {
        return;
    }

    for(size_t i = 0; i < msg->name.size(); i++)
    {
        auto wid = barplot_wid->wid_map.at(msg->name[i]);
        wid->setDanger();

        if(msg->name[i] == jstate_wid->getJointName().toStdString())
        {
            jstate_wid->setStatus(msg->fault[i]);
        }
    }
}



