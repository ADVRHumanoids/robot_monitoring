#include "joint_monitor_widget.h"
#include "bar_plot_widget.h"
#include <QHBoxLayout>
#include <QSplitter>

#include <urdf_parser/urdf_parser.h>
#include <memory>
#include "robot_monitoring/context.h"

#include <fmt/format.h>

JointMonitorWidget::JointMonitorWidget(int argc,
                                       char ** argv,
                                       QWidget *parent) :
    QWidget(parent),
    _valid_msg_recv(false),
    _widget_started(false)
{
    /* Create context */
    _ctx = std::make_shared<XBot::Ui::Context>();

    // subscribe to joint states, fault, aux
    ros::NodeHandle nh("xbotcore");

    _jstate_sub = nh.subscribe("joint_states",
                               10,
                               &JointMonitorWidget::on_jstate_recv,
                               this);

    // wait for a first valid joint state message
    int attempts = 100;
    while(!_valid_msg_recv && attempts--)
    {
        ros::spinOnce();
        usleep(10000);
    }

    // xbot2 not alive, only run xbot2 status widget
    if(!_valid_msg_recv)
    {
        // main layout
        auto layout = new QHBoxLayout(this);
        _xbot2_status = new XBot2StatusWidget(this);
        _xbot2_status->setMinimumSize(600, 200);
        layout->addWidget(_xbot2_status);

        // upon successful xbot2 start, we self-restart to get
        // the other widgets as well
        connect(_xbot2_status, &XBot2StatusWidget::xbot2Started,
                [argc, argv]()
        {
            // copy provided argv, and append a final null entry
            // this is needed by execv
            std::vector<char *> args_vec;

            for(int i = 0; i < argc; i++)
            {
                args_vec.push_back(argv[i]);
            }

            args_vec.push_back(static_cast<char *>(nullptr));

            // restart self
            execv("/proc/self/exe", args_vec.data());

        });

        return;
    }

    // if joint state received, go on constructing the whole gui
    _fault_sub = nh.subscribe("fault",
                              10, &
                              JointMonitorWidget::on_fault_recv,
                              this);

    // get robot description
    std::string urdf_str;
    nh.getParam("robot_description", urdf_str);

    if(urdf_str.empty())
    {
        throw std::runtime_error("Unable to read robot_description from parameter server");
    }

    // parse urdf
    _urdf = urdf::parseURDF(urdf_str);

    // try to load a joint id map
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

    // create sliders widget
    cartesio_gui::SlidersWidgetMainView::Options opt;
    opt.message_type = "xbot_msgs";
    opt.ns = "xbotcore";
    _sliders = new cartesio_gui::SlidersWidgetMainView(opt);
    _sliders->setFixedWidth(400);

    // create joint state widget
    jstate_wid = new JointStateWidget(this);
    jstate_wid->setJointName(QString::fromStdString(_jnames[0]), 0);

    // create joint bar plot widget
    barplot_wid = new BarPlotWidget(_jnames, this);

    // define a callback to be executed whenever a joint is
    // double-clicked in the bar plot
    auto on_joint_clicked = [this](std::string jname)
    {
        // remove bold name from currently active joint
        auto wid = barplot_wid->wid_map.at(jstate_wid->getJointName().toStdString());
        wid->setInactive();

        // make clicked joint visible in the joint state widget
        int jid = _jidmap.count(jname) > 0 ? _jidmap.at(jname) : 0;
        jstate_wid->setJointName(QString::fromStdString(jname), jid);

        // add bold text to new active joint
        wid = barplot_wid->wid_map.at(jname);
        wid->setActive();

        // ask slider widget to make the new active joint visible
        _sliders->makeJointVisible(QString::fromStdString(jname));
    };

    // run the callback for the first joint, to initialize the system
    on_joint_clicked(_jnames[0]);

    // connect the callback
    for(auto p: barplot_wid->wid_map)
    {
        connect(p.second, &JointBarWidget::doubleLeftClicked,
                [p, on_joint_clicked](){ on_joint_clicked(p.first); });

        // on double right click, add series to chart
        connect(p.second, &JointBarWidget::doubleRightClicked,
                [p, this]()
        {

            _chart->addSeries(p.second->getJointName() +
                              "/" +
                              QString::fromStdString(barplot_wid->getFieldShortType()));
        });
    }

    // create chart widget
    _chart = new QCustomChart;

    // place all widget into a proper layout

    // main layout (horizontal)
    auto layout = new QHBoxLayout(this);

    auto main_splitter = new QSplitter;
    main_splitter->setOrientation(Qt::Orientation::Horizontal);

    // left vertical layout
    auto lsplitter = new QSplitter;
    lsplitter->setOrientation(Qt::Orientation::Vertical);
    _xbot2 = new XBot2Widget(this);
    lsplitter->addWidget(_xbot2);
    lsplitter->setStretchFactor(0, 0);
    lsplitter->addWidget(barplot_wid);
    lsplitter->setStretchFactor(1, 1);
    main_splitter->addWidget(lsplitter);

    // right vertical layout
    auto rsplitter = new QSplitter;
    rsplitter->setOrientation(Qt::Orientation::Vertical);

    // upper right horizontal layout
    auto trlayout = new QHBoxLayout(this);
    trlayout->addWidget(jstate_wid); // add joint state widget
    trlayout->addWidget(_sliders); // add sliders
    auto trwid = new QWidget;
    trwid->setLayout(trlayout);
    _tr_tab = new TopRightTab;
    _tr_tab->chart = _chart;
    _tr_tab->addTab(trwid, "Joint");
    rsplitter->addWidget(_tr_tab);
    rsplitter->setStretchFactor(0, 0);

    // add chart
    rsplitter->addWidget(_chart);
    rsplitter->setStretchFactor(1, 1);
    main_splitter->addWidget(rsplitter);

    // set layout
    layout->addWidget(main_splitter);
    setLayout(layout);

    // connect 'plot' buttons to chart
    connect(jstate_wid, &JointStateWidget::plotAdded,
            [this](QString plot)
    {
        _chart->addSeries(jstate_wid->getJointName() + "/" + plot);
    });

    // main timer
    _timer = new QTimer(this);
    _timer->setInterval(40); // 40 ms = 25 fps
    connect(_timer, &QTimer::timeout,
            this, &JointMonitorWidget::on_timer_event);
    _timer->start();

    _widget_started = true;

    /* Load config */
    _tr_tab->loadConfig(_ctx->config()[_tr_tab->name().toStdString()]);
    _chart->loadConfig(_ctx->config()[_chart->name().toStdString()]);


}

JointMonitorWidget::~JointMonitorWidget()
{

}

void JointMonitorWidget::on_timer_event()
{
    // receive callbacks
    ros::spinOnce();

    // update joint state widget
    jstate_wid->updateStatus();

    // update all barplot widgets
    for(auto& p : barplot_wid->wid_map)
    {
        p.second->updateStatus();
    }

    // update xbot2 wid
    _xbot2->update();

}

void JointMonitorWidget::on_jstate_recv(xbot_msgs::JointStateConstPtr msg)
{
    // if first message received, just set the _jnames vector
    if(!_valid_msg_recv)
    {
        _jnames = msg->name;
        _valid_msg_recv = true;
        return;
    }

    // widget not started yet, do nothing
    if(!_widget_started)
    {
        return;
    }

    static auto t0 = msg->header.stamp;
    auto now = msg->header.stamp;

    // parse message
    for(int i = 0; i < msg->name.size(); i++)
    {

        // some variables for convenience
        double k = msg->stiffness[i];
        double d = msg->damping[i];
        double qerr = msg->position_reference[i] - msg->link_position[i];
        double dqerr = msg->velocity_reference[i] - msg->link_velocity[i];
        double tauref_imp = k*qerr + d*dqerr + msg->effort_reference[i];

        // add a point to all chars lines
        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/link_pos",
                         now.toSec(),
                         msg->link_position[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/motor_pos",
                         now.toSec(),
                         msg->motor_position[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/pos_ref",
                         now.toSec(),
                         msg->position_reference[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/link_vel",
                         now.toSec(),
                         msg->link_velocity[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/motor_vel",
                         now.toSec(),
                         msg->motor_velocity[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/vel_ref",
                         now.toSec(),
                         msg->velocity_reference[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/torque",
                         now.toSec(),
                         msg->effort[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/torque_ffwd",
                         now.toSec(),
                         msg->effort_reference[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/torque_imp",
                         now.toSec(),
                         tauref_imp);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/current",
                         now.toSec(),
                         msg->aux[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/driver_temp",
                         now.toSec(),
                         msg->temperature_driver[i]);

        _chart->addPoint(QString::fromStdString(msg->name[i]) + "/motor_temp",
                         now.toSec(),
                         msg->temperature_motor[i]);



        // update the currently active joint state widget
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

        // update field of barplot, if active
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
    // widget not started yet, do nothing
    if(!_widget_started)
    {
        return;
    }

    for(size_t i = 0; i < msg->name.size(); i++)
    {
        // set red color to barplot green square
        auto wid = barplot_wid->wid_map.at(msg->name[i]);
        wid->setDanger();
        wid->setStatus(QString::fromStdString(msg->fault[i]));

        // if joint state widget is showing this joint, set fault
        // string
        if(msg->name[i] == jstate_wid->getJointName().toStdString())
        {
            jstate_wid->setStatus(msg->fault[i]);
        }
    }
}


void JointMonitorWidget::closeEvent(QCloseEvent* event)
{
    // main configuration node
    auto main_cfg = _ctx->config();

    {
        // sub-node for the top-right tab manager
        auto cfg = main_cfg[_tr_tab->name().toStdString()];

        // save tr tab configs
        _tr_tab->saveConfig(cfg);

        // put result back to main config
        main_cfg[_tr_tab->name().toStdString()] = cfg;
    }

    {
        // sub-node for the chart tab manager
        auto cfg = main_cfg[_chart->name().toStdString()];

        // save chart configs
        _chart->saveConfig(cfg);

        // put result back to main config
        main_cfg[_chart->name().toStdString()] = cfg;
    }

    // dump to file
    _ctx->saveConfig(main_cfg);
}
