#include "joint_monitor_widget.h"
#include "bar_plot_widget.h"
#include <QHBoxLayout>

#include <urdf_parser/urdf_parser.h>

struct BIT_FAULT {

    // bits 0..3
    uint16_t  m3_rxpdo_pos_ref:1;   // ?
    uint16_t  m3_rxpdo_vel_ref:1;   // ?
    uint16_t  m3_rxpdo_tor_ref:1;   // ?
    uint16_t  m3_fault_hardware:1;  // ! +
    // bits 4..7
    uint16_t  m3_params_out_of_range:1;         // !
    uint16_t  m3_torque_array_not_loaded:1;     // !
    uint16_t  m3_torque_read_error:1;           // ? ++
    uint16_t  m3_spare:1;
    // bits 8, 9
    /* 20 BAD consecutive sensor readings give error
     * each GOOD or almost good ( green/orange) reading decrement error counter
     * each other conditions on reading increment error counter
     */
    uint16_t  m3_link_enc_error:1;  // ? +
    /* 20 consecutive sensor readings give error
     * see m3_link_enc_error
     */
    uint16_t  m3_defl_enc_error:1;  // ? ++
    /*
    The warning bit is set when the temperature is over 60 and released when decrease under 55 degrees.
    The warning bit is only checked before starting the controller, if is set the controller is not allowed to start.
    If set during run no action is provided, is only a signal that the system is getting warm.
    The error bit is set when the temperature is over 70 and released when decrease under 65 degrees.
    The error bit is always checked, if a temperature error is triggered the system doesn’t start the controller
    and if is running it stop the controller.
    For the motor temperature is the same behavior (the warning and error temperature bit are shared)
    but the temperature limits are different (80 for warning and 90 for error with 5 degrees histeresys)
    */
    // bits 10, 11
    uint16_t  m3_temperature_warning:1; // ?
    uint16_t  m3_temperature_error:1;   // ? +
    // bits 12..15
    /* 200 consecutive sensor readings give error
     * see m3_link_enc_error
     */
    uint16_t  c28_motor_enc_error:1;    // ? +
    uint16_t  c28_v_batt_read_fault:1;  // ?
    uint16_t  c28_enter_sand_box:1; // ?
    uint16_t  c28_spare_1:1;


    std::ostream& dump ( std::ostream& os, const std::string delim ) const {

        if(m3_rxpdo_pos_ref)    { os << "m3_rxpdo_pos_ref" << delim; }
        if(m3_rxpdo_vel_ref)    { os << "m3_rxpdo_vel_ref" << delim; }
        if(m3_rxpdo_tor_ref)    { os << "m3_rxpdo_tor_ref" << delim; }
        if(m3_fault_hardware)   { os << "m3_fault_hardware" << delim; }

        if(m3_params_out_of_range)      { os << "m3_params_out_of_range" << delim; }
        if(m3_torque_array_not_loaded)  { os << "m3_torque_array_not_loaded" << delim; }
        if(m3_torque_read_error) { os << "m3_torque_read_out_of_range" << delim; }

        if(m3_link_enc_error)       { os << "m3_link_enc_error" << delim; }
        if(m3_defl_enc_error)       { os << "m3_defl_enc_error" << delim; }
        if(m3_temperature_warning)  { os << "m3_temperature_warning" << delim; }
        if(m3_temperature_error)    { os << "m3_temperature_error" << delim; }

        if(c28_motor_enc_error)     { os << "c28_motor_enc_error" << delim; }
        if(c28_v_batt_read_fault)   { os << "c28_v_batt_read_fault" << delim; }
        if(c28_enter_sand_box)      { os << "c28_enter_sand_box" << delim; }

        return os;
    }

    void fprint ( FILE *fp ) {
        std::ostringstream oss;
        dump(oss,"\t");
        fprintf ( fp, "%s", oss.str().c_str() );
    }
    int sprint ( char *buff, size_t size ) {
        std::ostringstream oss;
        dump(oss,"\t");
        return snprintf ( buff, size, "%s", oss.str().c_str() );
    }

};

union centAC_fault_t{
    uint16_t all;
    BIT_FAULT bit;
};

JointMonitorWidget::JointMonitorWidget(QWidget *parent) :
    QWidget(parent),
    _valid_msg_recv(false)
{

    ros::NodeHandle nh("xbotcore");
    _jstate_sub = nh.subscribe("joint_states", 10, &JointMonitorWidget::on_jstate_recv, this);

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

    barplot_wid->setOnJointClicked(on_joint_clicked);

    _timer = new QTimer(this);
    _timer->setInterval(40);
    connect(_timer, &QTimer::timeout,
            this, &JointMonitorWidget::on_timer_event);



    auto layout = new QHBoxLayout(this);
    layout->addWidget(barplot_wid);
    layout->addWidget(jstate_wid);
    layout->addWidget(_sliders);
    setLayout(layout);

    _timer->start();
}

void JointMonitorWidget::on_timer_event()
{
    ros::spinOnce();
}

void JointMonitorWidget::on_jstate_recv(xbot_msgs::JointStateConstPtr msg)
{
    if(!_valid_msg_recv)
    {
        _jnames = msg->name;
        _valid_msg_recv = true;
        return;
    }

    for(int i = 0; i < msg->name.size(); i++)
    {
        if(msg->name[i] == jstate_wid->getJointName().toStdString())
        {
            jstate_wid->tor->setValue(msg->effort[i]);
            jstate_wid->torref->setValue(msg->effort_reference[i]);
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

            std::string fault_str = "Ok";

            if(msg->fault[i] != 0)
            {
                auto wid = barplot_wid->wid_map.at(msg->name[i]);
                centAC_fault_t fault;
                fault.all = msg->fault[i];
                std::stringstream ss;
                fault.bit.dump(ss, "\n");
                fault_str = ss.str();
                fault_str.resize(fault_str.size()-1);
            }

            jstate_wid->setStatus(fault_str);
        }

        if(msg->fault[i] == 0)
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setSafe();
        }
        else
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setDanger();
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
        else if(barplot_wid->getFieldType() == "Motor position")
        {
            auto wid = barplot_wid->wid_map.at(msg->name[i]);
            wid->setValue(msg->motor_position[i]);
            double qmin = _urdf->getJoint(msg->name[i])->limits->lower;
            double qmax = _urdf->getJoint(msg->name[i])->limits->upper;
            wid->setRange(qmin, qmax);
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
