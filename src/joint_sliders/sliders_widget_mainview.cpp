#include <robot_monitoring_tools/joint_sliders/sliders_widget_mainview.h>
#include <sensor_msgs/JointState.h>

#ifdef XBOT_MSGS_SUPPORT
    #include <xbot_msgs/JointState.h>
    #include <xbot_msgs/JointCommand.h>
#endif

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    QUiLoader loader;

    QFile file(":/ui/impedance_widget_mainview.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;

}

std::vector<double> eigen_to_std(Eigen::VectorXd v)
{
    return std::vector<double>(v.data(), v.data() + v.size());
}


}

cartesio_gui::SlidersWidgetMainView::Options::Options():
    message_type("sensor_msgs"),
    enable_velocity_tab(true),
    enable_effort_tab(true),
    joint_state_topic("joint_states"),
    command_topic("command")
{
}



cartesio_gui::SlidersWidgetMainView::SlidersWidgetMainView (Options opt,
                                                            QWidget* parent):
    QWidget(parent),
    _load_success(false),
    _opt(opt),
    _nh(opt.ns)
{

    /* Create publisher */
    make_publisher();

    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);
    setLayout(layout);

    /* Status bar */
    _status = new QStatusBar;
    layout->addWidget(_status);

    
    /* Fill chain selector and widget stack */
    _chain_select = findChild<QComboBox *>("chainSelector");
    _wid_stack = findChild<QStackedWidget *>("stackedWidget");



    /* Try to contruct all slider widgets */
    try_construct();
    
    /* Connect chain selector to stacked layout */
    void (QComboBox::* activated_int)(int) = &QComboBox::currentIndexChanged;
    connect(_chain_select, activated_int,
            _wid_stack, &QStackedWidget::setCurrentIndex
           );

    /* Connect reset button */
    connect(findChild<QPushButton *>("reloadButton"), &QPushButton::released,
            this, &SlidersWidgetMainView::on_reload);

    /* Connect reset button */
    connect(findChild<QPushButton *>("disableButton"), &QPushButton::released,
            this, &SlidersWidgetMainView::on_disable_enable);

}

void cartesio_gui::SlidersWidgetMainView::makeJointVisible(QString jointname)
{
    for(auto& cpair : _robot->getChainMap())
    {
        if(cpair.second->hasJoint(jointname.toStdString()))
        {
            int id = _chain_select->findText(QString::fromStdString(cpair.first));

            if(id != -1)
            {
                _chain_select->setCurrentIndex(id);
            }
        }
    }
}

void cartesio_gui::SlidersWidgetMainView::contextMenuEvent(QContextMenuEvent * event)
{
    /* Right-click context menu */
    
    QMenu menu(this);

    auto * ns_action = new QAction("Set ROS namespace", this);
    auto * js_topic_action = new QAction("Set joint state topic", this);
    auto * cmd_topic_action = new QAction("Set command topic", this);
    auto * msg_type_action = new QAction("Set message type", this);

    connect(ns_action, &QAction::triggered,
            this, &SlidersWidgetMainView::set_ros_namespace);

    connect(js_topic_action, &QAction::triggered,
            this, &SlidersWidgetMainView::set_js_topic_name);

    connect(cmd_topic_action, &QAction::triggered,
            this, &SlidersWidgetMainView::set_cmd_topic_name);

    connect(msg_type_action, &QAction::triggered,
            this, &SlidersWidgetMainView::set_msg_type);

    menu.addAction(msg_type_action);
    menu.addAction(ns_action);
    menu.addAction(js_topic_action);
    menu.addAction(cmd_topic_action);
    menu.exec(event->globalPos());
}


void cartesio_gui::SlidersWidgetMainView::on_reload()
{
    // sliders not loaded correctly, retry
    if(!_load_success)
    {
        try_construct();
    }

    // failed again, return
    if(!_load_success)
    {
        return;
    }

    // try to sense robot state, if failed, disable GUI
    if(!sense())
    {
        _wid_stack->setEnabled(false);
    }

    // set sliders to sensed values
    for(auto ch : _robot->getChainNames())
    {
        Eigen::VectorXd q, k, d, zero;
        _robot->getChainMap().at(ch)->getMotorPosition(q);
        _robot->getChainMap().at(ch)->getStiffness(k);
        _robot->getChainMap().at(ch)->getDamping(d);
        zero.setZero(q.size());

        _wid_p_map.at(ch)->setInitialValue(::eigen_to_std(q));
        _wid_k_map.at(ch)->setInitialValue(::eigen_to_std(k));
        _wid_d_map.at(ch)->setInitialValue(::eigen_to_std(d));
    }
}

void cartesio_gui::SlidersWidgetMainView::on_disable_enable()
{
    // sliders not loaded correctly, do nothing (user should press reset)
    if(!_load_success)
    {
        print_status_msg("Press reset to reload the GUI");
        return;
    }

    // if button says "Enable", enable the gui...
    
    auto * button = findChild<QPushButton *>("disableButton");

    if(button->text() == "Disable")
    {
        _wid_stack->setEnabled(false);
        button->setText("Enable");
    }
    else
    {
        button->setText("Disable");
        _wid_stack->setEnabled(true);
        on_reload();
    }
}

void cartesio_gui::SlidersWidgetMainView::pos_callback(std::string jname, double value)
{
    if(_opt.message_type == "sensor_msgs")
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.push_back(jname);
        msg.position.push_back(value);
        _pub.publish(msg);
    }
    else
    {
#ifdef XBOT_MSGS_SUPPORT
        xbot_msgs::JointCommand msg;
        msg.header.stamp = ros::Time::now();
        msg.header.seq = 0;
        msg.name.push_back(jname);
        msg.position.push_back(value);
        msg.ctrl_mode.push_back(1);
        _pub.publish(msg);
#endif
    }
}

void cartesio_gui::SlidersWidgetMainView::vel_callback(std::string jname, double value)
{
    if(_opt.message_type == "sensor_msgs")
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.push_back(jname);
        msg.velocity.push_back(value);
        _pub.publish(msg);
    }
    else
    {
#ifdef XBOT_MSGS_SUPPORT
        xbot_msgs::JointCommand msg;
        msg.header.stamp = ros::Time::now();
        msg.header.seq = 0;
        msg.name.push_back(jname);
        msg.velocity.push_back(value);
        msg.ctrl_mode.push_back(2);
        _pub.publish(msg);
#endif
    }
    
}


void cartesio_gui::SlidersWidgetMainView::tau_callback(std::string jname, double value)
{
    if(_opt.message_type == "sensor_msgs")
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.push_back(jname);
        msg.effort.push_back(value);
        _pub.publish(msg);
    }
    else
    {
#ifdef XBOT_MSGS_SUPPORT
        xbot_msgs::JointCommand msg;
        msg.header.stamp = ros::Time::now();
        msg.header.seq = 0;
        msg.name.push_back(jname);
        msg.effort.push_back(value);
        msg.ctrl_mode.push_back(4);
        _pub.publish(msg);
#endif
    }
    
}


void cartesio_gui::SlidersWidgetMainView::k_callback(std::string jname, double value)
{
    
#ifdef XBOT_MSGS_SUPPORT
    xbot_msgs::JointCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = 0;
    msg.name.push_back(jname);
    msg.stiffness.push_back(value);
    msg.ctrl_mode.push_back(8);
    _pub.publish(msg);
#endif
    
}


void cartesio_gui::SlidersWidgetMainView::d_callback(std::string jname, double value)
{
    
#ifdef XBOT_MSGS_SUPPORT
    xbot_msgs::JointCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = 0;
    msg.name.push_back(jname);
    msg.damping.push_back(value);
    msg.ctrl_mode.push_back(16);
    _pub.publish(msg);
#endif
    
}

void cartesio_gui::SlidersWidgetMainView::print_status_msg(QString msg)
{
    _status->showMessage(msg);
}


cartesio_gui::SlidersWidgetMainView::~SlidersWidgetMainView()
{

}

void connected(const ros::SingleSubscriberPublisher&) {}
void disconnected(const ros::SingleSubscriberPublisher&) {}

void cartesio_gui::SlidersWidgetMainView::make_publisher()
{
    if(_opt.message_type == "sensor_msgs")
    {
        _pub = _nh.advertise<sensor_msgs::JointState>(_opt.command_topic, 10);
    }
    else if(_opt.message_type == "xbot_msgs")
    {
#ifdef XBOT_MSGS_SUPPORT
        auto op = 
            ros::AdvertiseOptions::create<xbot_msgs::JointCommand>(_opt.command_topic, 
                                                                   10, 
                                                                   connected,
                                                                   disconnected, 
                                                                   ros::VoidPtr(), 
                                                                   NULL);
        op.has_header = false; 
        _pub = _nh.advertise(op);
#else
        throw std::runtime_error("Widget was compiled without -DXBOT_MSGS_SUPPORT");
#endif
    }
    else
    {
        throw std::runtime_error("Invalid parameter 'message_type'");
    }
}

void cartesio_gui::SlidersWidgetMainView::try_construct()
{
    try
    {
        construct();
    }
    catch(std::exception& e)
    {
        print_status_msg("Unable to load widget: " + QString::fromUtf8(e.what()));
    }
}

void cartesio_gui::SlidersWidgetMainView::construct()
{
    // load an xbotinterface from param server 
    auto opt = XBot::ConfigOptionsFromParamServer(_nh);
    _robot = std::make_shared<XBot::XBotInterface>();
    _robot->init(opt);

    // remove all widgets
    for(int i = _wid_stack->count() - 1; i >= 0; i--)
    {
        auto * widget = _wid_stack->widget(i);
        _wid_stack->removeWidget(widget);
        widget->deleteLater();
    }

    if(!sense()) // updates robot from ros topic
    {
        throw std::runtime_error("Unable to get joint states");
    } 

    // construct all widgets 
    for(auto ch : _robot->getChainNames())
    {
        Eigen::VectorXd zero, ones, q, k, d, qmin, qmax, qdotmax, taumax;
        _robot->getChainMap().at(ch)->getJointLimits(qmin, qmax);
        _robot->getChainMap().at(ch)->getVelocityLimits(qdotmax);
        _robot->getChainMap().at(ch)->getEffortLimits(taumax);
        _robot->getChainMap().at(ch)->getMotorPosition(q);
        _robot->getChainMap().at(ch)->getStiffness(k);
        _robot->getChainMap().at(ch)->getDamping(d);
        zero.setZero(q.size());
        ones.setOnes(q.size());
        auto j_list = _robot->getChainMap().at(ch)->getJointNames();

        auto * tab_wid = new QTabWidget;

        namespace pl = std::placeholders;

        auto * p_wid = new cartesio_gui::SlidersWidget(ch, j_list);
        p_wid->setBindEnabled(false);
        p_wid->setRange(::eigen_to_std(qmin), ::eigen_to_std(qmax));
        p_wid->setInitialValue(::eigen_to_std(q));
        p_wid->setCallback(std::bind(&SlidersWidgetMainView::pos_callback,
                                     this,
                                     pl::_1, pl::_2));
        _wid_p_map[ch] = p_wid;
        tab_wid->addTab(p_wid, "Position");

        auto * v_wid = new cartesio_gui::SlidersWidget(ch, j_list);
        v_wid->setBindEnabled(false);
        v_wid->setRange(::eigen_to_std(-qdotmax), ::eigen_to_std(qdotmax));
        v_wid->setCallback(std::bind(&SlidersWidgetMainView::vel_callback,
                                     this,
                                     pl::_1, pl::_2));

        auto * tau_wid = new cartesio_gui::SlidersWidget(ch, j_list);
        tau_wid->setBindEnabled(false);
        tau_wid->setRange(::eigen_to_std(-taumax), ::eigen_to_std(taumax));
        tau_wid->setCallback(std::bind(&SlidersWidgetMainView::tau_callback,
                                       this,
                                       pl::_1, pl::_2));

        auto * k_wid = new cartesio_gui::SlidersWidget(ch, j_list);
        k_wid->setInitialValue(::eigen_to_std(k));
        k_wid->setRange(::eigen_to_std(zero), ::eigen_to_std(ones * 2000));
        k_wid->setCallback(std::bind(&SlidersWidgetMainView::k_callback,
                                     this,
                                     pl::_1, pl::_2));
        _wid_k_map[ch] = k_wid;


        auto * d_wid = new cartesio_gui::SlidersWidget(ch, j_list);
        d_wid->setInitialValue(::eigen_to_std(d));
        d_wid->setRange(::eigen_to_std(zero), ::eigen_to_std(ones * 100));
        d_wid->setCallback(std::bind(&SlidersWidgetMainView::d_callback,
                                     this,
                                     pl::_1, pl::_2));
        _wid_d_map[ch] = d_wid;


        _wid_stack->addWidget(tab_wid);
        _chain_select->addItem(QString::fromStdString(ch));

        if(_opt.enable_velocity_tab)
        {
            tab_wid->addTab(v_wid, "Velocity");
        }

        if(_opt.enable_effort_tab)
        {
            tab_wid->addTab(tau_wid, "Effort");
        }

        if(_opt.message_type == "xbot_msgs")
        {
            tab_wid->addTab(k_wid, "Stiffness");
            tab_wid->addTab(d_wid, "Damping");
        }
    }

    _load_success = true;
}

bool cartesio_gui::SlidersWidgetMainView::sense()
{
    print_status_msg("Waiting for joint states..");

    if(_opt.message_type == "sensor_msgs")
    {
        auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>(_opt.joint_state_topic,
                                                                       _nh,
                                                                       ros::Duration(1.0));

        if(!msg || msg->name.empty())
        {
            print_status_msg("Unable to receive joint states");
            return false;
        }

        XBot::JointNameMap qmap;

        for(int i = 0; i < msg->name.size(); i++)
        {
            qmap[msg->name[i]] = msg->position.at(i);
        }

        _robot->setMotorPosition(qmap);

    }

#ifdef XBOT_MSGS_SUPPORT

    if(_opt.message_type == "xbot_msgs")
    {
        auto msg = ros::topic::waitForMessage<xbot_msgs::JointState>(_opt.joint_state_topic,
                                                                     _nh,
                                                                     ros::Duration(1.0));

        if(!msg || msg->name.empty())
        {
            print_status_msg("Unable to receive joint states");
            return false;
        }

        XBot::JointNameMap qmap, kmap, dmap;

        for(int i = 0; i < msg->name.size(); i++)
        {
            qmap[msg->name[i]] = msg->position_reference.at(i);
            kmap[msg->name[i]] = msg->stiffness.at(i);
            dmap[msg->name[i]] = msg->damping.at(i);
        }

        _robot->setMotorPosition(qmap);
        _robot->setStiffness(kmap);
        _robot->setDamping(dmap);

    }

#endif

    print_status_msg("Ready");
    return true;

}

void cartesio_gui::SlidersWidgetMainView::set_msg_type()
{
    QStringList items;
    items.push_back("sensor_msgs");

#ifdef XBOT_MSGS_SUPPORT
    items.push_back("xbot_msgs");
#endif

    int current_item = _opt.message_type == "sensor_msgs" ? 0 : 1;

    bool ok;
    QString item = QInputDialog::getItem(this, "Message type selection",
                                         "Message type:", items, current_item, false, &ok);
    if (ok && !item.isEmpty())
    {
        _opt.message_type = item.toStdString();
        make_publisher();
    }
}

void cartesio_gui::SlidersWidgetMainView::set_js_topic_name()
{
    bool ok;
    QString text = QInputDialog::getText(this,
                                         "Joint state topic name selection",
                                         "Joint state topic:",
                                         QLineEdit::Normal,
                                         QString::fromStdString(_opt.joint_state_topic),
                                         &ok);
    if (ok && !text.isEmpty())
    {
        _opt.joint_state_topic = text.toStdString();
    }
}

void cartesio_gui::SlidersWidgetMainView::set_cmd_topic_name()
{
    bool ok;
    QString text = QInputDialog::getText(this,
                                         "Command topic name selection",
                                         "Command topic:",
                                         QLineEdit::Normal,
                                         QString::fromStdString(_opt.command_topic),
                                         &ok);
    if (ok && !text.isEmpty())
    {
        _opt.command_topic = text.toStdString();
        make_publisher();
    }
}

void cartesio_gui::SlidersWidgetMainView::set_ros_namespace()
{
    bool ok;
    QString text = QInputDialog::getText(this,
                                         "ROS namespace selection",
                                         "ROS namespace:",
                                         QLineEdit::Normal,
                                         QString::fromStdString(_opt.ns),
                                         &ok);
    if (ok && !text.isEmpty())
    {
        _opt.ns = text.toStdString();
        _nh = ros::NodeHandle(_opt.ns);
        make_publisher();
    }
}


