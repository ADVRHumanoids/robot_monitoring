#include "gripper_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <sensor_msgs/JointState.h>

#include <eigen3/Eigen/Dense>
#include <tf2/buffer_core.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace XBot::Ui;

void gripper_widget_qrc_init()
{
    Q_INIT_RESOURCE(gripper_ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    gripper_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/gripper.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}


void GripperWidget::setProgressBarValue(QProgressBar* pb, double value, double max)
{
    pb->setValue(100*fabs(value)/max);
    pb->setFormat(QString("%1").arg(value, 5, 'f', 1));
}

ros::Publisher GripperWidget::getCommandPublisher()
{
    ros::Publisher pub;

    auto it = _pub_map.find(_select_gripper->currentText());

    if(it == _pub_map.end())
    {
        auto state_topic = _select_gripper->currentText();
        auto cmd_topic = state_topic;
        cmd_topic.replace("state", "command");
        std::cout << "advertised " << cmd_topic.toStdString() << "\n";
        pub = _nh.advertise<sensor_msgs::JointState>(cmd_topic.toStdString(),
                                                     1);
        _pub_map[state_topic] = pub;
    }
    else
    {
        pub = it->second;
    }

    return pub;

}

bool GripperWidget::init(CustomQtWidget::Args& args)
{
    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    // discovery
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    QList<QString> gripper_topics;
    for(const auto& info : master_topics)
    {
        if(info.datatype == "sensor_msgs/JointState")
        {
            auto name = QString::fromStdString(info.name);

            if(!name.contains("gripper") ||
                    name.contains("command") ||
                    !name.contains("state"))
            {
                continue;
            }

            gripper_topics.push_back(name);
        }
    }

    // set widget
    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    if(gripper_topics.empty())
    {
        setEnabled(false);
    }

    // retrieve children
    auto positionProgBar = findChild<QProgressBar*>("positionProgBar");
    auto effortProgBar = findChild<QProgressBar*>("effortProgBar");

    // gripper selection
    _select_gripper = findChild<QComboBox*>("gripperCombo");

    for(auto topic : gripper_topics)
    {
        // receive state
        auto cb = [this, topic, positionProgBar, effortProgBar]
                (const sensor_msgs::JointStateConstPtr& msg)
        {
            if(_select_gripper->currentText() != topic)
            {
                return;
            }

            _link_name_map.at(topic) = QString::fromStdString(msg->header.frame_id);

            setProgressBarValue(positionProgBar, msg->position[0], 1.0);
            setProgressBarValue(effortProgBar, msg->effort[0], 50);

            auto link_name = QString::fromStdString(msg->header.frame_id);

            emit pointAdded(link_name + "/effort",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->effort[0]));

        };

        auto sub = _nh.subscribe<sensor_msgs::JointState>(
                    topic.toStdString(),
                    1,
                    cb);

        _subs.push_back(sub);

        _select_gripper->addItem(topic);

        _link_name_map[topic] = "";

    }

    // command
    auto cmdSlider = findChild<QSlider*>("cmdSlider");
    auto cmdSpinBox = findChild<QDoubleSpinBox*>("cmdSpinBox");
    auto cmdSendBtn = findChild<QPushButton*>("cmdSendBtn");

    connect(cmdSlider, &QSlider::valueChanged,
            [cmdSpinBox](int value)
    {
        cmdSpinBox->setValue(value / 100.0);
    });

    connect(cmdSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
            [cmdSlider](double value)
    {
       cmdSlider->setValue(value * 100.);
    });

    auto openGripperBtn = findChild<QPushButton*>("openGripperBtn");

    connect(openGripperBtn, &QPushButton::released,
            [cmdSendBtn, cmdSpinBox]()
    {
        cmdSpinBox->setValue(0);
        cmdSendBtn->released();
    });

    connect(cmdSendBtn, &QPushButton::released,
            [this, cmdSpinBox]()
    {
        ros::Publisher pub = getCommandPublisher();

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.position = {cmdSpinBox->value()};
        pub.publish(msg);

    });

    // torque publisher
    auto torqueSlider = findChild<QSlider*>("torqueSlider");
    auto torqueSpinBox = findChild<QDoubleSpinBox*>("torqueSpinBox");
    auto torqueSendBtn = findChild<QPushButton*>("torqueSendBtn");

    const double max_torque = 10.0;

    connect(torqueSlider, &QSlider::valueChanged,
            [torqueSpinBox, max_torque](int value)
    {
        torqueSpinBox->setValue(((value / 500.0) - 1.0) * max_torque);
    });

    connect(torqueSpinBox, qOverload<double>(&QDoubleSpinBox::valueChanged),
            [torqueSlider, max_torque](double value)
    {
         torqueSlider->setValue((value + max_torque) * 500. / max_torque);
    });

    connect(torqueSendBtn, &QPushButton::released,
            [this, torqueSpinBox, torqueSendBtn]()
    {
        ros::Publisher pub = getCommandPublisher();

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.effort = {torqueSpinBox->value()};

        pub.publish(msg);
    });

    return true;
}

QString GripperWidget::name()
{
    return "Gripper";
}

void GripperWidget::update()
{
    CustomQtWidget::update();
}

bool GripperWidget::loadConfig(const YAML::Node &cfg)
{
    return true;
}

bool GripperWidget::saveConfig(YAML::Node &cfg)
{
    return true;
}

GripperWidget::~GripperWidget()
{

}
