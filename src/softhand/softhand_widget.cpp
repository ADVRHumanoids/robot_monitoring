#include "softhand_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Dense>
#include <tf2/buffer_core.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace XBot::Ui;

void softhand_widget_qrc_init()
{
    Q_INIT_RESOURCE(softhand_ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    softhand_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/softhand.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

QWidget * LoadUiFileHand(QWidget * parent)
{
    softhand_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/hand_state.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}


}


void SofthandWidget::setProgressBarValue(QProgressBar* pb, double value, double max)
{
    if (!pb->isEnabled())
    {
        pb->setEnabled(true);
    }

    pb->setValue(100*fabs(value)/max);
    pb->setFormat(QString("%1").arg(value, 5, 'f', 1));
}

void clearLayout(QLayout *layout) {
    if (layout == NULL)
        return;
    QLayoutItem *item;
    while((item = layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
           delete item->widget();
        }
        delete item;
    }
}

bool SofthandWidget::init(CustomQtWidget::Args& args)
{
    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    // discovery
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    QMap<QString, QList<QString>> softhand_names;

    for(const auto& info : master_topics)
    {
        if(info.datatype == "sensor_msgs/JointState")
        {
            auto name = QString::fromStdString(info.name);

            if(!name.contains("hand"))
            {
                continue;
            }

            if (name.contains("/motor_state"))
            {
                continue;
            }

            QRegularExpression regex("^/(.+?)/");
            QString ns_name = regex.match(name).captured(1);

            softhand_names[ns_name].append(name);


        }
    }

    // set widget
    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    if(softhand_names.empty())
    {
        setEnabled(false);
    }

    std::vector<std::string> fingers = {"index", "little", "middle", "ring", "thumb"};
    std::vector<std::string> finger_joints = {"knuckle", "proximal", "middle", "distal"};
    std::vector<std::string> thumb_joints = {"knuckle", "proximal", "middle"};

    std::map<std::string, std::vector<std::string>> hand_map;

    for (std::string finger : fingers)
    {
        if (finger != "thumb")
        {
            hand_map[finger] = finger_joints;
        }
        else
        {
            hand_map[finger] = thumb_joints;
        }

    }

    // softhand selection
    _select_softhand = findChild<QComboBox*>("softhandCombo");

//    auto createProgressBar = [this, softhand_names](QLayout * layout, QString hand_name)
//    {

//        // spawn a progress bar for each finger
//        const auto& finger_list = softhand_names[hand_name];

//        for(auto topic : finger_list)
//        {

//            QString strip_name = topic;
//            strip_name.replace("/" + hand_name, "");

//            // I should expect the phalanges to be fixed

//            QGroupBox *fingerBox = new QGroupBox(strip_name);

//            QGridLayout *grid = new QGridLayout;

//            QLabel *pos_name = new QLabel("Position");
//            QLabel *effort_name = new QLabel("Effort");

//            QProgressBar *positionProgBar = new QProgressBar();
//            QProgressBar *effortProgBar = new QProgressBar();

//            grid->addWidget(pos_name, 0, 0);
//            grid->addWidget(positionProgBar, 0, 1);

//            grid->addWidget(effort_name, 1, 0);
//            grid->addWidget(effortProgBar, 1, 1);

//            fingerBox->setLayout(grid);
//            layout->addWidget(fingerBox);

//            // receive state
//            auto cb = [this, hand_name, topic, positionProgBar, effortProgBar]
//                    (const sensor_msgs::JointStateConstPtr& msg)
//            {
//                if(_select_softhand->currentText() != hand_name)
//                {
//                    return;
//                }

//                _link_name_map.at(hand_name) = QString::fromStdString(msg->header.frame_id);

//                setProgressBarValue(positionProgBar, msg->position[0], 1.0);
//                setProgressBarValue(effortProgBar, msg->effort[0], 10);



//                for (std::string name : msg->name)
//                {

//                }
//                auto link_name = QString::fromStdString(msg->header.frame_id);

//                emit pointAdded(link_name + "/effort",
//                                QPointF(msg->header.stamp.toSec(),
//                                        msg->effort[0]));

//            };

//            auto sub = _nh.subscribe<sensor_msgs::JointState>(
//                        topic.toStdString(),
//                        1,
//                        cb);

//            _subs.push_back(sub);
//            _link_name_map[hand_name] = "";
//        }
//    };


    auto frame_states = findChild<QFrame*>("frameStates");
    auto frame_states_layout = new QGridLayout;
    frame_states->setLayout(frame_states_layout);

    std::map<std::string, std::vector<QWidget*>> finger_widgets;

    int row_i = 0;
    for (auto finger : hand_map)
    {
        auto finger_name = finger.first + "_state";
        QLabel *finger_label = new QLabel(QString::fromStdString(finger_name));
        finger_label->setStyleSheet("font-weight: bold; ");


        int col_i = 0;
        for (auto elem : finger.second)
        {
            auto wid_joints = LoadUiFileHand(this);
            finger_widgets[finger_name].push_back(wid_joints);

            auto frame_states = wid_joints->findChild<QLabel*>("jointName");
            frame_states->setText(QString::fromStdString(elem));

            frame_states_layout->addWidget(wid_joints, row_i+1, col_i);
            col_i++;
        }

        frame_states_layout->addWidget(finger_label, row_i, 0, 1, col_i+1, Qt::AlignCenter);

        QFrame *line_1 = new QFrame();
        line_1->setObjectName(QString::fromUtf8("line"));
        line_1->setGeometry(QRect(320, 150, 118, 3));
        line_1->setFrameShape(QFrame::HLine);
        line_1->setFrameShadow(QFrame::Sunken);

        frame_states_layout->addWidget(line_1, row_i+2, 0, 1, col_i+1);


        row_i = row_i+3;
    }

    const auto& m1 = softhand_names.toStdMap();
    for(auto softhand_name : m1)
    {
        for (auto finger_topic : softhand_name.second)
        {
            auto cb = [this, finger_topic, softhand_name, finger_widgets]
                    (const sensor_msgs::JointStateConstPtr& msg)
            {
                if(_select_softhand->currentText() != softhand_name.first)
                {
                    return;
                }

                QString strip_name = finger_topic;

                strip_name.replace("/" + softhand_name.first + "/", "");
                std::string name_finger = strip_name.toStdString();

                _link_name_map.at(finger_topic) = QString::fromStdString(msg->header.frame_id);

                if (finger_widgets.find(name_finger) == finger_widgets.end())
                {
                    std::cout << name_finger << " not considered in GUI." << std::endl;
                }
                else
                {
                    for (int wid_id = 0; wid_id < finger_widgets.at(name_finger).size(); wid_id++)
                    {
                        auto pb = finger_widgets.at(name_finger)[wid_id]->findChild<QProgressBar*>("jointProgressBar");
                        setProgressBarValue(pb, msg->position[wid_id], 1.0);
                    }
                }

            };

            auto sub = _nh.subscribe<sensor_msgs::JointState>(
                        finger_topic.toStdString(),
                        1,
                        cb);

            _subs.push_back(sub);
            _link_name_map[finger_topic] = "";

        }

        _select_softhand->addItem(softhand_name.first);

        auto command_topic_name = "/" + softhand_name.first + "/synergy_command";
        auto pub = _nh.advertise<std_msgs::Float64>(command_topic_name.toStdString(),
                                                     1);

        _pub_map[command_topic_name] = pub;

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

    auto openSofthandBtn = findChild<QPushButton*>("openSofthandBtn");

    connect(openSofthandBtn, &QPushButton::released,
            [cmdSendBtn, cmdSpinBox]()
    {
        cmdSpinBox->setValue(0);
        cmdSendBtn->released();
    });

    connect(cmdSendBtn, &QPushButton::released,
            [this, cmdSpinBox]()
    {
        auto command_topic_name = "/" + _select_softhand->currentText() + "/synergy_command";
        auto cmd_topic = _pub_map.find(command_topic_name);

        if(cmd_topic == _pub_map.end())
        {
            std::cout << "Topic: '" + command_topic_name.toStdString() + "' not found." << std::endl;
            return;
        }

        std_msgs::Float64 msg;
        msg.data = {cmdSpinBox->value()};
        cmd_topic->second.publish(msg);

    });

    return true;
}

QString SofthandWidget::name()
{
    return "Softhand";
}

void SofthandWidget::update()
{
    CustomQtWidget::update();
}

bool SofthandWidget::loadConfig(const YAML::Node &cfg)
{
    return true;
}

bool SofthandWidget::saveConfig(YAML::Node &cfg)
{
    return true;
}

SofthandWidget::~SofthandWidget()
{

}
