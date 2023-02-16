#include "softhand_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <sensor_msgs/JointState.h>

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

}


void SofthandWidget::setProgressBarValue(QProgressBar* pb, double value, double max)
{
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

            QRegularExpression regex("^/(.+?)/");
            QString ns_name = regex.match(name).captured(1);

            std::cout << "adding " << name.toStdString() << " to: " << ns_name.toStdString() << std::endl;
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

    // softhand selection
    _select_softhand = findChild<QComboBox*>("softhandCombo");

    auto createProgressBar = [this, softhand_names](QLayout * layout, QString hand_name)
    {

//         spawn a progress bar for each finger
        const auto& finger_list = softhand_names[hand_name];

        for(auto topic : finger_list)
        {
            QString strip_name = topic;
            strip_name.replace("/" + hand_name, "");

            std::cout << "Finger detected: " << strip_name.toStdString() << std::endl;

            QGroupBox *fingerBox = new QGroupBox(strip_name);

            QGridLayout *grid = new QGridLayout;

            QLabel *pos_name = new QLabel("Position");
            QLabel *effort_name = new QLabel("Effort");

            QProgressBar *positionProgBar = new QProgressBar();
            QProgressBar *effortProgBar = new QProgressBar();

            grid->addWidget(pos_name, 0, 0);
            grid->addWidget(positionProgBar, 0, 1);

            grid->addWidget(effort_name, 1, 0);
            grid->addWidget(effortProgBar, 1, 1);

            fingerBox->setLayout(grid);
            layout->addWidget(fingerBox);

            // receive state
            auto cb = [this, hand_name, topic, positionProgBar, effortProgBar]
                    (const sensor_msgs::JointStateConstPtr& msg)
            {
                if(_select_softhand->currentText() != hand_name)
                {
                    return;
                }

                _link_name_map.at(hand_name) = QString::fromStdString(msg->header.frame_id);

                setProgressBarValue(positionProgBar, msg->position[0], 1.0);
                setProgressBarValue(effortProgBar, msg->effort[0], 50);

//                auto link_name = QString::fromStdString(msg->header.frame_id);

//                emit pointAdded(link_name + "/effort",
//                                QPointF(msg->header.stamp.toSec(),
//                                        msg->effort[0]));

            };

            std::cout <<topic.toStdString() << std::endl;
            auto sub = _nh.subscribe<sensor_msgs::JointState>(
                        topic.toStdString(),
                        1,
                        cb);

            _subs.push_back(sub);
            _link_name_map[hand_name] = "";

        }
    };


    // for each softhand create a tab
    const auto& m1 = softhand_names.toStdMap();
    for (auto softhand : m1)
    {
        _select_softhand->addItem(softhand.first);
    }



    // get main frame to add progress bars
    auto frame_states = findChild<QFrame*>("frameStates");
    auto frame_states_layout = new QVBoxLayout;
    frame_states->setLayout(frame_states_layout);
    // initialize layout
    createProgressBar(frame_states_layout, _select_softhand->currentText());

    connect(_select_softhand, &QComboBox::currentTextChanged,
            [this, frame_states, createProgressBar](QString hand_name)
    {

        auto current_layout = frame_states->layout();
        clearLayout(current_layout);

        createProgressBar(current_layout, hand_name);
    });


//    // stupid hack to trigger text change
//    _select_softhand->setCurrentIndex(1);

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
        ros::Publisher pub;

        auto command_topic_name = _select_softhand->currentText() + "/synergy_command";
        auto cmd_topic = _pub_map.find(command_topic_name);

        if(cmd_topic == _pub_map.end())
        {
            std::cout << "advertised " << command_topic_name.toStdString() << "\n";
            pub = _nh.advertise<sensor_msgs::JointState>(command_topic_name.toStdString(),
                                                         1);
//            _pub_map[state_topic] = pub;
        }
        else
        {
            pub = cmd_topic->second;
        }

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.position = {cmdSpinBox->value()};
        pub.publish(msg);

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
