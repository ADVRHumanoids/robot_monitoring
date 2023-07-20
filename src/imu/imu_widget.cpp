#include "imu_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/Dense>
#include <tf2/buffer_core.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace XBot::Ui;

void imu_widget_qrc_init()
{
    Q_INIT_RESOURCE(imu_ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    imu_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/imu.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}



ImuWidget::ImuWidget()
{

}

bool ImuWidget::init(CustomQtWidget::Args& args)
{
    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    QList<QString> imu_topics;
    for(const auto& info : master_topics)
    {
        if(info.datatype == "sensor_msgs/Imu")
        {
            imu_topics.push_back(QString::fromStdString(info.name));
        }
    }

    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    for(auto i : {"X", "Y", "Z"})
    {
        _omega.push_back(findChild<QProgressBar*>("omega" + QString(i)));
        _linacc.push_back(findChild<QProgressBar*>("linacc" + QString(i)));
    }

    for(auto i : {"R", "P", "Y"})
    {
        _rot.push_back(findChild<QProgressBar*>("rot" + QString(i)));
    }

    if(imu_topics.empty())
    {
        setEnabled(false);
    }

    _select_imu = findChild<QComboBox*>("imuSelector");

    for(auto topic : imu_topics)
    {
        auto cb = [this, topic](const sensor_msgs::ImuConstPtr& msg)
        {
            if(_select_imu->currentText() != topic)
            {
                return;
            }

            _link_name_map.at(topic) = QString::fromStdString(msg->header.frame_id);

            const double max_vel = 3.0;
            const double max_acc = 20;

            setProgressBarValue(_omega[0], msg->angular_velocity.x, max_vel);
            setProgressBarValue(_omega[1], msg->angular_velocity.y, max_vel);
            setProgressBarValue(_omega[2], msg->angular_velocity.z, max_vel);

            setProgressBarValue(_linacc[0], msg->linear_acceleration.x, max_acc);
            setProgressBarValue(_linacc[1], msg->linear_acceleration.y, max_acc);
            setProgressBarValue(_linacc[2], msg->linear_acceleration.z, max_acc);

            tf2::Quaternion q(
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
                msg->orientation.w);

            tf2::Matrix3x3 m(q);

            double r, p, y;
            m.getRPY(r, p, y);

            setProgressBarValue(_rot[0], r*180/M_PI, 180);
            setProgressBarValue(_rot[1], p*180/M_PI, 180);
            setProgressBarValue(_rot[2], y*180/M_PI, 180);

            auto link_name = QString::fromStdString(msg->header.frame_id);

            emit pointAdded(link_name + "/omega/x",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->angular_velocity.x));

            emit pointAdded(link_name + "/omega/y",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->angular_velocity.y));

            emit pointAdded(link_name + "/omega/z",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->angular_velocity.z));

        };

        auto sub = _nh.subscribe<sensor_msgs::Imu>(topic.toStdString(),
                                                   1,
                                                   cb);
        _subs.push_back(sub);

        _select_imu->addItem(topic);

        _link_name_map[topic] = "";
    }

    auto plotAllBtn = findChild<QPushButton*>("plotAllBtn");
    connect(plotAllBtn, &QPushButton::released,
            [this]()
            {
                emit seriesAdded(_link_name_map.at(_select_imu->currentText()) +
                                 "/omega/x");

                emit seriesAdded(_link_name_map.at(_select_imu->currentText()) +
                                 "/omega/y");

                emit seriesAdded(_link_name_map.at(_select_imu->currentText()) +
                                 "/omega/z");
            });

    return true;
}

void ImuWidget::update()
{
}

ImuWidget::~ImuWidget()
{

}

void ImuWidget::setProgressBarValue(QProgressBar* pb, double value, double max)
{
    pb->setValue(100*fabs(value)/max);
    pb->setFormat(QString("%1").arg(value, 5, 'f', 1));
}

QString ImuWidget::name()
{
    return "Imu";
}
