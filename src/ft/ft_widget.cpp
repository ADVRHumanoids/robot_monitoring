#include "ft_widget.h"

#include <QLabel>
#include <QVBoxLayout>
#include <QtUiTools>

#include <geometry_msgs/WrenchStamped.h>
#include <fmt/format.h>

#include <eigen3/Eigen/Dense>
#include <tf2/buffer_core.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace XBot::Ui;

void ft_widget_qrc_init()
{
    Q_INIT_RESOURCE(ft_ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{
    ft_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/ft.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
}

}

FtWidget::FtWidget(int i)
{

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    QList<QString> ft_topics;
    for(const auto& info : master_topics)
    {
        if(info.datatype == "geometry_msgs/WrenchStamped")
        {
            ft_topics.push_back(QString::fromStdString(info.name));
        }
    }

    auto wid = LoadUiFile(this);

    auto l = new QVBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    for(auto i : {"X", "Y", "Z"})
    {
        _force.push_back(findChild<QProgressBar*>("f" + QString(i)));
        _torque.push_back(findChild<QProgressBar*>("tor" + QString(i)));
    }

    if(ft_topics.empty())
    {
        setEnabled(false);
    }

    _select_ft = findChild<QComboBox*>("ftSelector");

    for(auto topic : ft_topics)
    {
        auto cb = [this, topic](const geometry_msgs::WrenchStampedConstPtr& msg)
        {
            if(_select_ft->currentText() != topic)
            {
                return;
            }

            _link_name_map.at(topic) = QString::fromStdString(msg->header.frame_id);

            const double max_force = 1000.0;
            const double max_torque = 200.0;

            setProgressBarValue(_force[0], msg->wrench.force.x, max_force);
            setProgressBarValue(_force[1], msg->wrench.force.y, max_force);
            setProgressBarValue(_force[2], msg->wrench.force.z, max_force);

            setProgressBarValue(_torque[0], msg->wrench.torque.x, max_torque);
            setProgressBarValue(_torque[1], msg->wrench.torque.y, max_torque);
            setProgressBarValue(_torque[2], msg->wrench.torque.z, max_torque);

            auto link_name = QString::fromStdString(msg->header.frame_id);

            emit pointAdded(link_name + "/force/x",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->wrench.force.x));

            emit pointAdded(link_name + "/force/y",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->wrench.force.y));

            emit pointAdded(link_name + "/force/z",
                            QPointF(msg->header.stamp.toSec(),
                                    msg->wrench.force.z));

        };

        auto sub = _nh.subscribe<geometry_msgs::WrenchStamped>(
                    topic.toStdString(),
                    1,
                    cb);

        _subs.push_back(sub);

        _select_ft->addItem(topic);
        _select_ft->setCurrentIndex(i);

        _link_name_map[topic] = "";
    }

    auto plotAllBtn = findChild<QPushButton*>("plotAllBtn");
    connect(plotAllBtn, &QPushButton::released,
            [this]()
    {
        emit seriesAdded(_link_name_map.at(_select_ft->currentText()) +
                         "/force/x");

        emit seriesAdded(_link_name_map.at(_select_ft->currentText()) +
                         "/force/y");

        emit seriesAdded(_link_name_map.at(_select_ft->currentText()) +
                         "/force/z");
    });
}

void FtWidget::setProgressBarValue(QProgressBar* pb, double value, double max)
{
    pb->setValue(100*fabs(value)/max);
    pb->setFormat(QString("%1").arg(value, 5, 'f', 1));
}

bool FtWidgetGroup::init(CustomQtWidget::Args& args)
{
    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    auto l = new QHBoxLayout;
    setLayout(l);

    for(auto i : {0, 1})
    {
        auto w = new FtWidget(i);

        l->addWidget(w);

        _ft_wid.push_back(w);

        connect(w, &FtWidget::seriesAdded,
                this, &CustomQtWidget::seriesAdded);

        connect(w, &FtWidget::pointAdded,
                this, &CustomQtWidget::pointAdded);
    }

    return true;
}

QString FtWidgetGroup::name()
{
    return "Ft";
}

void FtWidgetGroup::update()
{
    CustomQtWidget::update();
}

bool FtWidgetGroup::loadConfig(const YAML::Node &cfg)
{
    int idx = 0;
    if(auto topics = cfg["topics"])
    {
        for(auto t : topics)
        {

            _ft_wid[idx]->getFtComboBox()->setCurrentText(
                        QString::fromStdString(t.as<std::string>())
                        );
        }

        idx++;
    }

    return true;
}

bool FtWidgetGroup::saveConfig(YAML::Node &cfg)
{
    cfg["topics"] = std::vector<std::string>();

    for(auto w : _ft_wid)
    {
        cfg["topics"].push_back(
                    w->getFtComboBox()->currentText().toStdString()
                    );
    }

    return true;
}

QComboBox* FtWidget::getFtComboBox()
{
    return _select_ft;
}

FtWidgetGroup::~FtWidgetGroup()
{

}
