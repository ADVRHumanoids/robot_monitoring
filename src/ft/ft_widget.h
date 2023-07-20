#ifndef FT_WIDGET_H
#define FT_WIDGET_H

#include "robot_monitoring/custom_qt_widget.h"

#include <QProgressBar>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>

#include <ros/ros.h>

namespace XBot { namespace Ui {

class FtWidget : public QWidget
{
    Q_OBJECT

public:

    FtWidget(int i = 0);

    QComboBox* getFtComboBox();

signals:

    bool seriesAdded(QString series);

    bool pointAdded(QString series,
                    QPointF point);


private:

    static void setProgressBarValue(QProgressBar * pb, double value, double max);

    ros::NodeHandle _nh;
    std::vector<ros::Subscriber> _subs;
    std::vector<QProgressBar*> _force, _torque;
    QComboBox * _select_ft;
    std::map<QString, QString> _link_name_map;

};

class FtWidgetGroup : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "ft_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    bool init(Args&) override;

    QString name() override;

    void update() override;

    bool loadConfig(const YAML::Node &cfg);

    bool saveConfig(YAML::Node &cfg);

    ~FtWidgetGroup() override;

private:

    std::vector<FtWidget*> _ft_wid;

};

}}

#endif // Ft_WIDGET_H
