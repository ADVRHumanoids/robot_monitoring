#ifndef Maintenance_WIDGET_H
#define Maintenance_WIDGET_H

#include "robot_monitoring/custom_qt_widget.h"

#include <QPushButton>
#include <QCheckBox>
#include <QTimer>
#include <QProcess>

#include <ros/ros.h>

namespace XBot { namespace Ui {

class MaintenanceWidget : public CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "maintenance_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    MaintenanceWidget();

    bool init(Args&) override;

    QString name() override;

    void update() override;

    ~MaintenanceWidget() override;

private:

    QProcess ssh;
    QString user_host;



    // CustomQtWidget interface
public:
    bool loadConfig(const YAML::Node &cfg) override;
    bool saveConfig(YAML::Node &cfg) override;
};

}}

#endif // Maintenance_WIDGET_H
