#ifndef TOP_RIGHT_TAB_H
#define TOP_RIGHT_TAB_H

#include <QTabWidget>
#include "load_plugin_wid.h"
#include "../qcustomplot/qcustom_chart.h"

#include "robot_monitoring/custom_qt_widget.h"

class TopRightTab : public QTabWidget
{

public:

    TopRightTab(QWidget * parent = nullptr);

    bool loadConfig(const YAML::Node& cfg);
    bool saveConfig(YAML::Node& cfg);
    QString name() const;

    QCustomChart * chart;

protected:

    void contextMenuEvent(QContextMenuEvent*) override;

private:

    LoadPluginWidget * _load_plugin_wid;

    QString _prefix, _suffix;

    std::map<QString, QString> _wid_to_plugin_name;

    void load(QString plugin_name);

};

#endif // TOP_RIGHT_TAB_H
