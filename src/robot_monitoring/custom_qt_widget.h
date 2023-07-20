#ifndef __ROBOT_MONITORING_PLUGIN_H__
#define __ROBOT_MONITORING_PLUGIN_H__

#include <QWidget>
#include <QPluginLoader>
#include <memory>

#include <robot_monitoring/context.h>

namespace XBot { namespace Ui {

class CustomQtWidget : public QWidget
{

    Q_OBJECT

public:

    static CustomQtWidget* MakeInstance(QString libname,
                                        QObject * parent = nullptr);

    class Args;

    virtual bool init(Args& args);

    virtual bool loadConfig(const YAML::Node& cfg);

    virtual bool saveConfig(YAML::Node& cfg);

    virtual bool usesOpenGl() const;

    virtual void update();

    virtual QString name();

    virtual ~CustomQtWidget();

signals:

    bool seriesAdded(QString series);

    bool pointAdded(QString series,
                    QPointF point);

protected:

    Context& context();

private:

    class Impl;
    Impl * impl;
};

}}

Q_DECLARE_INTERFACE(XBot::Ui::CustomQtWidget,
                    "robot_monitoring.CustomQtWidget.1.0.0")



#endif // PLUGIN_H
