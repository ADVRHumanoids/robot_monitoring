#ifndef PLUGIN_H
#define PLUGIN_H

#include <QWidget>
#include <QPluginLoader>
#include <memory>

namespace XBot { namespace Ui {

class CustomQtWidget : public QWidget
{

    Q_OBJECT

public:

    static CustomQtWidget* MakeInstance(QString libname,
                                        QObject * parent = nullptr);

    class Args;

    virtual bool init(Args&);

    virtual void update();

    virtual QString name();

    virtual ~CustomQtWidget();

signals:

    bool seriesAdded(QString series);

    bool pointAdded(QString series,
                    QPointF point);

private:

    class Impl;
    Impl * impl;
};

}}

Q_DECLARE_INTERFACE(XBot::Ui::CustomQtWidget,
                    "robot_monitoring.CustomQtWidget.1.0.0")



#endif // PLUGIN_H
