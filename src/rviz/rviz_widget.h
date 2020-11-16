#ifndef RVIZ_WIDGET_H
#define RVIZ_WIDGET_H

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

#include "../plugin/custom_qt_widget.h"

class RvizWidget : public XBot::Ui::CustomQtWidget
{

    Q_OBJECT
    Q_PLUGIN_METADATA(IID "robot_monitoring.CustomQtWidget.1.0.0" FILE "rviz_widget.json")
    Q_INTERFACES(XBot::Ui::CustomQtWidget)

public:

    RvizWidget(QWidget * parent = nullptr);

    bool init(Args&) override;
    void update() override;
    QString name() override;

private:

    rviz::VisualizationManager* _manager;
    rviz::RenderPanel* _render_panel;
    rviz::Display* _robot_model;


    // QWidget interface
protected:
    void contextMenuEvent(QContextMenuEvent* event) override;
};

#endif // RVIZ_WIDGET_H
