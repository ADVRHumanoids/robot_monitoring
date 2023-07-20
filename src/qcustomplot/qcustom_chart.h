#ifndef QCUSTOM_CHART_H
#define QCUSTOM_CHART_H

#include <robot_monitoring/custom_qt_widget.h>

#include "qcustomplot.h"

class QCustomChart : public XBot::Ui::CustomQtWidget
{

public:


    QCustomChart(QWidget * parent = nullptr);
    void setDefaultRange(double range);
    void addSeries(QString name);
    void removeSeries(QString name);
    void addPoint(QString name, double t, double x);
    void removeAll();
    void resetView();

    bool loadConfig(const YAML::Node &cfg);
    bool saveConfig(YAML::Node &cfg);
    QString name();

private:

    void on_timer_event();

    QColor pick_color();

    QCustomPlot * _plt;

    std::map<QString, QCPGraph*> _graphs;

    QTimer * _scroll_timer;
    double _timer_period_ms;

    double _last_point_t;
    double _y_min, _y_max;

    bool _autoscroll;
    bool _autorange;

    void mousePress(QMouseEvent *event);

    void mouseMove(QMouseEvent* event);

    void legendDoubleClick(QCPLegend *legend,
                           QCPAbstractLegendItem *item,
                           QMouseEvent *event);

    void legendSingleClick(QCPLegend* legend,
                           QCPAbstractLegendItem* item,
                           QMouseEvent* event);


};

#endif // QCUSTOM_CHART_H
