#ifndef CHART_H
#define CHART_H

#include <QChart>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QLabel>

QT_CHARTS_USE_NAMESPACE

class CustomChartView : public QChartView
{

    Q_OBJECT

public:

    CustomChartView(QChart* chart, QWidget *parent = nullptr);

signals:

    void resetViewEvent();
    void customViewEvent();

protected:

    virtual void mousePressEvent(QMouseEvent *event) override;
    virtual void mouseReleaseEvent(QMouseEvent *event) override;
    virtual void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:

    QPointF _lastMousePos, _firstMousePos;
};

class ChartWidget : public QWidget
{

public:

    ChartWidget(QWidget * parent = nullptr);
    void setDefaultRange(double range);
    void addSeries(QString name);
    void removeSeries(QString name);
    void addPoint(QString name, double t, double x);

private:

    void on_timer_event();
    void legend_marker_hovered(bool hover);
    void legend_marker_clicked();
    void reset_view();

    void setSeriesVisible(QAbstractSeries *series, bool visible);

    std::map<QString, QLineSeries *> _series;
    std::map<QString, QPointF> _point_to_add;
    QChart * _chart;
    CustomChartView * _chart_view;
    QValueAxis * _axis_x, * _axis_y;
    bool _auto_y_range;
    bool _autoscroll;
    double _x_range;
    double _t;
    bool _empty_chart;

    QTimer * _scroll_timer;
    double _timer_period_ms;
    QLabel * _fps_label;
};

#endif // CHART_H
