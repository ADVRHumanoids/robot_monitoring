#include "chart.h"

#include <QGraphicsLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QLegendMarker>
#include <QtUiTools/QUiLoader>
#include <QCheckBox>
#include <QPushButton>

#include <QApplication>

void qrc_init()
{
    Q_INIT_RESOURCE(chart_resources);
}

namespace  {


QWidget * LoadUiFile(QWidget * parent, QString name)
{
    qrc_init();

    QUiLoader loader;

    QFile file(":/" + name + ".ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;

}

}

ChartWidget::ChartWidget(QWidget * parent):
    _auto_y_range(true),
    _x_range(10.0),
    _autoscroll(true),
    _t(.0),
    _empty_chart(true)
{
    /* Chart */
    _chart = new QChart;
    _chart->legend()->setAlignment(Qt::AlignRight);

    /* Axes */
    _axis_x = new QValueAxis;
    _axis_y = new QValueAxis;
    _axis_x->setRange(-_x_range, 0.0);
    _axis_y->setRange(-1., 1.);

    _chart->addAxis(_axis_x, Qt::AlignBottom);
    _chart->addAxis(_axis_y, Qt::AlignLeft);

    // Formatting
    _chart->setBackgroundVisible(false);
    _chart->setMargins(QMargins(0,0,0,0));
    _chart->layout()->setContentsMargins(0,0,0,0);
    _chart->setPlotAreaBackgroundBrush(QBrush(Qt::black));
    _chart->setPlotAreaBackgroundVisible(true);
    _chart_view = new CustomChartView(_chart);
    _chart_view->setRubberBand(QChartView::RectangleRubberBand);
    _chart_view->setRenderHint(QPainter::Antialiasing, true);


    auto hlayout = new QVBoxLayout;
    setLayout(hlayout);
    hlayout->addWidget(_chart_view);

    _scroll_timer = new QTimer;
    _timer_period_ms = 40.;
    _scroll_timer->setInterval(int(_timer_period_ms + 0.5));

    auto right_panel = LoadUiFile(this, "right_panel");
    hlayout->addWidget(right_panel);

    connect(_scroll_timer, &QTimer::timeout,
            this, &ChartWidget::on_timer_event);

    connect(_chart_view, &CustomChartView::resetViewEvent,
            this, &ChartWidget::reset_view);

    connect(_chart_view, &CustomChartView::customViewEvent,
            [this]()
    {
        _auto_y_range = false;
    });

    auto autoscroll = right_panel->findChild<QCheckBox*>("autoscroll");
    autoscroll->setCheckState(Qt::CheckState::Checked);
    connect(autoscroll, &QCheckBox::toggled,
            [this](bool toggled)
    {
        _autoscroll = toggled;
    });

    auto removeall = right_panel->findChild<QPushButton*>("removeAll");
    connect(removeall, &QPushButton::released,
            [this]()
    {
        for(auto p : _series)
        {
            removeSeries(p.first);
        }
    });

    auto resetview = right_panel->findChild<QPushButton*>("resetView");
    connect(resetview, &QPushButton::released,
            this, &ChartWidget::reset_view);

    _fps_label = right_panel->findChild<QLabel*>("fpsLabel");

    _scroll_timer->start();
}

void ChartWidget::setDefaultRange(double range)
{
    _x_range = range;
}

void ChartWidget::addSeries(QString name)
{
    if(_series.count(name) > 0)
    {
        return;
    }

    auto series = new QLineSeries;
    series->setName(name);
    series->setUseOpenGL();
    _chart->addSeries(series);
    series->attachAxis(_axis_x);
    series->attachAxis(_axis_y);


    _series[name] = series;

    auto markers = _chart->legend()->markers(series);

    for(auto m : markers)
    {
        connect(m, &QLegendMarker::hovered,
                this, &ChartWidget::legend_marker_hovered);

        connect(m, &QLegendMarker::clicked,
                this, &ChartWidget::legend_marker_clicked);

    }

    reset_view();

}

void ChartWidget::removeSeries(QString name)
{
    {
        auto it = _series.find(name);

        if(it == _series.end())
        {
            return;
        }

        _chart->removeSeries(it->second);
        _series.erase(it);
    }

    {
        auto it = _point_to_add.find(name);

        if(it == _point_to_add.end())
        {
            return;
        }

        _point_to_add.erase(it);
    }


    if(_series.empty())
    {
        _empty_chart = true;
    }

}

void ChartWidget::addPoint(QString name,
                           double t,
                           double val)
{
    auto it = _series.find(name);

    if(it == _series.end())
    {
        return;
    }

    _point_to_add[name] = QPointF(t, val);

    //    it->second->append(t, val);
    _t = t;

    if(_auto_y_range)
    {
        if(val + 0.1*std::fabs(val) > _axis_y->max())
        {
            _axis_y->setMax(val + 0.1*std::fabs(val));
        }

        if(val - 0.1*std::fabs(val) < _axis_y->min())
        {
            _axis_y->setMin(val - 0.1*std::fabs(val));
        }
    }

    if(_empty_chart)
    {
        _empty_chart = false;
        reset_view();
    }
}

void ChartWidget::on_timer_event()
{
    auto tic = std::chrono::high_resolution_clock::now();

    for(const auto& p : _point_to_add)
    {
        _series.at(p.first)->append(p.second);
    }

    _point_to_add.clear();

    if(_autoscroll)
    {
        auto range = _axis_x->max() - _axis_x->min();
        auto dx = _chart->plotArea().width() / range * _timer_period_ms * 0.001;
        _chart->scroll(dx, 0);
    }

    auto toc = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> dt = toc - tic;
    int fps = 1/dt.count();

    _fps_label->setText(QString("%1 FPS").arg(fps));


}

void ChartWidget::legend_marker_hovered(bool hover)
{
    auto* marker = qobject_cast<QLegendMarker*>(sender());
    Q_ASSERT(marker);

    QFont font = marker->font();
    font.setBold(hover);
    marker->setFont(font);

    if (marker->series()->type() == QAbstractSeries::SeriesTypeLine)
    {
        auto series = qobject_cast<QLineSeries*>(marker->series());
        auto pen = series->pen();
        pen.setWidth(hover ? (pen.width() * 3) : (pen.width() / 3));
        series->setPen(pen);
    }

}

void ChartWidget::legend_marker_clicked()
{
    auto* marker = qobject_cast<QLegendMarker*>(sender());
    Q_ASSERT(marker);

    // Toggle visibility of series
    setSeriesVisible(marker->series(), !marker->series()->isVisible());
}

void ChartWidget::reset_view()
{

    _auto_y_range = true;
    _axis_x->setRange(_t - _x_range, _t);

    double y_min =  1e10;
    double y_max = -1e10;

    bool point_found = false;

    for(auto p : _series)
    {
        auto s = p.second;
        auto points = s->pointsVector();
        if(!points.empty()) point_found = true;

        auto comp = [](const QPointF& a, const QPointF& b)
        {
            return a.y() < b.y();
        };

        auto it_max = std::max_element(points.begin(), points.end(),
                                       comp);
        auto it_min = std::min_element(points.begin(), points.end(),
                                       comp);

        y_min = std::min(y_min, it_min->y());
        y_max = std::max(y_max, it_max->y());

    }

    auto range = y_max - y_min;
    range = std::max(range, 1.0);
    y_min = y_min - 0.1*(range);
    y_max = y_max + 0.1*(range);

    if(point_found)
    {
        _axis_y->setRange(y_min, y_max);
    }

}

void ChartWidget::setSeriesVisible(QAbstractSeries *series, bool visible)
{
    series->setVisible(visible);
    for (QLegendMarker *marker : _chart->legend()->markers(series)) {
        // Turn legend marker back to visible, since hiding series also hides the marker
        // and we don't want it to happen now.
        marker->setVisible(true);

        // Dim the marker, if series is not visible
        qreal alpha = visible ? 1.0 : 0.5;
        QColor color;
        QBrush brush = marker->labelBrush();
        color = brush.color();
        color.setAlphaF(alpha);
        brush.setColor(color);
        marker->setLabelBrush(brush);

        brush = marker->brush();
        color = brush.color();
        color.setAlphaF(alpha);
        brush.setColor(color);
        marker->setBrush(brush);

        QPen pen = marker->pen();
        color = pen.color();
        color.setAlphaF(alpha);
        pen.setColor(color);
        marker->setPen(pen);
    }

    for (QAbstractAxis *axis : _chart->axes(Qt::Vertical)) {
        bool hideAxis = true;
        for (QAbstractSeries *series : _chart->series()) {
            for (QAbstractAxis *attachedAxis : series->attachedAxes()) {
                if (series->isVisible() && attachedAxis == axis) {
                    hideAxis = false;
                    break;
                }
            }
            if (!hideAxis)
                break;
        }
        axis->setVisible(!hideAxis);
    }
}


CustomChartView::CustomChartView(QChart* chart, QWidget* parent):
    QChartView (chart, parent)
{
    setDragMode(QGraphicsView::NoDrag);
    setMouseTracking(true);

}

void CustomChartView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton)
    {
        QApplication::setOverrideCursor(QCursor(Qt::SizeAllCursor));
        event->accept();
    }

    _lastMousePos = event->pos();
    _firstMousePos = _lastMousePos;

    emit customViewEvent();

    QChartView::mousePressEvent(event);
}

void CustomChartView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton)
    {
        event->accept();
        return;
    }

    QChartView::mouseReleaseEvent(event);
}

void CustomChartView::mouseMoveEvent(QMouseEvent* event)
{
    // pan the chart with a middle mouse drag
    if (event->buttons() & Qt::MiddleButton)
    {
        auto dPos = event->pos() - _lastMousePos;
        chart()->scroll(-dPos.x(), dPos.y());

        _lastMousePos = event->pos();
        event->accept();

        QApplication::restoreOverrideCursor();
    }

    if(event->buttons() & Qt::RightButton)
    {
        auto dPos = event->pos() - _lastMousePos;
        _lastMousePos = event->pos();

        double factor_x = 1 - 10.*dPos.x()/chart()->plotArea().width();
        double factor_y = 1 - 10.*dPos.y()/chart()->plotArea().height();

        factor_x = std::min(10., std::max(0.1, factor_x));
        factor_y = std::min(10., std::max(0.1, factor_y));

        QRectF r = QRectF(chart()->plotArea().left(),
                          chart()->plotArea().top(),
                          chart()->plotArea().width()/factor_x,
                          chart()->plotArea().height()/factor_y);

        chart()->zoomIn(r);

        QPointF mousePos = mapFromGlobal(QCursor::pos());

        QPointF delta = chart()->plotArea().center() - mousePos;
        chart()->scroll(-dPos.x(), dPos.y());
    }


    QChartView::mouseMoveEvent(event);
}

void CustomChartView::wheelEvent(QWheelEvent* event)
{
    qreal factor;
    if ( event->delta() > 0 )
        factor = 1.33;
    else
        factor = 0.75;

    QRectF r = QRectF(chart()->plotArea().left(),
                      chart()->plotArea().top(),
                      chart()->plotArea().width()/factor,
                      chart()->plotArea().height()/factor);

    QPointF mousePos = mapFromGlobal(QCursor::pos());
    r.moveCenter(mousePos);
    chart()->zoomIn(r);
    QPointF delta = chart()->plotArea().center() - mousePos;
    chart()->scroll(delta.x(), -delta.y());
}
