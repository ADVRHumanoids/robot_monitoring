#include "qcustom_chart.h"
#include "qcustomplot.h"

#include <QUiLoader>

void qrc_init()
{
    Q_INIT_RESOURCE(qcustomchart_resources);
}

namespace
{

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


QCustomChart::QCustomChart(QWidget* parent):
    QWidget(parent),
    _last_point_t(0.0),
    _autoscroll(true),
    _autorange(true),
    _y_min(1e3),
    _y_max(-1e3)
{
    _plt = new QCustomPlot;

    auto l = new QHBoxLayout;
    l->addWidget(_plt);
    l->setStretch(0, 1);

    auto panel = LoadUiFile(this, "right_panel");
    l->addWidget(panel);
    l->setStretch(1, 0);

    setLayout(l);

    _scroll_timer = new QTimer(this);
    _timer_period_ms = 40.;
    _scroll_timer->setInterval(int(_timer_period_ms + 0.5));
    connect(_scroll_timer, &QTimer::timeout,
            this, &QCustomChart::on_timer_event);

    _plt->setMultiSelectModifier(Qt::KeyboardModifier::ControlModifier);

    _plt->xAxis->setLabel("time [s]");

    QFont legendFont = font();  // start out with Widget's font..
    legendFont.setPointSize(9); // and make a bit smaller for legend
    _plt->legend->setFont(legendFont);
    _plt->legend->setBrush(QBrush(QColor(255,255,255,230)));
    _plt->axisRect()->insetLayout()->setInsetAlignment(0,
                                                       Qt::AlignBottom|Qt::AlignLeft);
    _plt->axisRect()->setRangeDrag(static_cast<Qt::Orientation>(0));

    // set axes ranges, so we see all data:
    _plt->xAxis->setRange(0, 15);
    _plt->yAxis->setRange(-1.0, 1.0);
    _plt->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    connect(_plt, &QCustomPlot::legendClick,
            this, &QCustomChart::legendSingleClick);

    connect(_plt, &QCustomPlot::legendDoubleClick,
            this, &QCustomChart::legendDoubleClick);

    connect(_plt, &QCustomPlot::mousePress,
            this, &QCustomChart::mousePress);

    connect(_plt, &QCustomPlot::mouseMove,
            this, &QCustomChart::mouseMove);

    auto autoscrollChkBox = findChild<QCheckBox*>("autoscrollChkBox");
    connect(autoscrollChkBox,
            &QCheckBox::stateChanged,
            [this](int state)
            {
                _autoscroll = state;

                if(_autoscroll)
                {
                    auto orient = _plt->axisRect()->rangeDrag();
                    orient &= ~Qt::Horizontal;
                    _plt->axisRect()->setRangeDrag(orient);
                }
                else
                {
                    auto orient = _plt->axisRect()->rangeDrag();
                    orient |= Qt::Horizontal;
                    _plt->axisRect()->setRangeDrag(orient);
                }

            });

    auto autorangeChkBox = findChild<QCheckBox*>("autorangeChkBox");
    connect(autorangeChkBox,
            &QCheckBox::stateChanged,
            [this](int state)
            {
                _autorange = state;

                if(_autorange)
                {
                    auto orient = _plt->axisRect()->rangeDrag();
                    orient &= ~Qt::Vertical;
                    _plt->axisRect()->setRangeDrag(orient);
                }
                else
                {
                    auto orient = _plt->axisRect()->rangeDrag();
                    orient |= Qt::Vertical;
                    _plt->axisRect()->setRangeDrag(orient);
                }

            });

    connect(findChild<QPushButton*>("removeAllBtn"),
            &QPushButton::released,
            [this]()
            {
                removeAll();
            });

    connect(findChild<QPushButton*>("resetViewBtn"),
            &QPushButton::released,
            [this, autoscrollChkBox, autorangeChkBox]()
            {
                resetView();
                autoscrollChkBox->setCheckState(Qt::CheckState::Checked);
                autorangeChkBox->setCheckState(Qt::CheckState::Checked);

            });

    connect(findChild<QPushButton*>("savePngBtn"),
            &QPushButton::released,
            [this]()
            {
                const QDateTime now = QDateTime::currentDateTime();
                auto now_str = now.toString("yyyyMMdd_hhmmss_zzz");

                auto aaelems = _plt->antialiasedElements();
                _plt->setAntialiasedElements(QCP::aeAll);
                _plt->savePng("/tmp/xbot2_gui_chart__" + now_str + ".png",
                              0, 0, 3.0);
                _plt->setAntialiasedElements(aaelems);

            });

    _scroll_timer->start();
}

void QCustomChart::setDefaultRange(double range)
{

}

void QCustomChart::addSeries(QString name)
{
    if(_graphs.count(name))
    {
        return;
    }

    auto g = _graphs[name] = _plt->addGraph();
    g->setName(name);

    QPen pen(pick_color());
    pen.setWidth(2);
    g->setPen(pen);
    g->setAntialiased(true);

    _plt->legend->setVisible(true);

}

void QCustomChart::removeSeries(QString name)
{
    _plt->removeGraph(_graphs.at(name));
}

void QCustomChart::addPoint(QString name, double t, double x)
{
    static auto t0 = t;

    auto it = _graphs.find(name);

    if(it == _graphs.end())
    {
        return;
    }

    auto graph_empty = it->second->data()->size() == 0;

    _last_point_t = t - t0;
    it->second->addData(_last_point_t, x);

    _y_min = std::min(_y_min, x);
    _y_max = std::max(_y_max, x);

    if(graph_empty)
    {
        resetView();
    }
}

void QCustomChart::removeAll()
{
    _plt->clearGraphs();
    _graphs.clear();
    _plt->legend->setVisible(false);

    _y_min = 1e3;
    _y_max = -1e3;

}

void QCustomChart::resetView()
{
    auto range = std::max(1.0, _y_max - _y_min);
    _plt->yAxis->setRange(_y_min - 0.1*range,
                          _y_max + 0.1*range);

}

void QCustomChart::on_timer_event()
{
    if(_autoscroll)
    {
        _plt->xAxis->setRange(_last_point_t,
                              _plt->xAxis->range().size(),
                              Qt::AlignRight);

    }

    if(_autorange)
    {
        resetView();
    }

    _plt->replot();
}

QColor QCustomChart::pick_color()
{
    static std::vector<QColor> colormap {
        QColor(0.0000*256,    0.4470*256,    0.7410*256),
        QColor(0.8500*256,    0.3250*256,    0.0980*256),
        QColor(0.9290*256,    0.6940*256,    0.1250*256),
        QColor(0.4940*256,    0.1840*256,    0.5560*256),
        QColor(0.4660*256,    0.6740*256,    0.1880*256),
        QColor(0.3010*256,    0.7450*256,    0.9330*256),
        QColor(0.6350*256,    0.0780*256,    0.1840*256)
    };

    static int idx = 0;

    return colormap[idx++ % colormap.size()];
}

void QCustomChart::mousePress(QMouseEvent* event)
{

    if(_plt->plottable()->selectTest(event->pos(), false) < 0)
    {
        return;
    }

    switch(event->button())
    {
    case Qt::MouseButton::LeftButton:
    {
        _plt->setSelectionRectMode(QCP::SelectionRectMode::srmNone);
        break;
    }
    case Qt::MouseButton::RightButton:
    {
        _plt->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);
        break;
    }
    }
}

void QCustomChart::mouseMove(QMouseEvent* event)
{
    auto over_legend = _plt->legend->selectTest(event->pos(), false) >= 0;

    for(int i = 0; i < _plt->legend->itemCount(); i++)
    {
        auto item = _plt->legend->item(i);
        auto pl_item = qobject_cast<QCPPlottableLegendItem*>(item);
        auto g = pl_item->plottable();

        bool item_selected = over_legend &&
                             item->selectTest(event->pos(), false) >= 0;

        auto pen = g->pen();
        pen.setWidth(item_selected ? 4 : 2);
        g->setPen(pen);

        auto font = item->font();
        font.setBold(item_selected);
        item->setFont(font);
    }
}

void QCustomChart::legendSingleClick(QCPLegend* legend,
                                     QCPAbstractLegendItem* item,
                                     QMouseEvent* event)
{
    // only react if item was clicked
    // (user could have clicked on border padding)
    if(item)
    {
        auto pl_item = qobject_cast<QCPPlottableLegendItem*>(item);
        auto plottable = pl_item->plottable();
        auto gname = plottable->name();
        plottable->setVisible(!plottable->visible());

        auto font = item->font();
        font.setStrikeOut(!plottable->visible());
        item->setFont(font);

    }
}

void QCustomChart::legendDoubleClick(QCPLegend* legend,
                                     QCPAbstractLegendItem* item,
                                     QMouseEvent* event)
{
    // only react if item was clicked
    // (user could have clicked on border padding)
    if(item)
    {
        auto pl_item = qobject_cast<QCPPlottableLegendItem*>(item);
        auto plottable = pl_item->plottable();
        auto gname = plottable->name();
        _plt->removePlottable(plottable);
        _graphs.erase(_graphs.find(gname));
    }
}
