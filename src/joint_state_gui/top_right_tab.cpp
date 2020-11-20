#include "top_right_tab.h"
#include "../plugin/custom_qt_widget_impl.h"

#include <QMenu>
#include <QContextMenuEvent>
#include <QMessageBox>
#include <iostream>
#include <QApplication>

TopRightTab::TopRightTab(QWidget* parent):
    QTabWidget(parent)
{
    connect(this, &QTabWidget::currentChanged,
            [this](int index)
            {
                for(int i=0;i<count();i++)
                    if(i!=index)
                        widget(i)->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

                widget(index)->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
                widget(index)->resize(widget(index)->minimumSizeHint());
                widget(index)->adjustSize();
                resize(minimumSizeHint());
                adjustSize();
            });

    setTabsClosable(true);

}

void TopRightTab::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);

    auto * load_action = new QAction("Load plugin", this);

    connect(load_action, &QAction::triggered,
            [this, event]()
            {
                _load_plugin_wid = new LoadPluginWidget(this);
                auto geom = _load_plugin_wid->geometry();
                geom.setTopLeft(event->globalPos());
                _load_plugin_wid->setGeometry(geom);
                _load_plugin_wid->exec();

                for(auto p: _load_plugin_wid->selected_plugins)
                {
                    load(p);
                }
            });

    menu.addAction(load_action);

    menu.exec(event->globalPos());
}

void TopRightTab::load(QString plugin_name)
{
    // test plugin
    auto wid = XBot::Ui::CustomQtWidget::MakeInstance(
        _load_plugin_wid->prefix + plugin_name + _load_plugin_wid->suffix,
        this);

    if(!wid)
    {
        QMessageBox msgBox;
        msgBox.setText("Unable to load '" + plugin_name + "'");
        msgBox.exec();
        return;
    }
    else
    {
        XBot::Ui::CustomQtWidget::Args args;

        if(!wid->init(args))
        {
            QMessageBox msgBox;
            msgBox.setText("Unable to initialize '" + plugin_name + "'");
            msgBox.exec();
            return;
        }

        addTab(wid, wid->name());

        connect(wid, &XBot::Ui::CustomQtWidget::seriesAdded,
                chart, &QCustomChart::addSeries);

        connect(wid, &XBot::Ui::CustomQtWidget::pointAdded,
                [this](QString name, QPointF point)
                {
                    chart->addPoint(name, point.x(), point.y());
                });
    }
}
