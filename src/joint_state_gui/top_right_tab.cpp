#include "top_right_tab.h"
#include "../robot_monitoring/custom_qt_widget_impl.h"

#include <QMenu>
#include <QContextMenuEvent>
#include <QMessageBox>
#include <iostream>
#include <QApplication>

TopRightTab::TopRightTab(QWidget* parent):
    QTabWidget(parent),
    _prefix("libxbot_rob_mon_plugin_"),
    _suffix(".so")
{
    connect(this, &QTabWidget::currentChanged,
            [this](int index)
            {
                for(int i = 0; i < count(); i++)
                    if(i != index)
                        widget(i)->setSizePolicy(QSizePolicy::Ignored,
                                                 QSizePolicy::Ignored);

                widget(index)->setSizePolicy(QSizePolicy::Preferred,
                                             QSizePolicy::Preferred);
                widget(index)->resize(widget(index)->minimumSizeHint());
                widget(index)->adjustSize();
                resize(minimumSizeHint());
                adjustSize();
            });

    connect(this, &QTabWidget::tabCloseRequested,
            [this](int index)
            {
                removeTab(index);
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
                _load_plugin_wid = new LoadPluginWidget(this,
                                                        _prefix,
                                                        _suffix);
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
        _prefix + plugin_name + _suffix,
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

        _wid_to_plugin_name[wid->name()] = plugin_name;

        connect(wid, &XBot::Ui::CustomQtWidget::seriesAdded,
                chart, &QCustomChart::addSeries);

        connect(wid, &XBot::Ui::CustomQtWidget::pointAdded,
                [this](QString name, QPointF point)
                {
                    chart->addPoint(name, point.x(), point.y());
                });

        wid->loadConfig(_cfg[wid->name().toStdString()]);
    }
}


bool TopRightTab::loadConfig(const YAML::Node& cfg)
{
    _cfg = cfg;

    if(auto c = cfg["loaded_widgets"])
    {
        for(auto w : c)
        {
            load(QString::fromStdString(w.as<std::string>()));
        }
    }

    return true;
}

bool TopRightTab::saveConfig(YAML::Node& cfg)
{
    cfg["loaded_widgets"] = std::vector<std::string>();

    for(int i = 1; i < count(); i++)  // skip first (joint)
    {
        auto plname = _wid_to_plugin_name.at(tabText(i));
        cfg["loaded_widgets"].push_back(plname.toStdString());

        if(auto w = dynamic_cast<XBot::Ui::CustomQtWidget*>(widget(i)))
        {
            auto wid_cfg = cfg[plname.toStdString()];
            w->saveConfig(wid_cfg);
            cfg[tabText(i).toStdString()] = wid_cfg;
        }
    }

    return true;
}

QString TopRightTab::name() const
{
    return "top_right_tab";
}
