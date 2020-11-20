#ifndef TOP_RIGHT_TAB_H
#define TOP_RIGHT_TAB_H

#include <QTabWidget>
#include "load_plugin_wid.h"
#include "../qcustomplot/qcustom_chart.h"

class TopRightTab : public QTabWidget
{

public:

    TopRightTab(QWidget * parent = nullptr);

    QCustomChart * chart;

protected:

    void contextMenuEvent(QContextMenuEvent*) override;

private:

    LoadPluginWidget * _load_plugin_wid;

    void load(QString plugin_name);
};

#endif // TOP_RIGHT_TAB_H
