#ifndef LOAD_PLUGIN_WID_H
#define LOAD_PLUGIN_WID_H

#include <QWidget>
#include <QDialog>
#include <QPushButton>
#include <QListWidget>
#include <QListWidgetItem>

class LoadPluginWidget : public QDialog
{

public:

    LoadPluginWidget(QWidget * parent = nullptr);

    QString prefix, suffix;
    QStringList selected_plugins;

private:

};

#endif // LOAD_PLUGIN_WID_H
