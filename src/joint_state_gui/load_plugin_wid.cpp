#include "load_plugin_wid.h"

#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QEvent>
#include <QMouseEvent>
#include <QFrame>
#include <QDir>

#include <iostream>

void load_plugin_widget_qrc_init()
{
    Q_INIT_RESOURCE(ui_resources);
}

namespace  {

QWidget * LoadUiFile(QWidget * parent)
{

    load_plugin_widget_qrc_init();

    QUiLoader loader;

    QFile file(":/load_plugin.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;

}

}


LoadPluginWidget::LoadPluginWidget(QWidget* parent)
{
    auto wid = LoadUiFile(this);
    auto l = new QHBoxLayout;
    l->addWidget(wid);
    setLayout(l);

    prefix = "libxbot_rob_mon_plugin_";
    suffix = ".so";

    auto list = findChild<QListWidget*>("pluginList");

    QStringList dir_list = {"/usr/lib", "/usr/local/lib"};

    auto ld_lib_path_env = std::getenv("LD_LIBRARY_PATH");

    if(ld_lib_path_env)
    {
        auto ld_dirs = QString(ld_lib_path_env).split(':');

        dir_list.append(ld_dirs);
    }

    for(auto dir : dir_list)
    {
        std::cout << "processing dir: " << dir.toStdString() << std::endl;

        QDir directory(dir);

        auto files = directory.entryList(QStringList() << prefix + "*" + suffix,
                                         QDir::Files);

        for(auto f: files)
        {
            auto plname = f.mid(prefix.length(),
                                f.length() - prefix.length() - suffix.length());

            list->addItem(plname);
        }
    }

    auto load = findChild<QPushButton*>("loadBtn");
    connect(load, &QPushButton::released,
            [this, list]()
            {
                for(auto w : list->selectedItems())
                {
                    selected_plugins.append(w->text());
                }

                done(0);
            });
}

