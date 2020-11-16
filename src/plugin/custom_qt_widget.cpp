#include "custom_qt_widget.h"
#include "custom_qt_widget_impl.h"
#include <fmt/format.h>

#include <cstdio>
#include <memory>
#include <string>
#include <dlfcn.h>
#include <link.h>

using namespace XBot::Ui;


inline std::string GetLibPath(std::string lib_name)
{
    /* Try to open the provided library */
    std::shared_ptr<void> lib_handle(dlopen(lib_name.c_str(), RTLD_NOW),
                                     [](void * ptr)
                                     {
                                         if(!ptr) return;
                                         dlclose(ptr);
                                     });

    /* Not able to open so, report error */
    if(!lib_handle)
    {
        fmt::print(stderr,
                    "could not open lib '{}', error '{}' \n",
                   lib_name, dlerror());
        return "";
    }
    else
    {
        struct link_map* map;

        dlinfo(lib_handle.get(), RTLD_DI_LINKMAP, &map);

        if(!map)
        {
            return "";
        }

        std::shared_ptr<char> real(realpath(map->l_name, nullptr),
                                   [](char * ptr)
                                   {
                                       if(!ptr) return;
                                       std::free(ptr);
                                   });

        if(!real)
        {
            return "";
        }

        std::string ret = real.get();

        return ret;

    }
}

CustomQtWidget* CustomQtWidget::MakeInstance(QString libname,
                                             QObject * parent)
{
    auto libpath = GetLibPath(libname.toStdString());

    if(libpath.empty())
    {
        return nullptr;
    }

    QPluginLoader loader(QString::fromStdString(libpath), parent);
    return dynamic_cast<CustomQtWidget*>(loader.instance());
}

class CustomQtWidget::Impl
{

};

bool CustomQtWidget::init(Args& args)
{
    impl = new Impl;
    return true;
}

void CustomQtWidget::update()
{

}

QString CustomQtWidget::name()
{
    return "NoName";
}

CustomQtWidget::~CustomQtWidget()
{
    delete impl;
}

//#include "moc_custom_qt_widget.cpp"
