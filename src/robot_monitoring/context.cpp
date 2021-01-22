#include <robot_monitoring/context.h>

#include <QFile>
#include <QDir>
#include <QMessageBox>

using namespace XBot::Ui;

class Context::Impl
{

public:

    Impl();

    void saveConfig(YAML::Node node);

    ~Impl();

    YAML::Node node;
    bool node_changed;

    QFile file;
};

Context::Context()
{
    impl = std::make_unique<Impl>();
}

YAML::Node& Context::config()
{
    return impl->node;
}

void Context::saveConfig(YAML::Node node)
{
    impl->saveConfig(node);
}

Context::~Context()
{

}

Context::Impl::Impl():
    node_changed(false)
{
    QString home = std::getenv("HOME");
    QDir().mkpath(home + "/.xbot2/ui");
    file.setFileName(home + "/.xbot2/ui/perspective.yaml");

    try
    {
        node = YAML::LoadFile(file.fileName().toStdString());
        printf("loaded config is: \n"
               "%s \n", file.fileName().toStdString().c_str());
    }
    catch(YAML::BadFile& e)
    {
        printf("no config available \n");
    }


}

void Context::Impl::saveConfig(YAML::Node _node)
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr,
                                  "Message",
                                  "Save perspective file?",
                                  QMessageBox::Yes|QMessageBox::No);

    if(reply == QMessageBox::No)
    {
        return;
    }

    std::stringstream ss;
    ss << _node << "\n";
    auto str = ss.str();

    file.open(QFile::WriteOnly);
    file.write(str.data(), str.length());
    file.flush();
}

Context::Impl::~Impl()
{
}
