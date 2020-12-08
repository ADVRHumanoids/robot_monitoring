#include <robot_monitoring/context.h>

#include <QFile>
#include <QDir>
#include <QMessageBox>

#include <fmt/format.h>
#include <fmt/ostream.h>

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

YAML::Node Context::config()
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
    node = YAML::LoadFile(file.fileName().toStdString());

    fmt::print("loaded config is: \n"
               "{} \n", node);

}

void Context::Impl::saveConfig(YAML::Node _node)
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr,
                                  "Message",
                                  "Save perspective file?",
                                  QMessageBox::Yes|QMessageBox::No);

    fmt::print("{}\n", __LINE__);

    if(reply == QMessageBox::No)
    {
        fmt::print("{}\n", __LINE__);
        return;
    }

    fmt::print("{}\n", __LINE__);

    std::stringstream ss;
    ss << _node << "\n";
    auto str = ss.str();

    fmt::print("{}\n", str);

    file.open(QFile::WriteOnly);
    fmt::print("{}, {} \n",
               file.write(str.data(), str.length()),
               file.flush());
}

Context::Impl::~Impl()
{
}
