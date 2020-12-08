#ifndef __ROBOT_MONITORING_CONTEXT_H__
#define __ROBOT_MONITORING_CONTEXT_H__

#include <yaml-cpp/yaml.h>

namespace XBot { namespace Ui {

class Context
{

public:

    typedef std::shared_ptr<Context> Ptr;

    Context();

    YAML::Node config();
    void saveConfig(YAML::Node);

    ~Context();

private:

    class Impl;
    std::unique_ptr<Impl> impl;

};

}}

#endif
