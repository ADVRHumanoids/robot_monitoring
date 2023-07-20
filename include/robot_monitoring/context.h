#ifndef __ROBOT_MONITORING_CONTEXT_H__
#define __ROBOT_MONITORING_CONTEXT_H__

#include <yaml-cpp/yaml.h>

namespace XBot { namespace Ui {

class Context
{

public:

    typedef std::shared_ptr<Context> Ptr;

    Context();

    /**
     * @brief config returns an handle to the global
     * gui config
     */
    YAML::Node& config();

    /**
     * @brief saveConfig dumps the provided config node to
     * file, so that is is persistently saved
     */
    void saveConfig(YAML::Node);

    ~Context();

private:

    class Impl;
    std::unique_ptr<Impl> impl;

};

}}

#endif
