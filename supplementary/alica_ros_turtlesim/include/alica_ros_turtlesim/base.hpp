#ifndef IO_TURTLE_BASE_BASE_HPP
#define IO_TURTLE_BASE_BASE_HPP

#include "world_model.hpp"

namespace turtlesim
{
class AlicaContext;
class Base
{
public:
    Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const std::string& roleset, const std::string& master_plan,
            const std::string& path);
    ~Base();
    void start();

private:
    alica::AlicaContext* ac;
};

} // namespace turtlesim

#endif /* IO_TURTLE_BASE_BASE_HPP */