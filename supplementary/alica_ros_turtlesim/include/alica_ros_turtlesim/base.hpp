#ifndef IO_TURTLE_BASE_BASE_HPP
#define IO_TURTLE_BASE_BASE_HPP

#include "world_model.hpp"
#include <engine/AlicaEngine.h>

namespace turtlesim
{
class AlicaContext;
class Base
{
public:
    Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
            const std::string& path, bool dynamic);
    ~Base();
    void start();

private:
    ros::AsyncSpinner spinner;
    alica::AlicaContext* ac;
    void ALICATurtleWorldModelCallInit(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

    void ALICATurtleWorldModelCallDel();

    bool _loadDynamically{false};
};

} // namespace turtlesim

#endif /* IO_TURTLE_BASE_BASE_HPP */
