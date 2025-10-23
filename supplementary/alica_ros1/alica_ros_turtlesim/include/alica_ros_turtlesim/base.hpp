#pragma once

#include <engine/AlicaContext.h>
#include <engine/IAlicaTimer.h>

#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ros/ros.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace turtlesim
{
class AlicaContext;
class Base
{
public:
    Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
            const std::vector<std::string>& paths, std::optional<std::string> placeholderMapping = std::nullopt, bool drivenExecutor = false);
    ~Base();
    void start();
    void tick();

private:
    ros::AsyncSpinner spinner;
    std::unique_ptr<alica::AlicaContext> ac;
    std::shared_ptr<alica::IAlicaTimerFactory> timer_factory;
    std::optional<ros::CallbackQueue> callback_queue;
    bool _drivenExecutor = false;
};

} // namespace turtlesim
