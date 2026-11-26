#pragma once

#include <engine/AlicaContext.h>

#include <ros/callback_queue.h>
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
    /**
     * dedicatedEngineThread: true runs the ALICA engine on a thread of its own. False leaves it to the caller's loop,
     * which must call spinEngineOnce() regularly.
     */
    Base(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& name, const int agent_id, const std::string& roleset, const std::string& master_plan,
            const std::vector<std::string>& paths, std::optional<std::string> placeholderMapping = std::nullopt, bool dedicatedEngineThread = true);
    ~Base();
    void start();
    // Runs the engine timer callbacks that are due. Only needed without a dedicated engine thread.
    void spinEngineOnce();

private:
    // Declared before ac, so that the engine timer is gone before its queue and spinner are destroyed
    ros::AsyncSpinner spinner;
    ros::CallbackQueue engine_queue;
    std::optional<ros::AsyncSpinner> engine_spinner;
    std::unique_ptr<alica::AlicaContext> ac;
    const bool _dedicatedEngineThread;
};

} // namespace turtlesim
