#pragma once

#include <engine/IAlicaLogger.h>

#include <iostream>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>

using namespace alica;

namespace alicaRosLogger
{

class AlicaRosLogger : public alica::IAlicaLogger
{
public:
    AlicaRosLogger(Verbosity verbosity)
    {
        ros::console::Level level;
        if (verbosity == alica::Verbosity::DEBUG) {
            level = ros::console::levels::Debug;
        } else if (verbosity == alica::Verbosity::INFO) {
            level = ros::console::levels::Info;
        } else if (verbosity == alica::Verbosity::WARNING) {
            level = ros::console::levels::Warn;
        } else if (verbosity == alica::Verbosity::ERROR) {
            level = ros::console::levels::Error;
        } else if (verbosity == alica::Verbosity::FATAL) {
            level = ros::console::levels::Fatal;
        }

        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level)) {
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    virtual ~AlicaRosLogger() = default;

    template <class... Args>
    void log(std::string& msg, Verbosity verbosity)
    {
        if (verbosity == alica::Verbosity::DEBUG) {
            ROS_DEBUG("%s", msg.c_str());
        } else if (verbosity == alica::Verbosity::INFO) {
            ROS_INFO("%s", msg.c_str());
        } else if (verbosity == alica::Verbosity::WARNING) {
            ROS_WARN("%s", msg.c_str());
        } else if (verbosity == alica::Verbosity::ERROR) {
            ROS_ERROR("%s", msg.c_str());
        } else if (verbosity == alica::Verbosity::FATAL) {
            ROS_FATAL("%s", msg.c_str());
        }
    }
};

} // namespace alicaRosLogger
