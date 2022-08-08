#include <logger/AlicaRosLogger.h>

#include <iostream>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sstream>

using namespace alica;

namespace alicaRosLogger
{
AlicaRosLogger::AlicaRosLogger(Verbosity verbosity, std::string localAgentName, int64_t localAgentId)
        : _localAgentName(localAgentName)
        , _localAgentId(localAgentId)
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

    std::stringstream streamName;
    streamName << ROSCONSOLE_DEFAULT_NAME << ".[ALICA] "
               << "[" << _localAgentId << "/" << _localAgentName << "]";
    if (ros::console::set_logger_level(streamName.str(), level)) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

void AlicaRosLogger::log(const std::string& msg, Verbosity verbosity)
{
    std::stringstream streamName;
    streamName << "[ALICA] "
               << "[" << _localAgentId << "/" << _localAgentName << "]";
    if (verbosity == alica::Verbosity::DEBUG) {
        ROS_DEBUG_STREAM_NAMED(streamName.str(), msg);
    } else if (verbosity == alica::Verbosity::INFO) {
        ROS_INFO_STREAM_NAMED(streamName.str(), msg);
    } else if (verbosity == alica::Verbosity::WARNING) {
        ROS_WARN_STREAM_NAMED(streamName.str(), msg);
    } else if (verbosity == alica::Verbosity::ERROR) {
        ROS_ERROR_STREAM_NAMED(streamName.str(), msg);
    } else if (verbosity == alica::Verbosity::FATAL) {
        ROS_FATAL_STREAM_NAMED(streamName.str(), msg);
    }
}

} // namespace alicaRosLogger
