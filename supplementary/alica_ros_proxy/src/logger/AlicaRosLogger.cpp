#include <logger/AlicaRosLogger.h>

#include <iostream>
#include <ros/callback_queue.h>
#include <sstream>

using namespace alica;

namespace alicaRosLogger
{
AlicaRosLogger::AlicaRosLogger(const Verbosity verbosity, const std::string& localAgentName, const alica::AgentId localAgentId)
        : _localAgentName(localAgentName)
        , _localAgentId(localAgentId)
{
    ros::console::Level level = _verbosityRosLevelMap.at(verbosity);

    std::stringstream streamName;
    streamName << ROSCONSOLE_DEFAULT_NAME << ".[ALICA] "
               << "[" << _localAgentId << "/" << _localAgentName << "]";
    if (ros::console::set_logger_level(streamName.str(), level)) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

void AlicaRosLogger::log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace)
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

const std::unordered_map<alica::Verbosity, ros::console::Level> AlicaRosLogger::_verbosityRosLevelMap = {{alica::Verbosity::DEBUG, ros::console::levels::Debug},
        {alica::Verbosity::INFO, ros::console::levels::Info}, {alica::Verbosity::WARNING, ros::console::levels::Warn},
        {alica::Verbosity::ERROR, ros::console::levels::Error}, {alica::Verbosity::FATAL, ros::console::levels::Fatal}};

} // namespace alicaRosLogger
