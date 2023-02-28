#include <logger/AlicaRosLogger.h>

#include <iostream>
#include <ros/callback_queue.h>
#include <sstream>

using namespace alica;

namespace alicaRosLogger
{
AlicaRosLogger::AlicaRosLogger(const Verbosity verbosity, const std::string& localAgentName)
        : _localAgentName(localAgentName)
{
    ros::console::Level level = _verbosityRosLevelMap.at(verbosity);

    if (ros::console::set_logger_level(_localAgentName, level)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_STREAM_NAMED(_localAgentName, "Instantiated ROS logging for ALICA");
}

void AlicaRosLogger::log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace)
{
    if (verbosity == alica::Verbosity::DEBUG) {
        ROS_DEBUG_STREAM_NAMED(_localAgentName, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::INFO) {
        ROS_INFO_STREAM_NAMED(_localAgentName, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::WARNING) {
        ROS_WARN_STREAM_NAMED(_localAgentName, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::ERROR) {
        ROS_ERROR_STREAM_NAMED(_localAgentName, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::FATAL) {
        ROS_FATAL_STREAM_NAMED(_localAgentName, "[" << logSpace << "] " << msg);
    }
}

const std::unordered_map<alica::Verbosity, ros::console::Level> AlicaRosLogger::_verbosityRosLevelMap = {{alica::Verbosity::DEBUG, ros::console::levels::Debug},
        {alica::Verbosity::INFO, ros::console::levels::Info}, {alica::Verbosity::WARNING, ros::console::levels::Warn},
        {alica::Verbosity::ERROR, ros::console::levels::Error}, {alica::Verbosity::FATAL, ros::console::levels::Fatal}};

} // namespace alicaRosLogger
