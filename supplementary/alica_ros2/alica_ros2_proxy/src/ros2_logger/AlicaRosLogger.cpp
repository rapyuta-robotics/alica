#include "ros2_logger/AlicaRosLogger.h"
#include <iostream>
#include <rclcpp/logging.hpp>
#include <sstream>

using namespace alica;

namespace alicaRosLogger
{
AlicaRosLogger::AlicaRosLogger(const Verbosity verbosity, const std::string& localAgentName, const alica::AgentId localAgentId)
        : _localAgentName(localAgentName)
        , _localAgentId(localAgentId)
{
}

void AlicaRosLogger::log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace)
{
    std::stringstream streamName;
    streamName << "[ALICA] "
               << "[" << _localAgentId << "/" << _localAgentName << "]";
    if (verbosity == alica::Verbosity::DEBUG) {
        RCLCPP_DEBUG(rclcpp::get_logger(streamName.str()), msg.c_str());
    } else if (verbosity == alica::Verbosity::INFO) {
        RCLCPP_INFO(rclcpp::get_logger(streamName.str()), msg.c_str());
    } else if (verbosity == alica::Verbosity::WARNING) {
        RCLCPP_WARN(rclcpp::get_logger(streamName.str()), msg.c_str());
    } else if (verbosity == alica::Verbosity::ERROR) {
        RCLCPP_ERROR(rclcpp::get_logger(streamName.str()), msg.c_str());
    } else if (verbosity == alica::Verbosity::FATAL) {
        RCLCPP_FATAL(rclcpp::get_logger(streamName.str()), msg.c_str());
    }
}

const std::unordered_map<alica::Verbosity, rclcpp::Logger::Level> AlicaRosLogger::_verbosityRosLevelMap = {
        {alica::Verbosity::DEBUG, rclcpp::Logger::Level::Debug}, {alica::Verbosity::INFO, rclcpp::Logger::Level::Info},
        {alica::Verbosity::WARNING, rclcpp::Logger::Level::Warn}, {alica::Verbosity::ERROR, rclcpp::Logger::Level::Error},
        {alica::Verbosity::FATAL, rclcpp::Logger::Level::Fatal}};

} // namespace alicaRosLogger
