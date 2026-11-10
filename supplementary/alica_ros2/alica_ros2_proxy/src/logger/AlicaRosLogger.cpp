#include "logger/AlicaRosLogger.h"

#include <rclcpp/logging.hpp>

using namespace alica;

namespace alicaRosLogger
{
AlicaRosLogger::AlicaRosLogger(const Verbosity verbosity, const std::string& localAgentName)
        : _localAgentName(localAgentName)
        , _logger(rclcpp::get_logger(localAgentName))
{
    _logger.set_level(_verbosityRosLevelMap.at(verbosity));
    RCLCPP_INFO(_logger, "Instantiated ROS logging for ALICA");
}

void AlicaRosLogger::log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace)
{
    if (verbosity == alica::Verbosity::DEBUG) {
        RCLCPP_DEBUG_STREAM(_logger, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::INFO) {
        RCLCPP_INFO_STREAM(_logger, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::WARNING) {
        RCLCPP_WARN_STREAM(_logger, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::ERROR) {
        RCLCPP_ERROR_STREAM(_logger, "[" << logSpace << "] " << msg);
    } else if (verbosity == alica::Verbosity::FATAL) {
        RCLCPP_FATAL_STREAM(_logger, "[" << logSpace << "] " << msg);
    }
}

const std::unordered_map<alica::Verbosity, rclcpp::Logger::Level> AlicaRosLogger::_verbosityRosLevelMap = {
        {alica::Verbosity::DEBUG, rclcpp::Logger::Level::Debug}, {alica::Verbosity::INFO, rclcpp::Logger::Level::Info},
        {alica::Verbosity::WARNING, rclcpp::Logger::Level::Warn}, {alica::Verbosity::ERROR, rclcpp::Logger::Level::Error},
        {alica::Verbosity::FATAL, rclcpp::Logger::Level::Fatal}};

} // namespace alicaRosLogger
