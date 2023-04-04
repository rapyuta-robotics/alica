#pragma once

#include <engine/Types.h>
#include <engine/logging/IAlicaLogger.h>

#include <rclcpp/logger.hpp>

#include <string>

using namespace alica;

namespace alicaRosLogger
{

class AlicaRosLogger : public alica::IAlicaLogger
{
public:
    AlicaRosLogger(const Verbosity verbosity, const std::string& localAgentName);
    [[deprecated]] AlicaRosLogger(const Verbosity verbosity, const std::string& localAgentName, [[maybe_unused]] const alica::AgentId localAgentId)
            : AlicaRosLogger(verbosity, localAgentName)
    {
    }
    void log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace) override;

private:
    const std::string _localAgentName;
    const alica::AgentId _localAgentId;
    static const std::unordered_map<alica::Verbosity, rclcpp::Logger::Level> _verbosityRosLevelMap;
};

} // namespace alicaRosLogger
