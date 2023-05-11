#include <engine/logging/AlicaDefaultLogger.h>

#include <iostream>

namespace alica
{
AlicaDefaultLogger::AlicaDefaultLogger(const Verbosity verbosity, const std::string& localAgentName)
        : _verbosity(verbosity)
        , _localAgentName(localAgentName)
{
}

const std::unordered_map<alica::Verbosity, std::pair<const char*, const char*>> AlicaDefaultLogger::_verbosityStringAndColorMap = {
        {alica::Verbosity::DEBUG, {"DEBUG", GREEN}}, {alica::Verbosity::INFO, {"INFO", WHITE}}, {alica::Verbosity::WARNING, {"WARNING", YELLOW}},
        {alica::Verbosity::ERROR, {"ERROR", RED}}, {alica::Verbosity::FATAL, {"FATAL", RED}}};

void AlicaDefaultLogger::log(const std::string& msg, Verbosity verbosity, const std::string& logSpace)
{
    // dont print log message when logger verbosity is greater than log verbosity.
    if (_verbosity > verbosity) {
        return;
    }

    std::ostringstream oss;
    if (verbosity < alica::Verbosity::WARNING) {
        oss << _verbosityStringAndColorMap.at(verbosity).second << "[" << _verbosityStringAndColorMap.at(verbosity).first << "][" << _localAgentName << "]";
        if (!logSpace.empty()) {
            oss << "[" << logSpace << "]";
        }
        oss << ": " << msg << RESET << std::endl;
        std::cout << oss.str();
    } else {
        oss << _verbosityStringAndColorMap.at(verbosity).second << "[" << _verbosityStringAndColorMap.at(verbosity).first << "][" << _localAgentName << "]";
        if (!logSpace.empty()) {
            oss << "[" << logSpace << "]";
        }
        oss << ": " << msg << RESET << std::endl;
        std::cerr << oss.str();
    }
}

} /* namespace alica */
