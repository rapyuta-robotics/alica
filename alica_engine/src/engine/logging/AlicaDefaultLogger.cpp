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

    if (verbosity < alica::Verbosity::WARNING) {
        std::cout << _verbosityStringAndColorMap.at(verbosity).second << "[" << _verbosityStringAndColorMap.at(verbosity).first << "][" << _localAgentName
                  << "]";
        if (!logSpace.empty()) {
            std::cout << "[" << logSpace << "]";
        }
        std::cout << ": " << msg << RESET << std::endl;
    } else {
        std::cerr << _verbosityStringAndColorMap.at(verbosity).second << "[" << _verbosityStringAndColorMap.at(verbosity).first << "][" << _localAgentName
                  << "]";
        if (!logSpace.empty()) {
            std::cerr << "[" << logSpace << "]";
        }
        std::cerr << ": " << msg << RESET << std::endl;
    }
}

} /* namespace alica */
