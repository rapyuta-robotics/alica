#include <engine/logging/AlicaDefaultLogger.h>

#include <iostream>

#define RESET "\033[0m"
#define GREEN "\033[32m"
#define WHITE "\033[37m"
#define YELLOW "\033[31m"
#define RED "\033[33m"

namespace alica
{
AlicaDefaultLogger::AlicaDefaultLogger(Verbosity verbosity, const std::string& localAgentName)
        : _verbosity(verbosity)
        , _localAgentName(localAgentName)
{
    _verbosityMap = {
            {alica::Verbosity::DEBUG, {"DEBUG", GREEN}},
            {alica::Verbosity::INFO, {"INFO", WHITE}},
            {alica::Verbosity::WARNING, {"WARNING", YELLOW}},
            {alica::Verbosity::ERROR, {"ERROR", RED}},
            {alica::Verbosity::FATAL, {"FATAL", RED}},
    };
}

void AlicaDefaultLogger::log(const std::string& msg, Verbosity verbosity)
{
    // dont print log message when logger verbosity is greater than log verbosity.
    if (_verbosity > verbosity) {
        return;
    }

    if (verbosity < alica::Verbosity::WARNING) {
        std::cout << _verbosityMap.at(verbosity).second << "[" << _verbosityMap.at(verbosity).first << "] " << _localAgentName << ": " << msg << RESET
                  << std::endl;
    } else {
        std::cerr << _verbosityMap.at(verbosity).second << "[" << _verbosityMap.at(verbosity).first << "] " << _localAgentName << ": " << msg << RESET
                  << std::endl;
    }
}

} /* namespace alica */
