#pragma once

#include <engine/logging/IAlicaLogger.h>

#include <string>

namespace alica
{

class AlicaDefaultLogger : public alica::IAlicaLogger
{
public:
    AlicaDefaultLogger(const Verbosity verbosity, const std::string& localAgentName);
    void log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace) override;

private:
    static constexpr const char* RESET = "\033[0m";
    static constexpr const char* GREEN = "\033[32m";
    static constexpr const char* WHITE = "\033[37m";
    static constexpr const char* YELLOW = "\033[31m";
    static constexpr const char* RED = "\033[33m";

    const Verbosity _verbosity;
    const std::string _localAgentName;
    // maps the verbosity level to the corresponding string representation and the color used for printing logs.
    static const std::unordered_map<alica::Verbosity, std::pair<const char*, const char*>> _verbosityStringAndColorMap;
};

} /* namespace alica */
