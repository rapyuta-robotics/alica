#pragma once

#include <engine/logging/IAlicaLogger.h>

#include <string>

namespace alica
{

class AlicaDefaultLogger : public alica::IAlicaLogger
{
public:
    AlicaDefaultLogger(Verbosity verbosity, const std::string& localAgentName);

    void log(const std::string& msg, Verbosity verbosity) override;

private:
    Verbosity _verbosity;
    std::string _localAgentName;
    std::unordered_map<alica::Verbosity, std::pair<std::string, std::string>> _verbosityMap;
};

} /* namespace alica */
