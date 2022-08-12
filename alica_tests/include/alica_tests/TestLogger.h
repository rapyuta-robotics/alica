#pragma once

#include <engine/IAlicaLogger.h>

#include <iostream>
#include <string>

namespace alicaTests
{

class TestLogger : public alica::IAlicaLogger
{
public:
    TestLogger(alica::Verbosity verbosity, const std::string& localAgentName)
            : _verbosity(verbosity)
            , _localAgentName(localAgentName)
    {
    }

    virtual ~TestLogger() = default;

    void log(const std::string& msg, const alica::Verbosity verbosity, const std::string& logSpace) { logs.emplace_back(verbosity, msg); }

    std::vector<std::pair<alica::Verbosity, std::string>> logs;

private:
    alica::Verbosity _verbosity;
    std::string _localAgentName;
};

} /* namespace alicaTests */
