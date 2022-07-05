#pragma once

#include <engine/IAlicaLogger.h>

#include <iostream>
#include <string>

namespace alicaTests
{

class TestLogger : public alica::IAlicaLogger
{
public:
    TestLogger(alica::Verbosity verbosity)
            : _verbosity(verbosity)
    {
    }

    virtual ~TestLogger() = default;

    void log(const std::string& msg, alica::Verbosity verbosity) { logs.emplace_back(verbosity, msg); }

    std::vector<std::pair<alica::Verbosity, std::string>> logs;

private:
    alica::Verbosity _verbosity;
};

} /* namespace alicaTests */
