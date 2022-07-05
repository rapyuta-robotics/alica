#pragma once

#include <engine/IAlicaLogger.h>

#include <iostream>
#include <string>

namespace alica
{

class AlicaDefaultLogger : public alica::IAlicaLogger
{
public:
    AlicaDefaultLogger(Verbosity verbosity)
            : _verbosity(verbosity)
    {
    }

    virtual ~AlicaDefaultLogger() = default;

    void log(const std::string& msg, Verbosity verbosity)
    {
        // dont print log message when logger verbosity is greater than log verbosity.
        if (_verbosity > verbosity) {
            return;
        }

        if (verbosity == alica::Verbosity::DEBUG || verbosity == alica::Verbosity::INFO) {
            std::cout << msg << std::endl;
        } else if (verbosity == alica::Verbosity::WARNING || verbosity == alica::Verbosity::ERROR || verbosity == alica::Verbosity::FATAL) {
            std::cerr << msg << std::endl;
        }
    }

private:
    Verbosity _verbosity;
};

} /* namespace alica */
