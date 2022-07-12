#pragma once

#include <iostream>
#include <sstream>
#include <string>

namespace alica
{
// IAlicaLogger has nothing to do with engine/Logger.h

enum class Verbosity
{
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

class IAlicaLogger
{
public:
    // IAlicaLogger() = default;
    virtual ~IAlicaLogger() = default;

    template <class... Args>
    void log(Verbosity verbosity, Args&&... args)
    {
        // parse args
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));

        log(oss.str(), verbosity);
    }

    virtual void log(const std::string& msg, Verbosity verbosity) = 0;
};

} // namespace alica
