#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

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
    virtual ~IAlicaLogger() = default;

    virtual void log(const std::string& msg, const Verbosity verbosity, const std::string& logSpace = "") = 0;
};

} // namespace alica
