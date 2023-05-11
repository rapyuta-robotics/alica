#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace alica
{

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
