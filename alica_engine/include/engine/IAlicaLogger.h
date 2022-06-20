#pragma once

#include <string>

namespace alica
{
// IAlicaLogger has nothing to do with engine/Logger.h
class IAlicaLogger
{
public:
    
    virtual ~IAlicaLogger() = default;

    /**
     * Alternative: 
     * A single method that has the verbosity level as an argument.
     * virtual void log(const std::string& msg, Verbosity verbosity=Verbosity.DEBUG)
     **/
    virtual void debug(const std::string& msg) = 0;
    virtual void info(const std::string& msg) = 0;
    virtual void warning(const std::string& msg) = 0;
    virtual void error(const std::string& msg) = 0;
    virtual void fatal(const std::string& msg) = 0;
};

} // namespace alica
