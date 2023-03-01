#pragma once

#include "engine/Output.h"
#include "engine/logging/AlicaLogger.h"

#include <iostream>

namespace alica
{
class LogEntry
{
public:
    LogEntry(Verbosity v, const std::string& logSpace);
    ~LogEntry();

    template <typename T>
    LogEntry& operator<<(const T& x)
    {
        _ss << x;
        return *this;
    }

private:
    std::stringstream _ss;
    const Verbosity _v;
    const std::string _logSpace;
};

class Logging
{
public:
    template <class... Args>
    static void log(const Verbosity verbosity, const std::string& logSpace, Args&&... args)
    {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        if (!isInitialized()) {
            std::cerr << "Logger is not initialized, log message: [" << logSpace << "] " << oss.str() << std::endl;
            return;
        }
        AlicaLogger::instance()->log(oss.str(), verbosity, logSpace);
    }

    static LogEntry logDebug(const std::string& logSpace = "");
    static LogEntry logInfo(const std::string& logSpace = "");
    static LogEntry logWarn(const std::string& logSpace = "");
    static LogEntry logError(const std::string& logSpace = "");
    static LogEntry logFatal(const std::string& logSpace = "");

    static bool isInitialized();
};
} // namespace alica
