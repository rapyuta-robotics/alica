#pragma once

#include "engine/logging/AlicaLogger.h"
namespace alica
{
namespace Logging
{
class LogEntry
{
public:
    LogEntry(Verbosity v);
    LogEntry(LogEntry&& entry);
    ~LogEntry();

    template <typename T>
    LogEntry& operator<<(const T& x)
    {
        _ss << x;
        return *this;
    }

private:
    std::stringstream _ss;
    Verbosity _v;
};

class LoggingUtil
{
public:
    template <class... Args>
    static void log(Verbosity verbosity, Args&&... args)
    {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        AlicaLogger::instance()->log(oss.str(), verbosity);
    }

    static LogEntry logDebug();
    static LogEntry logInfo();
    static LogEntry logWarn();
    static LogEntry logError();
    static LogEntry logFatal();

    static bool isInitialized();
};
} // namespace Logging
} // namespace alica