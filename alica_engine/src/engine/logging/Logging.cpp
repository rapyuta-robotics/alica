#include "engine/logging/Logging.h"

namespace alica
{
bool Logging::isInitialized()
{
    return AlicaLogger::isInitialized();
}

LogEntry Logging::logDebug(const std::string& logSpace)
{
    return LogEntry(Verbosity::DEBUG, logSpace);
}

LogEntry Logging::logInfo(const std::string& logSpace)
{
    return LogEntry(Verbosity::INFO, logSpace);
}

LogEntry Logging::logWarn(const std::string& logSpace)
{
    return LogEntry(Verbosity::WARNING, logSpace);
}

LogEntry Logging::logError(const std::string& logSpace)
{
    return LogEntry(Verbosity::ERROR, logSpace);
}

LogEntry Logging::logFatal(const std::string& logSpace)
{
    return LogEntry(Verbosity::FATAL, logSpace);
}

LogEntry::LogEntry(const Verbosity v, const std::string& logSpace)
        : _v(v)
        , _logSpace(logSpace)
{
    _ss = std::stringstream();
}

LogEntry::~LogEntry()
{
    Logging::log(_v, _logSpace, _ss.str());
}
} // namespace alica