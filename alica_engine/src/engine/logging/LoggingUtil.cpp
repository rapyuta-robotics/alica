#include "engine/logging/LoggingUtil.h"

namespace alica
{
namespace Logging
{
bool LoggingUtil::isInitialized()
{
    return AlicaLogger::isInitialized();
}

LogEntry LoggingUtil::logDebug()
    {
        return LogEntry(Verbosity::DEBUG);
    }

     LogEntry LoggingUtil::logInfo()
    {
        return LogEntry(Verbosity::INFO);
    }

     LogEntry LoggingUtil::logWarn()
    {
        return LogEntry(Verbosity::WARNING);
    }

     LogEntry LoggingUtil::logError()
    {
        return LogEntry(Verbosity::ERROR);
    }

     LogEntry LoggingUtil::logFatal()
    {
        return LogEntry(Verbosity::FATAL);
    }

LogEntry::LogEntry(Verbosity v)
    {
        _ss = std::stringstream();
        _v = v;
    }

    LogEntry::LogEntry(LogEntry&& entry)
    {
        _ss << entry._ss.str();
        _v = entry._v;
    }

    LogEntry::~LogEntry()
    {
        LoggingUtil::log(_v, _ss.str());
    }
} // namespace Logging
} // namespace alica