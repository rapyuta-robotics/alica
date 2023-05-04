#include "test_alica.h"

#include <gtest/gtest.h>

namespace alica::test
{
namespace
{

TEST_F(SingleAgentUninitializedTestFixture, noLoggerSet)
{
    // check if logging is initialized even if the context is not yet initialized
    ASSERT_TRUE(AlicaLogger::isInitialized());

    // check if the logger is set to AlicaDefaultLogger
    ASSERT_TRUE(dynamic_cast<AlicaDefaultLogger*>(AlicaLogger::instance()));
}

// A custom logger that stores the last log msg that was logged + the total number of logs. The logger is thread safe which indirectly verifies if a
// non-copyable, non-movable class can be used as a logger
class CustomLogger : public IAlicaLogger
{
public:
    CustomLogger(Verbosity verbosity, const std::string& agentName, const std::string& customParam)
            : _verbosity(verbosity)
            , _agentName(agentName)
            , _customParam(customParam)
            , _numLogs(0)
    {
    }

    bool verifyConstruction(Verbosity verbosity, const std::string& agentName, const std::string& customParam) const
    {
        return _verbosity == verbosity && _agentName == agentName && _customParam == customParam;
    }

    void log(const std::string& msg, Verbosity verbosity, const std::string& logSpace) override
    {
        std::unique_lock<std::mutex> lock(_logMutex);
        _lastLogMsg = msg;
        _lastLogVerbosity = verbosity;
        _lastLogSpace = logSpace;
        ++_numLogs;
    }

    bool verifyLog(const std::string& msg, Verbosity verbosity, const std::string& logSpace) const
    {
        std::unique_lock<std::mutex> lock(_logMutex);
        return msg == _lastLogMsg && verbosity == _lastLogVerbosity && logSpace == _lastLogSpace;
    }

    std::size_t numLogs() const
    {
        std::unique_lock<std::mutex> lock(_logMutex);
        return _numLogs;
    }

private:
    // The constructor parameters
    const Verbosity _verbosity;
    const std::string _agentName;
    const std::string _customParam;

    // Info about the last log
    mutable std::mutex _logMutex;
    std::string _lastLogMsg;
    Verbosity _lastLogVerbosity;
    std::string _lastLogSpace;
    std::size_t _numLogs;
};

TEST_F(SingleAgentUninitializedTestFixture, loggerSet)
{
    // check if a custom logger can be used

    // set a custom logger
    _tc->setLogger<CustomLogger>("custom_param");

    // check if the logger is set to CustomLogger
    ASSERT_TRUE(AlicaLogger::isInitialized());
    auto customLogger = dynamic_cast<CustomLogger*>(AlicaLogger::instance());
    ASSERT_TRUE(customLogger);

    // check if the logger is constructed from the expected parameters
    ASSERT_TRUE(customLogger->verifyConstruction(Verbosity::INFO, agentName(), "custom_param"));

    // Try to log something & see if it works
    Logging::log(Verbosity::ERROR, "log_space_log", "log_msg_log");
    ASSERT_TRUE(customLogger->verifyLog("log_msg_log", Verbosity::ERROR, "log_space_log"));

    // Use various log api's & see if they work
    Logging::logDebug("log_space_debug") << "log_msg_debug";
    ASSERT_TRUE(customLogger->verifyLog("log_msg_debug", Verbosity::DEBUG, "log_space_debug"));
    Logging::logError("log_space_error") << "log_msg_error";
    ASSERT_TRUE(customLogger->verifyLog("log_msg_error", Verbosity::ERROR, "log_space_error"));
    Logging::logFatal("log_space_fatal") << "log_msg_fatal";
    ASSERT_TRUE(customLogger->verifyLog("log_msg_fatal", Verbosity::FATAL, "log_space_fatal"));
    Logging::logInfo("log_space_info") << "log_msg_info";
    ASSERT_TRUE(customLogger->verifyLog("log_msg_info", Verbosity::INFO, "log_space_info"));
    Logging::logWarn("log_space_warn") << "log_msg_warn";
    ASSERT_TRUE(customLogger->verifyLog("log_msg_warn", Verbosity::WARNING, "log_space_warn"));

    // Initialize the engine & check if some logs are added
    auto numLogsBeforeInit = customLogger->numLogs();
    initialize();
    auto numLogsAfterInit = customLogger->numLogs();
    ASSERT_EQ(customLogger, dynamic_cast<CustomLogger*>(AlicaLogger::instance()));
    ASSERT_GE(numLogsAfterInit, numLogsBeforeInit);
}

} // namespace
} // namespace alica::test
