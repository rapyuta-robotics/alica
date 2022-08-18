#pragma once
#include "engine/logging/IAlicaLogger.h"

#include <memory>

namespace alica
{
class AlicaLogger
{
public:
    template <class LoggerType, class... Args>
    static void create(Args&&... args)
    {
        static_assert(std::is_base_of_v<IAlicaLogger, LoggerType>, "LoggerType needs to inherit from IAlicaLogger");
        if (_logger) {
            _logger->log("Logger has already been created!", alica::Verbosity::WARNING);
            return;
        }

#if (defined __cplusplus && __cplusplus >= 201402L)
        _logger = std::make_unique<LoggerType>(std::forward<decltype(args)>(args)...);
#else
        _logger = std::unique_ptr<LoggerType>(new LoggerType(verbosity, _localAgentName, std::forward<Args>(args)...));
#endif
    }

    static IAlicaLogger* instance();
    static bool isInitialized();

private:
    static std::unique_ptr<IAlicaLogger> _logger;
};
} // namespace alica
