#pragma once
#include "engine/logging/IAlicaLogger.h"

#include <memory>

namespace alica
{
class AlicaLogger
{
public:
    template <class LoggerType, class... Args>
    static void set(Args&&... args)
    {
        static_assert(std::is_base_of_v<IAlicaLogger, LoggerType>, "LoggerType needs to inherit from IAlicaLogger");
        _logger = std::make_unique<LoggerType>(std::forward<decltype(args)>(args)...);
    }

    /**
     * Install a logger only if none is installed yet.
     *
     * Unlike set(), this never replaces a logger that is already in place. It exists so that
     * constructing an AlicaContext cannot silently take the process-wide logger away from a
     * context that is already running - see AlicaContext::setLogger.
     *
     * @return True if this call installed the logger, false if one was already present.
     */
    template <class LoggerType, class... Args>
    static bool setIfUnset(Args&&... args)
    {
        static_assert(std::is_base_of_v<IAlicaLogger, LoggerType>, "LoggerType needs to inherit from IAlicaLogger");
        if (_logger) {
            return false;
        }
        _logger = std::make_unique<LoggerType>(std::forward<decltype(args)>(args)...);
        return true;
    }

    static void destroy();
    static IAlicaLogger* instance();
    static bool isInitialized();

private:
    static std::unique_ptr<IAlicaLogger> _logger;
};
} // namespace alica
