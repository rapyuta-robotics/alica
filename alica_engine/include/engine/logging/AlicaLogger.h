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

    static void destroy();
    static IAlicaLogger* instance();
    static bool isInitialized();

private:
    static std::unique_ptr<IAlicaLogger> _logger;
};
} // namespace alica
