#pragma once

#include "engine/logging/AlicaLogger.h"
namespace alica
{
namespace Logging
{
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

    static bool isInitialized();
};
} // namespace Logging
} // namespace alica