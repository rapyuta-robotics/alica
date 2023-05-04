#include "engine/logging/AlicaLogger.h"

#include <cassert>
#include <type_traits>
#include <utility>

namespace alica
{
class IAlicaLogger;

void AlicaLogger::destroy()
{
    _logger.reset();
}

IAlicaLogger* AlicaLogger::instance()
{
    assert(_logger);
    return _logger.get();
}

bool AlicaLogger::isInitialized()
{
    return _logger != nullptr;
}

std::unique_ptr<IAlicaLogger> AlicaLogger::_logger;
} // namespace alica
