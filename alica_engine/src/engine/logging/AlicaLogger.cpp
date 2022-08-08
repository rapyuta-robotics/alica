#include "engine/logging/AlicaLogger.h"
#include "engine/logging/IAlicaLogger.h"

#include <cassert>
#include <type_traits>
#include <utility>

namespace alica
{
IAlicaLogger* AlicaLogger::instance()
{
    assert(_logger);
    return _logger.get();
}

void AlicaLogger::destroy()
{
    if (!_logger) {
        return;
    }
    _logger.reset();
}

bool AlicaLogger::isInitialized()
{
    return _logger != nullptr;
}

std::unique_ptr<IAlicaLogger> AlicaLogger::_logger;
} // namespace alica
