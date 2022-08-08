#include "engine/logging/LoggingUtil.h"
namespace alica
{
namespace Logging
{
bool LoggingUtil::isInitialized()
{
    return AlicaLogger::isInitialized();
}
} // namespace Logging
} // namespace alica