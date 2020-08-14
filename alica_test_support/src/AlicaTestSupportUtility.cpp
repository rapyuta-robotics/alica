#include "alica/AlicaTestSupportUtility.h"

#include <engine/AlicaContext.h>

namespace alica
{
// Don't want to expose engine to app developers
alica::AlicaEngine* AlicaTestSupportUtility::getEngine(alica::AlicaContext* ac)
{
    return ac->_engine.get();
}

} // namespace alica
