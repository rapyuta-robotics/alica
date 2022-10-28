#include "engine/blackboard/BlackboardUtil.h"

#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/KeyMapping.h"
#include "engine/logging/Logging.h"

#include <iostream>

namespace alica
{
void BlackboardUtil::setInput(const Blackboard* parent_bb, Blackboard* child_bb, const KeyMapping* keyMapping)
{
    const auto lockedParentBb = LockedBlackboardRO(*parent_bb);
    auto& childBb = child_bb->impl(); // Child not started yet, no other user exists, dont' use lock
    for (const auto& [parentKey, childKey] : keyMapping->getInputMapping()) {
        try {
            childBb.set(childKey, lockedParentBb.get(parentKey));
            Logging::logDebug("BlackboardUtil") << "passing " << parentKey << " into " << childKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what();
        }
    }
}

void BlackboardUtil::setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping)
{
    auto lockedParentBb = LockedBlackboardRW(*parent_bb);
    const auto& childBb = child_bb->impl(); // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getOutputMapping()) {
        try {
            lockedParentBb.set(parentKey, childBb.get(childKey));
            Logging::logDebug("BlackboardUtil") << "passing " << childKey << " into " << parentKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << childKey << " into " << parentKey << ". " << e.what();
        }
    }
}
} // namespace alica
