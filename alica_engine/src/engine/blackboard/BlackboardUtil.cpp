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
            // Converter::setValue(childKey, lockedParentBb.get(parentKey), childBb.getBlackboardValueType(childKey), childBb);
            childBb.map(parentKey, childKey, parent_bb->impl());
            Logging::logDebug("BlackboardUtil") << "passing " << parentKey << " into " << childKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what();
        }
    }
}

void BlackboardUtil::setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping)
{
    auto& parentBb = parent_bb->impl();
    const auto& childBb = child_bb->impl(); // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getOutputMapping()) {
        try {
            parent_bb->impl().map(childKey, parentKey, childBb);
            // Converter::setValue(parentKey, childBb.get(childKey), parentBb.getBlackboardValueType(parentKey), parentBb);
            Logging::logDebug("BlackboardUtil") << "passing " << childKey << " into " << parentKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << childKey << " into " << parentKey << ". " << e.what();
        }
    }
}
} // namespace alica