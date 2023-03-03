#include "engine/blackboard/BlackboardUtil.h"

#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/KeyMapping.h"
#include "engine/logging/Logging.h"

#include <iostream>

namespace alica
{
void BlackboardUtil::setInput(const Blackboard* parent_bb, Blackboard* child_bb, const KeyMapping* keyMapping)
{
    const LockedBlackboardRO lockedParentBb(*parent_bb);
    auto& childBb = child_bb->_impl; // Child not started yet, no other user exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getInputMapping()) {
        try {
            childBb.map(parentKey, childKey, parent_bb->_impl);
            Logging::logDebug("BlackboardUtil") << "passing " << parentKey << " into " << childKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what();
            throw;
        }
    }
    for (const auto& [value, childKey] : keyMapping->getInputValueMapping()) {
        try {
            childBb.mapValue(childKey, value);
            Logging::logDebug("BlackboardUtil") << "passing value: " << value << " into " << childKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing value: " << value << " into " << childKey << ". " << e.what();
            throw;
        }
    }
}

void BlackboardUtil::setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping)
{
    LockedBlackboardRW lockedParentBb(*parent_bb);
    const auto& childBb = child_bb->_impl; // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getOutputMapping()) {
        try {
            parent_bb->_impl.map(childKey, parentKey, childBb);
            Logging::logDebug("BlackboardUtil") << "passing " << childKey << " into " << parentKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << childKey << " into " << parentKey << ". " << e.what();
            throw;
        }
    }
}
} // namespace alica
