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
    auto& childBb = child_bb->impl(); // Child not started yet, no other user exists, don't use lock
    for (const auto& [parentKey, value, childKey] : keyMapping->getInputMapping()) {
        if (parentKey.empty()) {
            // parentKey does not exist, setting constant value for child key
            try {
                childBb.setConstValue(childKey, value);
                Logging::logDebug("BlackboardUtil") << "passing const value " << value << " into " << childKey;
            } catch (std::exception& e) {
                Logging::logError("BlackboardUtil") << "Blackboard error passing const value " << value << " into " << childKey << ". " << e.what();
            }
        } else {
            // parent key exists, mapping from parentKey to childKey
            try {
                childBb.map(parentKey, childKey, parent_bb->impl());
                Logging::logDebug("BlackboardUtil") << "passing " << parentKey << " into " << childKey;
            } catch (std::exception& e) {
                Logging::logError("BlackboardUtil") << "Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what();
                throw;
            }
        }
    }
}

void BlackboardUtil::setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping)
{
    LockedBlackboardRW lockedParentBb(*parent_bb);
    const auto& childBb = child_bb->impl(); // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey, value] : keyMapping->getOutputMapping()) {
        try {
            parent_bb->impl().map(childKey, parentKey, childBb);
            Logging::logDebug("BlackboardUtil") << "passing " << childKey << " into " << parentKey;
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << childKey << " into " << parentKey << ". " << e.what();
            throw;
        }
    }
}
} // namespace alica
