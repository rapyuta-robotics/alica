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
    for (const Mapping& mapping : keyMapping->getInputMapping()) {
        if (mapping.parentKeyOrConstant.index() == 1) {
            // parentKey does not exist, setting constant value for child key
            try {
                childBb.setConstValue(mapping.childKey, std::get<std::string>(mapping.parentKeyOrConstant));
                Logging::logDebug("BlackboardUtil") << "passing const value " << std::get<std::string>(mapping.parentKeyOrConstant) << " into "
                                                    << mapping.childKey;
            } catch (std::exception& e) {
                Logging::logError("BlackboardUtil") << "Blackboard error passing const value " << std::get<std::string>(mapping.parentKeyOrConstant) << " into "
                                                    << mapping.childKey << ". " << e.what();
                throw;
            }
        } else {
            // parent key exists, mapping from parentKey to childKey
            try {
                childBb.map(std::get<std::string>(mapping.parentKeyOrConstant), mapping.childKey, parent_bb->impl());
                Logging::logDebug("BlackboardUtil") << "passing " << std::get<std::string>(mapping.parentKeyOrConstant) << " into " << mapping.childKey;
            } catch (std::exception& e) {
                Logging::logError("BlackboardUtil") << "Blackboard error passing " << std::get<std::string>(mapping.parentKeyOrConstant) << " into "
                                                    << mapping.childKey << ". " << e.what();
                throw;
            }
        }
    }
}

void BlackboardUtil::setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping)
{
    LockedBlackboardRW lockedParentBb(*parent_bb);
    const auto& childBb = child_bb->impl(); // Child is terminated, no other users exists, don't use lock
    for (const Mapping& mapping : keyMapping->getOutputMapping()) {
        try {
            parent_bb->impl().map(mapping.childKey, std::get<std::string>(mapping.parentKeyOrConstant), childBb);
            Logging::logDebug("BlackboardUtil") << "passing " << mapping.childKey << " into " << std::get<std::string>(mapping.parentKeyOrConstant);
        } catch (std::exception& e) {
            Logging::logError("BlackboardUtil") << "Blackboard error passing " << mapping.childKey << " into "
                                                << std::get<std::string>(mapping.parentKeyOrConstant) << ". " << e.what();
            throw;
        }
    }
}
} // namespace alica
