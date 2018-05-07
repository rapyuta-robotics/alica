#include "supplementary/AgentIDManager.h"
#include "supplementary/AgentIDFactory.h"
namespace supplementary {

/**
 * Attention: The idFactory will be deleted by the AgentIDManager's destructor.
 */
AgentIDManager::AgentIDManager(AgentIDFactory* idFactory)
        : idFactory(idFactory) {}

AgentIDManager::~AgentIDManager() {
    delete this->idFactory;
    for (auto& id : this->agentIDs) {
        delete id;
    }
}

const AgentID* AgentIDManager::generateID(int size) {
    return this->idFactory->generateID(size);
}

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding IAgentID object.
 */
const AgentID* AgentIDManager::getIDFromBytes(const std::vector<uint8_t>& idByteVector) {
    // create tmpID for lookup the ID
    const AgentID* tmpID = this->idFactory->create(idByteVector);

    // make the manager thread-safe
    std::lock_guard<std::mutex> guard(mutex);

    // lookup the ID and insert it, if not available, yet
    auto entry = this->agentIDs.insert(tmpID);
    if (!entry.second) {  // delete tmpID if already present in agentIDs
        delete tmpID;
    }
    return *(entry.first);
}
}  // namespace supplementary
