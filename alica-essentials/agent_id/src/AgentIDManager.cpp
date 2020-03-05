#include "essentials/AgentIDManager.h"
#include "essentials/AgentIDFactory.h"
#include <cassert>

namespace essentials
{

/**
 * Attention: The idFactory will be deleted by the AgentIDManager's destructor.
 */
AgentIDManager::AgentIDManager(AgentIDFactory* idFactory)
    : _idFactory(idFactory)
{
}

AgentIDManager::~AgentIDManager()
{
    delete _idFactory;
    for (auto& id : _agentIds) {
        delete id;
    }
}

const AgentID* AgentIDManager::generateID(std::size_t size)
{
    const AgentID* tmpID = _idFactory->generateID(size);
    std::lock_guard<std::mutex> guard(_mtx);
    auto entry = _agentIds.insert(tmpID);
    assert(entry.second);
    return tmpID;
}

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding IAgentID object.
 */
const AgentID* AgentIDManager::getIDFromBytes(const std::vector<uint8_t>& idByteVector)
{
    if (idByteVector.empty()) { // empty values result in none-id
        return nullptr;
    }
    // create tmpID for lookup the ID
    const AgentID* tmpID = _idFactory->create(idByteVector);

    // make the manager thread-safe
    std::lock_guard<std::mutex> guard(_mtx);

    // lookup the ID and insert it, if not available, yet
    auto entry = _agentIds.insert(tmpID);
    if (!entry.second) { // delete tmpID if already present in _agentIds
        delete tmpID;
    }
    return *(entry.first);
}

} // namespace essentials
