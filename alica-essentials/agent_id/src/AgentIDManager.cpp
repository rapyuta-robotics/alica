#include "supplementary/AgentIDManager.h"
#include "supplementary/IAgentIDFactory.h"
namespace supplementary{

/**
 * The method for getting the singleton instance.
 * @return A pointer to the AgentIDManager object, you must not delete.
 */
//AgentIDManager *AgentIDManager::getInstance()
//{
//    static AgentIDManager instance;
//    return &instance;
//}

/**
 * Attention: The idFactory will be deleted by the AgentIDManager's destructor.
 */
AgentIDManager::AgentIDManager(IAgentIDFactory* idFactory)
    : idFactory(idFactory)
{
}

AgentIDManager::~AgentIDManager()
{
	delete this->idFactory;
	for (auto& id : this->agentIDs)
	{
		delete id;
	}
}

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing a part of a ROS
 * message and receiving a pointer to a corresponding IAgentID object.
 */
const IAgentID *AgentIDManager::getIDFromBytes(const std::vector<uint8_t> &idByteVector)
{
    // create tmpID for lookup the ID
    const IAgentID *tmpID = this->idFactory->create(idByteVector);

    // make the manager thread-safe
    std::lock_guard<std::mutex> guard(mutex);

    // lookup the ID and insert it, if not available, yet
    auto entry = this->agentIDs.insert(tmpID);
    if (!entry.second)
    { // delete tmpID if already present in agentIDs
        delete tmpID;
    }
    return *(entry.first);
}

}
