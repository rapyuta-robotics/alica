#pragma once

#include "supplementary/IAgentIDFactory.h"
#include "supplementary/IAgentID.h"

#include <mutex>
#include <unordered_set>
#include <vector>

namespace supplementary
{

class AgentIDManager
{
  public:
    // static AgentIDManager *getInstance();
    AgentIDManager(IAgentIDFactory* idFactory);
    virtual ~AgentIDManager();

    const IAgentID *getIDFromBytes(const std::vector<uint8_t> &vectorID);

    template <class Prototype>
    const IAgentID *getID(Prototype &idPrototype);

  private:
    std::unordered_set<const IAgentID *, supplementary::IAgentIDHash, supplementary::IAgentIDEqualsComparator> agentIDs;
    IAgentIDFactory *idFactory;
    std::mutex mutex;
};

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing an int and receiving
 * a pointer to a corresponding IAgentID object (in that case an
 * IntRobotID).
 */
template <class Prototype>
const IAgentID *AgentIDManager::getID(Prototype &idPrototype)
{
    // little-endian encoding
    std::vector<uint8_t> idByteVector;
    for (int i = 0; i < sizeof(Prototype); i++)
    {
        idByteVector.push_back(*(((uint8_t *)&idPrototype) + i));
    }
    return this->getIDFromBytes(idByteVector);
}

} /* namespace supplementary */
