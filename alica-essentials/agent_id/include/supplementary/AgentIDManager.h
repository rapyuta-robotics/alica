#pragma once

#include "supplementary/IAgentIDFactory.h"

#include <unordered_set>
#include <vector>


namespace supplementary
{
class IAgentIDFactory;
class IAgentID;

class AgentIDManager
{
  public:
    static AgentIDManager *getInstance();

    template <class IDType>
    const IDType *createIDFromBytes(const std::vector<uint8_t> &vectorID);

    template <class IDType, class Prototype>
    const IDType *createID(Prototype &idPrototype);

  private:
    AgentIDManager();
    virtual ~AgentIDManager();

    std::unordered_set<const IAgentID *> agentIDs;
    IAgentIDFactory *idFactory;
};

template <class IDType>
const IDType *AgentIDManager::createIDFromBytes(const std::vector<uint8_t> &idByteVector)
{
    const IDType *tmpID = this->idFactory->create(idByteVector);
    auto entry = this->agentIDs.insert(tmpID);
    if (!entry.second)
    {
        delete tmpID;
    }

    return entry.first;
}

template <class IDType, class Prototype>
const IDType *AgentIDManager::createID(Prototype &idPrototype)
{
    std::vector<uint8_t> idByteVector;
    for (int i = 0; i < sizeof(Prototype); i++)
    {
    	idByteVector.push_back(*(((uint8_t *)&idPrototype) + i));
    }
    return this->createIDFromBytes<IDType>(idByteVector);
}



} /* namespace supplementary */
