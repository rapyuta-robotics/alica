#pragma once

#include "essentials/AgentID.h"
#include "essentials/AgentIDFactory.h"

#include <mutex>
#include <unordered_set>
#include <vector>

namespace essentials
{

class AgentIDManager
{
public:
    // static AgentIDManager *getInstance();
    AgentIDManager(AgentIDFactory* idFactory);
    virtual ~AgentIDManager();

    const AgentID* getIDFromBytes(const std::vector<uint8_t>& vectorID);

    template <class Prototype>
    const AgentID* getID(Prototype& idPrototype);

    const AgentID* generateID(int size = 16);

private:
    std::unordered_set<const AgentID*, essentials::AgentIDHash, essentials::AgentIDEqualsComparator> agentIDs;
    AgentIDFactory* idFactory;
    std::mutex mutex;
};

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing an int and receiving
 * a pointer to a corresponding AgentID object.
 */
template <class Prototype>
const AgentID* AgentIDManager::getID(Prototype& idPrototype)
{
    // little-endian encoding
    std::vector<uint8_t> idByteVector;
    // TODO: replace with memcpy or std copy
    for (int i = 0; i < static_cast<int>(sizeof(Prototype)); i++) {
        idByteVector.push_back(*(((uint8_t*)&idPrototype) + i));
    }
    return this->getIDFromBytes(idByteVector);
}

} /* namespace essentials */
