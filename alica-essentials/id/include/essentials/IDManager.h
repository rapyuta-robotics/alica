#pragma once

#include <essentials/ID.h>

#include <mutex>
#include <unordered_set>
#include <vector>

namespace essentials
{
class IDManager
{
public:
    IDManager();
    virtual ~IDManager();

    /**
     * If present, returns the ID corresponding to the given prototype.
     * Otherwise, it creates a new one, stores and returns it.
     *
     * This method can be used, e.g., for passing a part of a ROS
     * message and receiving a pointer to a corresponding IAgentID object.
     */
    const essentials::ID* getIDFromBytes(const std::vector<uint8_t>& vectorID);
    template <class Prototype>
    const ID* getID(Prototype& idPrototype);
    const ID* generateID(int size = 16) const;
    const ID* create(const std::vector<uint8_t>& bytes) const;

private:
    std::unordered_set<const ID*, essentials::IDHash, essentials::IDEqualsComparator> ids;
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
const ID* IDManager::getID(Prototype& idPrototype)
{
    // little-endian encoding
    std::vector<uint8_t> idByteVector;
    // TODO: replace with memcpy or std copy
    for (int i = 0; i < static_cast<int>(sizeof(Prototype)); i++) {
        idByteVector.push_back(*(((uint8_t*) &idPrototype) + i));
    }
    return this->getIDFromBytes(idByteVector);
}

} // namespace essentials
