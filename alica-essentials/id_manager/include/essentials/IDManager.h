#pragma once

#include "essentials/Identifier.h"
#include "WildcardID.h"

#include <mutex>
#include <unordered_set>
#include <vector>
#include <bitset>

namespace essentials
{

class IDManager
{
public:
    IDManager();
    virtual ~IDManager();

    /**
     * If present, returns the ID corresponding to the given bytes.
     * Otherwise, it creates a new one, stores and returns it.
     *
     * This method can be used, e.g., for passing a part of a ROS
     * message and receiving a pointer to a corresponding ID object.
     */
    const essentials::Identifier* getIDFromBytes(const uint8_t* idBytes, int idSize, uint8_t type = Identifier::UUID_TYPE);
    template <class Prototype>
    const Identifier* getID(Prototype& idPrototype, uint8_t type = Identifier::UUID_TYPE);
    const Identifier* generateID(int size = 16);
    const Identifier* getWildcardID();
private:
    std::unordered_set<const Identifier*, essentials::IdentifierHash, essentials::IdentifierEqualsComparator> ids;
    std::mutex idsMutex;
    WildcardID* wildcardId;
};

/**
 * If present, returns the ID corresponding to the given prototype.
 * Otherwise, it creates a new one, stores and returns it.
 *
 * This method can be used, e.g., for passing an int and receiving
 * a pointer to a corresponding ID object.
 */
template <class Prototype>
const Identifier* IDManager::getID(Prototype& idPrototype, uint8_t type)
{
    // little-endian encoding
    std::vector<uint8_t> idByteVector;
    // TODO: replace with memcpy or std copy
    for (int i = 0; i < static_cast<int>(sizeof(Prototype)); i++) {
        idByteVector.push_back(*(((uint8_t*) &idPrototype) + i));
    }

    return this->getIDFromBytes(idByteVector.data(), idByteVector.size(), type);
}

} // namespace essentials
