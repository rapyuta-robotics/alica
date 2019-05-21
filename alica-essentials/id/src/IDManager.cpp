#include "essentials/IDManager.h"

#include <uuid/uuid.h>

namespace essentials
{
IDManager::IDManager() {}
IDManager::~IDManager() {
    for (auto& id : this->ids) {
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
const essentials::Identifier* IDManager::getIDFromBytes(const std::vector<uint8_t>& vectorID)
{
    if (vectorID.empty()) { // empty values result in none-id
        return nullptr;
    }
    // create tmpID for lookup the ID
    const essentials::Identifier* tmpID = new essentials::Identifier(vectorID.data(), vectorID.size());

    // make the manager thread-safe
    std::lock_guard<std::mutex> guard(mutex);

    // lookup the ID and insert it, if not available, yet
    auto entry = this->ids.insert(tmpID);
    if (!entry.second) { // delete tmpID if already present in agentIDs
        delete tmpID;
    }
    return *(entry.first);
}

const essentials::Identifier* IDManager::generateID(int size) const
{
    uuid_t uuid; // a UUID is 16 bytes long
    if (size <= 16) {
        uuid_generate(uuid);
        return new essentials::Identifier(uuid, size);
    } else { // in case you need an id which is longer than 16 bytes
        std::vector<uint8_t> bytes;
        while (bytes.size() < size) {
            uuid_generate(uuid);
            for (int i = 0; i < 16; i++) {
                bytes.push_back(uuid[i]);
            }
        }
        return new essentials::Identifier(bytes.data(), size);
    }
}

const essentials::Identifier* IDManager::create(const std::vector<uint8_t>& bytes) const
{
    return new essentials::Identifier(bytes.data(), bytes.size());
}

} // namespace srgsim