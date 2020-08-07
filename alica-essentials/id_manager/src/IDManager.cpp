#include "essentials/IDManager.h"

#include <uuid/uuid.h>

namespace essentials
{
IDManager::IDManager() {
    this->wildcardId = new WildcardID(nullptr, 0);
    this->ids.insert(this->wildcardId);
}
IDManager::~IDManager()
{
    for (auto& id : this->ids) {
        delete id;
    }
}

const essentials::Identifier* IDManager::getIDFromBytes(const uint8_t *idBytes, int idSize, uint8_t type)
{
    if (type == essentials::Identifier::WILDCARD_TYPE) {
        return this->wildcardId;
    }

    if (idBytes == 0) {
        // empty values result in none-id
        return nullptr;
    }

    // create tmpID for lookup the ID
    const essentials::Identifier* tmpID = new essentials::Identifier(idBytes, idSize, type);

    // make the manager thread-safe
    std::lock_guard<std::mutex> guard(idsMutex);

    // lookup the ID and insert it, if not available, yet
    auto entry = this->ids.insert(tmpID);
    if (!entry.second) { // delete tmpID if already present in IDs
        delete tmpID;
    }
    return *(entry.first);
}

const essentials::Identifier* IDManager::generateID(int size)
{
    uuid_t uuid; // a UUID is 16 bytes long
    if (size <= 16) {
        uuid_generate(uuid);
        return this->getIDFromBytes(uuid, size);
    } else { // in case you need an id which is longer than 16 bytes
        std::vector<uint8_t> bytes;
        while (bytes.size() < size) {
            uuid_generate(uuid);
            for (int i = 0; i < 16; i++) {
                bytes.push_back(uuid[i]);
            }
        }
        return this->getIDFromBytes(bytes.data(), size);
    }
}

const essentials::Identifier* IDManager::getWildcardID() const {
    return this->wildcardId;
}

} // namespace essentials