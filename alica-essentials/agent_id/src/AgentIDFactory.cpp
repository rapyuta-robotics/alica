#include "supplementary/AgentIDFactory.h"

#include <uuid/uuid.h>

#include <random>

namespace supplementary {

AgentIDFactory::AgentIDFactory() {}

AgentIDFactory::~AgentIDFactory() {}

const AgentID* AgentIDFactory::create(const std::vector<uint8_t>& bytes) const {
    return new AgentID(bytes.data(), bytes.size());
}

const AgentID* AgentIDFactory::generateID(int size) const {
    uuid_t uuid;  // a UUID is 16 bytes long
    uuid_generate(uuid);

    if (size <= 16) {
        return new AgentID(uuid, size);
    } else {  // in case you need an id which is longer than 16 bytes
        std::vector<uint8_t> bytes;
        while (bytes.size() < size) {
            for (int i = 0; i < 16; i++) {
                bytes.push_back(uuid[i]);
            }
        }
        return new AgentID(bytes.data(), size);
    }
}

} /* namespace supplementary */
