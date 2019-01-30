#include "essentials/AgentID.h"

namespace essentials
{

AgentID::AgentID(const uint8_t* idBytes, int idSize, uint8_t type)
        : TYPE(type)
{
    for (int i = 0; i < idSize; i++) {
        this->id.push_back(idBytes[i]);
    }
}

AgentID::~AgentID(){};

uint8_t AgentID::getType() const
{
    return this->TYPE;
}

bool AgentID::operator==(const AgentID& other) const
{
    if (this->id.size() != other.id.size()) {
        return false;
    }
    for (int i = 0; i < static_cast<int>(this->id.size()); i++) {
        if (this->id[i] != other.id[i]) {
            return false;
        }
    }
    return true;
}

bool AgentID::operator!=(const AgentID& other) const
{
    return !(*this == other);
}

bool AgentID::operator<(const AgentID& other) const
{
    if (this->id.size() < other.id.size()) {
        return true;
    } else if (this->id.size() > other.id.size()) {
        return false;
    }
    for (int i = 0; i < this->id.size(); i++) {
        if (this->id[i] < other.id[i]) {
            return true;
        } else if (this->id[i] > other.id[i]) {
            return false;
        }
        // else continue, because both bytes where equal and the next byte needs to be considered
    }
    return false;
}

bool AgentID::operator>(const AgentID& other) const
{
    if (this->id.size() > other.id.size()) {
        return true;
    } else if (this->id.size() < other.id.size()) {
        return false;
    }
    for (int i = 0; i < this->id.size(); i++) {
        if (this->id[i] > other.id[i]) {
            return true;
        } else if (this->id[i] < other.id[i]) {
            return false;
        }
        // else continue, because both bytes where equal and the next byte needs to be considered
    }
    return false;
}

const uint8_t* AgentID::getRaw() const
{
    return this->id.data();
}

int AgentID::getSize() const
{
    return this->id.size();
}

std::vector<uint8_t> AgentID::toByteVector() const
{
    return this->id;
}

/**
 * See:
 * https://en.wikipedia.org/wiki/MurmurHash
 * https://softwareengineering.stackexchange.com/questions/49550/which-hashing-algorithm-is-best-for-uniqueness-and-speed
 */
std::size_t AgentID::hash() const
{
    const uint8_t* key = this->id.data();
    int len = this->id.size();
    uint32_t h = 13;
    if (len > 3) {
        const uint32_t* key_x4 = (const uint32_t*)key;
        size_t i = this->id.size() >> 2;
        do {
            uint32_t k = *key_x4++;
            k *= 0xcc9e2d51;
            k = (k << 15) | (k >> 17);
            k *= 0x1b873593;
            h ^= k;
            h = (h << 13) | (h >> 19);
            h = (h * 5) + 0xe6546b64;
        } while (--i);
        key = (const uint8_t*)key_x4;
    }
    if (len & 3) {
        size_t i = len & 3;
        uint32_t k = 0;
        key = &key[i - 1];
        do {
            k <<= 8;
            k |= *key--;
        } while (--i);
        k *= 0xcc9e2d51;
        k = (k << 15) | (k >> 17);
        k *= 0x1b873593;
        h ^= k;
    }
    h ^= len;
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}

} /* namespace essentials */
