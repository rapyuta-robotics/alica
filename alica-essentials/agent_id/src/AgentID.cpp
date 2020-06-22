#include "essentials/AgentID.h"

#include <sstream>

namespace essentials
{

AgentID::AgentID(uint64_t prototypeID)
    : _type(UUID_TYPE)
{
    setID(prototypeID);
}

AgentID::AgentID(const std::vector<uint8_t>& bytes)
    : _type(UUID_TYPE)
    , _id(bytes)
{
}

AgentID::AgentID(const uint8_t* idBytes, int idSize, uint8_t type)
    : _type(type)
{
    for (int i = 0; i < idSize; i++) {
        _id.push_back(idBytes[i]);
    }
}

AgentID::~AgentID(){};

uint8_t AgentID::getType() const
{
    return _type;
}

bool AgentID::operator==(const AgentID& other) const
{
    return (_id == other._id);
}

bool AgentID::operator!=(const AgentID& other) const
{
    return (_id != other._id);
}

bool AgentID::operator<(const AgentID& other) const
{
    if (_id.size() < other._id.size()) {
        return true;
    } else if (_id.size() > other._id.size()) {
        return false;
    }
    for (int i = 0; i < _id.size(); i++) {
        if (_id[i] < other._id[i]) {
            return true;
        } else if (_id[i] > other._id[i]) {
            return false;
        }
        // else continue, because both bytes where equal and the next byte needs to be considered
    }
    return false;
}

bool AgentID::operator>(const AgentID& other) const
{
    return other < *this;
}

void AgentID::operator=(const std::vector<uint8_t>& other_id)
{
    _id = other_id;
    _type = UUID_TYPE;
}

AgentID::operator uint64_t() const
{
    // check that length of id fits into uint64_t
    if (sizeof(uint64_t) < _id.size()) {
        std::stringstream ss;
        ss << "Conversion of ID " << *this << " to uint64_t is not allowed, because (ID.size() = " << _id.size()
           << " bytes) > (sizeof(uint64_t) = " << sizeof(uint64_t) << " bytes)!";
        throw ss.str();
    }

    // Determine the endianness
    constexpr short n = 1;
    bool isLittleEndian = *((char*)(&n)) == 1;

    // moving data
    uint64_t out = 0;
    if (isLittleEndian) {
        std::copy(_id.begin(), _id.end(), (uint8_t*)(&out));
    } else {
        /**
         * Note: This is not changing the order of bytes, as it should happen for a different endianness.
         * Instead it simply moves the bytes at the beginning or end of the range of the uint64_t.
         */
        int offset = static_cast<int>(sizeof(uint64_t) - _id.size());
        std::copy(_id.begin(), _id.end(), (uint8_t*)(&out) + offset);
    }
    return out;
}

const uint8_t* AgentID::getRaw() const
{
    return _id.data();
}

int AgentID::getSize() const
{
    return _id.size();
}

std::vector<uint8_t> AgentID::toByteVector() const
{
    return _id;
}

/**
 * See:
 * https://en.wikipedia.org/wiki/MurmurHash
 * https://softwareengineering.stackexchange.com/questions/49550/which-hashing-algorithm-is-best-for-uniqueness-and-speed
 */
std::size_t AgentID::hash() const
{
    const uint8_t* key = _id.data();
    int len = _id.size();
    uint32_t h = 13;
    if (len > 3) {
        const uint32_t* key_x4 = (const uint32_t*)key;
        size_t i = _id.size() >> 2;
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
