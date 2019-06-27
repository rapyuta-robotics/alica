#pragma once
#include <iostream>
#include <vector>

namespace essentials
{

class AgentID
{
    friend struct std::hash<essentials::AgentID>;

  public:
    AgentID()
        : _type(UUID_TYPE)
    {
    }
    AgentID(const std::vector<uint8_t>& bytes);
    AgentID(const uint8_t* idBytes, int idSize, uint8_t type = UUID_TYPE);
    virtual ~AgentID();
    virtual bool operator==(const AgentID& obj) const;
    virtual bool operator!=(const AgentID& obj) const;
    virtual bool operator<(const AgentID& other) const;
    virtual bool operator>(const AgentID& other) const;
    void operator=(const std::vector<uint8_t>& other_id);
    operator bool() const { return !_id.empty(); }
    operator unsigned long long() const;

    virtual const uint8_t* getRaw() const;
    virtual int getSize() const;
    virtual std::vector<uint8_t> toByteVector() const;
    virtual std::size_t hash() const;
    virtual uint8_t getType() const;

    friend std::ostream& operator<<(std::ostream& os, const essentials::AgentID& obj)
    {
        if (obj._id.size() <= sizeof(int)) {
            std::vector<uint8_t> tmpLong;
            for (int i = 0; i < static_cast<int>(obj._id.size()); i++) {
                tmpLong.push_back(obj._id[i]);
            }
            for (int i = 0; i < static_cast<int>(sizeof(int) - obj._id.size()); i++) {
                tmpLong.push_back(0);
            }
            os << (int)(*tmpLong.data());
        } else {
            std::vector<uint8_t> tmpShort;
            tmpShort.push_back(obj._id[0]);
            tmpShort.push_back(obj._id[1]);
            os << (short)(*tmpShort.data()) << "[...]";
            tmpShort.clear();
            tmpShort.push_back(obj._id[obj._id.size() - 2]);
            tmpShort.push_back(obj._id[obj._id.size() - 1]);
            os << (short)(*tmpShort.data());
        }
        return os;
    }

    static const uint8_t INT_TYPE = 0;
    static const uint8_t BC_TYPE = 1;
    static const uint8_t UUID_TYPE = 2;

  private:
    std::vector<uint8_t> _id;
    uint8_t _type;
};

struct AgentIDComparator
{
    bool operator()(const AgentID* a, const AgentID* b) const { return *a < *b; }
};

struct AgentIDEqualsComparator
{
    bool operator()(const AgentID* const a, const AgentID* b) const { return *a == *b; }
};

struct AgentIDHash
{
    std::size_t operator()(const AgentID* const obj) const { return obj->hash(); }
};
} /* namespace essentials */

namespace std
{
template <>
struct hash<essentials::AgentID>
{
    std::size_t operator()(const essentials::AgentID& id) const noexcept { return id.hash(); }
};
}
