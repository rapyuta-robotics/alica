#pragma once
#include <iostream>
#include <vector>

namespace supplementary
{

class AgentID
{
    friend struct std::hash<supplementary::AgentID>;

  public:
    AgentID(const uint8_t *idBytes, int idSize, uint8_t type = UUID_TYPE);
    virtual ~AgentID();
    virtual bool operator==(const AgentID &obj) const;
    virtual bool operator!=(const AgentID &obj) const;
    virtual bool operator<(const AgentID &other) const;
    virtual bool operator>(const AgentID &other) const;
    virtual const uint8_t *getRaw() const;
    virtual int getSize() const;
    virtual std::vector<uint8_t> toByteVector() const;
    virtual std::size_t hash() const;
    virtual uint8_t getType() const;

    friend std::ostream &operator<<(std::ostream &os, const supplementary::AgentID &obj)
    {
        if (obj.id.size() <= sizeof(int))
        {
            std::vector<uint8_t> tmpLong;
            for (int i = 0; i < obj.id.size(); i++)
            {
                tmpLong.push_back(obj.id[i]);
            }
            for (int i = 0; i < sizeof(int) - obj.id.size(); i++)
            {
                tmpLong.push_back(0);
            }
            os << (int)(*tmpLong.data());
        }
        else
        {
            std::vector<uint8_t> tmpShort;
            tmpShort.push_back(obj.id[0]);
            tmpShort.push_back(obj.id[1]);
            os << (short)(*tmpShort.data()) << "[...]";
            tmpShort.clear();
            tmpShort.push_back(obj.id[obj.id.size() - 2]);
            tmpShort.push_back(obj.id[obj.id.size() - 1]);
            os << (short)(*tmpShort.data());
        }
        return os;
    }

    const uint8_t TYPE;

    static const uint8_t INT_TYPE = 0;
    static const uint8_t BC_TYPE = 1;
    static const uint8_t UUID_TYPE = 2;

  private:
    std::vector<uint8_t> id;
};

struct AgentIDComparator
{
    bool operator()(const AgentID *a, const AgentID *b) const
    {
        return *a < *b;
    }
};

struct AgentIDEqualsComparator
{
    AgentID *test;
    bool operator()(const AgentID *const &a, const AgentID *&b) const
    {
        return *a == *b;
    }
};

struct AgentIDHash
{
    std::size_t operator()(const supplementary::AgentID *const &obj) const
    {
        return obj->hash();
    }
};

} /* namespace supplementary */
