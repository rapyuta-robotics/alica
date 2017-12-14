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
    virtual std::string toString() const;
    virtual std::size_t hash() const;
    virtual uint8_t getType() const;

    friend std::ostream &operator<<(std::ostream &os, const supplementary::AgentID &obj)
    {
    	for (auto byte : obj.id)
    	{
    		os << byte << " ";
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
