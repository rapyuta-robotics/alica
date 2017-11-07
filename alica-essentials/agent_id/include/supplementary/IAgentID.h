#pragma once
#include <iostream>
#include <vector>

namespace supplementary
{

class IAgentID
{
	friend struct std::hash<supplementary::IAgentID>;
  public:
    virtual ~IAgentID(){};
    virtual bool operator==(const IAgentID &obj) const = 0;
    virtual bool operator!=(const IAgentID &obj) const = 0;
    virtual bool operator<(const IAgentID &other) const = 0;
    virtual bool operator>(const IAgentID &other) const = 0;
    virtual uint8_t *getRaw() const = 0;
    virtual int getSize() const = 0;
    virtual uint8_t getType() const = 0;
    virtual std::vector<uint8_t> toByteVector() const = 0;
    virtual std::string toString() const = 0;
    virtual std::size_t hash() const = 0;

    friend std::ostream &operator<<(std::ostream &os, const supplementary::IAgentID &obj)
    {
        os << obj.toString();
        return os;
    }
};

struct IAgentIDComparator
{
    bool operator()(const IAgentID *a, const IAgentID *b) const
    {
        return *a < *b;
    }
};

struct IAgentIDEqualsComparator
{
    bool operator()(const IAgentID *const &a, const IAgentID *&b) const
    {
        return *a == *b;
    }
};

struct IAgentIDHash
{
    std::size_t operator()(const supplementary::IAgentID* const &obj) const
    {
    	return obj->hash();
    }
};

} /* namespace supplementary */


