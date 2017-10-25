#pragma once
#include <iostream>
#include <vector>

namespace supplementary
{

class IAgentID
{
  public:
    virtual ~IAgentID(){};
    virtual bool operator==(const IAgentID &obj) const = 0;
    virtual bool operator!=(const IAgentID &obj) const = 0;
    virtual bool operator<(const IAgentID &other) const = 0;
    virtual bool operator>(const IAgentID &other) const = 0;
    virtual uint8_t *getRaw() const = 0;
    virtual int getSize() const = 0;
    virtual std::vector<uint8_t> toByteVector() const = 0;
    virtual std::string toString() const = 0;

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

} /* namespace supplementary */

namespace std
{
template <>
struct hash<supplementary::IAgentID>
{
    typedef const supplementary::IAgentID &argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type &pa) const
    {
        return 0;
    }
};
}
