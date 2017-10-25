#pragma once

#include "supplementary/IAgentID.h"

#include <iostream>

namespace supplementary
{

class BroadcastID : public supplementary::IAgentID
{
    friend struct std::hash<supplementary::BroadcastID>;

  public:
    BroadcastID(const uint8_t *idBytes, int idSize);
    virtual ~BroadcastID();

    uint8_t *getRaw() const;
    int getSize() const;

    std::string toString() const;
    bool operator==(const supplementary::IAgentID &obj) const;
    bool operator!=(const supplementary::IAgentID &obj) const;
    bool operator<(const supplementary::IAgentID &other) const;
    bool operator>(const supplementary::IAgentID &other) const;

    std::vector<uint8_t> toByteVector() const;

  private:
    uint8_t id;
};
} /* namespace supplementary */

namespace std
{
template <>
struct hash<supplementary::BroadcastID>
{
    typedef const supplementary::BroadcastID &argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type &pa) const
    {
        return std::hash<uint8_t>()(0);
    }
};
}
