#pragma once

#include "essentials/AgentID.h"

#include <iostream>

namespace essentials
{

class BroadcastID : public essentials::AgentID
{
  public:
    BroadcastID(const uint8_t* idBytes, int idSize);
    virtual ~BroadcastID();

    std::string toString() const;
    std::size_t hash() const;
    bool operator==(const essentials::AgentID& obj) const;
    bool operator!=(const essentials::AgentID& obj) const;
    bool operator<(const essentials::AgentID& other) const;
    bool operator>(const essentials::AgentID& other) const;
};
} /* namespace supplementary */
