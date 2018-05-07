#pragma once

#include "supplementary/AgentID.h"

#include <iostream>

namespace supplementary
{

class BroadcastID : public supplementary::AgentID
{
  public:
    BroadcastID(const uint8_t *idBytes, int idSize);
    virtual ~BroadcastID();

    std::string toString() const;
    std::size_t hash() const;
    bool operator==(const supplementary::AgentID &obj) const;
    bool operator!=(const supplementary::AgentID &obj) const;
    bool operator<(const supplementary::AgentID &other) const;
    bool operator>(const supplementary::AgentID &other) const;

};
} /* namespace supplementary */
