#pragma once

#include "AgentID.h"

namespace essentials
{

class AgentIDFactory
{
public:
    AgentIDFactory();
    virtual ~AgentIDFactory();
    virtual const AgentID* create(const std::vector<uint8_t>& bytes) const;
    virtual const AgentID* generateID(int size = 16) const;
};

} /* namespace essentials */
