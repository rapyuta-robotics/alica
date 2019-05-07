#pragma once

#include "engine/AgentIDConstPtr.h"

namespace alica
{
struct AgentQuery
{
    AgentQuery()
            : senderID(nullptr)
            , senderSdk(0)
            , planHash(0)
    {
    }
    AgentIDConstPtr senderID;
    uint32_t senderSdk;
    uint32_t planHash;
};
} // namespace alica
