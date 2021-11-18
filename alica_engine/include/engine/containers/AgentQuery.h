#pragma once

#include "engine/Types.h"

namespace alica
{
struct AgentQuery
{
    AgentQuery()
            : senderID(InvalidAgentID)
            , senderSdk(0)
            , planHash(0)
    {
    }
    AgentId senderID;
    uint32_t senderSdk;
    uint32_t planHash;
};
} // namespace alica
