#pragma once

#include <essentials/IdentifierConstPtr.h>

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
    essentials::IdentifierConstPtr senderID;
    uint32_t senderSdk;
    uint32_t planHash;
};
} // namespace alica
