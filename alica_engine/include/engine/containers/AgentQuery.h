#pragma once

namespace alica
{
struct AgentQuery
{
    AgentQuery()
            : senderID(0)
            , senderSdk(0)
            , planHash(0)
    {
    }
    uint64_t senderID;
    uint32_t senderSdk;
    uint32_t planHash;
};
} // namespace alica
