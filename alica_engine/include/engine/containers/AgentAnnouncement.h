#pragma once

#include <engine/Types.h>

#include <string>
#include <tuple>
#include <vector>

namespace alica
{
struct AgentAnnouncement
{
    using CapabilityPair = std::pair<std::string, std::string>;
    AgentAnnouncement()
            : senderID(InvalidAgentID)
            , token(0)
            , senderSdk(0)
            , planHash(0)
            , roleId(0)
    {
    }
    AgentId senderID;
    uint32_t token;
    std::string senderName;
    uint32_t senderSdk;
    uint64_t planHash;
    int64_t roleId;
    // Agent capabilities
    std::vector<CapabilityPair> capabilities;
};
} // namespace alica
