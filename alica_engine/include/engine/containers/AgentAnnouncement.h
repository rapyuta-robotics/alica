#pragma once

#include "engine/AgentIDConstPtr.h"

#include <string>
#include <tuple>
#include <vector>

namespace alica
{
struct AgentAnnouncement
{
    using CapabilityPair = std::pair<std::string, std::string>;
    AgentAnnouncement()
            : senderID(nullptr)
    {
    }
    AgentIDConstPtr senderID;
    std::string senderName;
    uint32_t senderSdk;
    uint32_t planHash;
    std::string role;
    // Agent capabilities
    std::vector<CapabilityPair> capabilities;
};
} // namespace alica
