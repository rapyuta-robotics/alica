#pragma once

#include <engine/Types.h>

namespace alica
{
typedef std::tuple<AgentId, int64_t> stdRoleSwitch;
struct RoleSwitch
{
    RoleSwitch()
            : senderID(InvalidAgentID)
            , roleID(InvalidAgentID)
    {
    }

    RoleSwitch(const stdRoleSwitch& s)
            : senderID(std::get<0>(s))
            , roleID(std::get<1>(s))
    {
    }

    AgentId senderID;
    int64_t roleID;

    stdRoleSwitch toStandard() const { return std::make_tuple(senderID, roleID); }
};

} /* namespace alica */
