#pragma once

namespace alica
{
typedef std::tuple<essentials::AgentIDConstPtr, int64_t> stdRoleSwitch;
struct RoleSwitch
{
    RoleSwitch()
            : senderID(nullptr)
    {
    }

    RoleSwitch(const stdRoleSwitch& s)
            : senderID(std::get<0>(s))
            , roleID(std::get<1>(s))
    {
    }

    essentials::AgentIDConstPtr senderID;
    int64_t roleID;

    stdRoleSwitch toStandard() const { return std::make_tuple(senderID, roleID); }
};

} /* namespace alica */
