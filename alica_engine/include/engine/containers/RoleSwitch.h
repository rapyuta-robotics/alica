#pragma once

namespace alica
{
typedef std::tuple<essentials::IdentifierConstPtr, int64_t> stdRoleSwitch;
struct RoleSwitch
{
    RoleSwitch()
            : senderID(nullptr)
            , roleID(0)
    {
    }

    RoleSwitch(const stdRoleSwitch& s)
            : senderID(std::get<0>(s))
            , roleID(std::get<1>(s))
    {
    }

    essentials::IdentifierConstPtr senderID;
    int64_t roleID;

    stdRoleSwitch toStandard() const { return std::make_tuple(senderID, roleID); }
};

} /* namespace alica */
