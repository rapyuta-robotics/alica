#pragma once

namespace alica
{
typedef std::tuple<uint64_t, int64_t> stdRoleSwitch;
struct RoleSwitch
{
    RoleSwitch()
            : senderID(0)
            , roleID(0)
    {
    }

    RoleSwitch(const stdRoleSwitch& s)
            : senderID(std::get<0>(s))
            , roleID(std::get<1>(s))
    {
    }

    uint64_t senderID;
    int64_t roleID;

    stdRoleSwitch toStandard() const { return std::make_tuple(senderID, roleID); }
};

} /* namespace alica */
