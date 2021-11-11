#pragma once

#include "EntryPointRobots.h"

#include <iterator>
#include <ostream>
#include <tuple>
#include <vector>

namespace alica
{

typedef std::tuple<alica::AgentId, int64_t, int64_t, int64_t, alica::AgentId, std::vector<stdEntryPointRobot>> stdAllocationAuthorityInfo;
struct AllocationAuthorityInfo
{
    AllocationAuthorityInfo()
            : senderID(0)
            , planId(0)
            , parentState(0)
            , planType(0)
            , authority(0)
    {
    }

    alica::AgentId senderID;
    int64_t planId;
    int64_t parentState;
    int64_t planType;
    alica::AgentId authority;
    std::vector<EntryPointRobots> entryPointRobots;

    AllocationAuthorityInfo(const stdAllocationAuthorityInfo& s)
    {
        senderID = std::get<0>(s);
        planId = std::get<1>(s);
        parentState = std::get<2>(s);
        planType = std::get<3>(s);
        authority = std::get<4>(s);
        const std::vector<stdEntryPointRobot>& tmp = std::get<5>(s);
        for (const stdEntryPointRobot& e : tmp) {
            entryPointRobots.push_back(EntryPointRobots(e));
        }
    }

    stdAllocationAuthorityInfo toStandard() const
    {
        std::vector<stdEntryPointRobot> r;
        for (const EntryPointRobots& e : entryPointRobots) {
            r.push_back(e.toStandard());
        }
        return std::make_tuple(senderID, planId, parentState, planType, authority, std::move(r));
    }
};

inline std::ostream& operator<<(std::ostream& o, const AllocationAuthorityInfo& aai)
{
    o << "AAI sender: " << aai.senderID << " plan: " << aai.planId << std::endl;
    std::copy(aai.entryPointRobots.begin(), aai.entryPointRobots.end(), std::ostream_iterator<EntryPointRobots>(o, "\n"));
    return o;
}

} /* namespace alica */
