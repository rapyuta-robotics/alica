#pragma once

#include <engine/Types.h>

#include <tuple>
#include <vector>

namespace alica
{
typedef std::tuple<AgentId, std::vector<int64_t>, std::vector<int64_t>> stdPlanTreeInfo;
struct PlanTreeInfo
{
    PlanTreeInfo()
            : senderID(InvalidAgentID)
    {
    }

    PlanTreeInfo(const stdPlanTreeInfo& s)
            : senderID(std::get<0>(s))
            , stateIDs(std::get<1>(s))
            , succeededEPs(std::get<2>(s))
    {
    }

    AgentId senderID;
    std::vector<int64_t> stateIDs;
    std::vector<int64_t> succeededEPs;

    stdPlanTreeInfo toStandard() const { return std::make_tuple(senderID, stateIDs, succeededEPs); }
};
} /* namespace alica*/
