#pragma once

#include <tuple>
#include <vector>

namespace alica
{
typedef std::tuple<AgentIDConstPtr, std::vector<int64_t>, std::vector<int64_t>> stdPlanTreeInfo;
struct PlanTreeInfo
{
    PlanTreeInfo()
            : senderID(nullptr)
    {
    }

    PlanTreeInfo(const stdPlanTreeInfo& s)
            : senderID(std::get<0>(s))
            , stateIDs(std::get<1>(s))
            , succeededEPs(std::get<2>(s))
    {
    }

    AgentIDConstPtr senderID;
    std::vector<int64_t> stateIDs;
    std::vector<int64_t> succeededEPs;

    stdPlanTreeInfo toStandard() const { return std::make_tuple(senderID, stateIDs, succeededEPs); }
};
} /* namespace alica*/
