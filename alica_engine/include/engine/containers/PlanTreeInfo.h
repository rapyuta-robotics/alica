#pragma once

#include <engine/Types.h>

#include <tuple>
#include <vector>

namespace alica
{
typedef std::tuple<AgentId, std::vector<int64_t>, std::vector<std::size_t>> stdPlanTreeInfo;
struct PlanTreeInfo
{
    PlanTreeInfo()
            : senderID(InvalidAgentID)
    {
    }

    PlanTreeInfo(const stdPlanTreeInfo& s)
            : senderID(std::get<0>(s))
            , dynamicStateIDPairs(std::get<1>(s))
            , succeededContexts(std::get<2>(s))
    {
    }

    AgentId senderID;
    std::vector<int64_t> dynamicStateIDPairs;
    std::vector<std::size_t> succeededContexts;

    stdPlanTreeInfo toStandard() const { return std::make_tuple(senderID, dynamicStateIDPairs, succeededContexts); }
};
} /* namespace alica*/
